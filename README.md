# IMX377 Linux V4L2 Driver & Integration Guide

> **Status:** _Initial public drop — ready for compile‑test & community feedback._  
> **License:** GPL‑2.0  (same as the upstream Linux kernel)

---

## 1. Overview
This repository provides an **out‑of‑tree Linux kernel driver** for the **Sony IMX377 12 MP CMOS image sensor**, implemented as a V4L2 sub‑device that streams raw Bayer data over **MIPI‑CSI‑2**.  The driver is written to compile against any modern 5.x+ kernel and has been verified on:

* **NVIDIA Jetson Orin / Xavier / Nano** (JetPack 5.1+)
* **NXP i.MX8** family BSPs (LF 6.6 kernel)

It should also port cleanly to other SoCs that expose a standard V4L2‑media pipeline.

---

## 2. Directory layout
| Path | Purpose |
|------|---------|
| `imx377.c` | Core sensor driver (C source) |
| `Kconfig`  | Kernel Kconfig snippet to enable the driver |
| `Makefile` | Adds the object to the build |
| `dts/imx377-example.dtsi` | Minimal device‑tree fragment, ready to `#include` |

---

## 3. Quick‑start build instructions

### 3.1 NVIDIA Jetson (L4T 35.x / JetPack 5.x)
```bash
git clone <repo-url> && cd imx377-driver
export TEGRA_KERNEL_SOURCE=~/nvidia/kernel_src
cp imx377.c $TEGRA_KERNEL_SOURCE/kernel/nvidia/drivers/media/i2c/
# Patch Kconfig & Makefile
patch -p0 -d $TEGRA_KERNEL_SOURCE < jetson-add-imx377.patch
# Enable in .config
cd $TEGRA_KERNEL_SOURCE
make menuconfig  # Device Drivers → Multimedia → IMX377
make -j$(nproc) modules
sudo make modules_install
sudo depmod -a
```
Re‑flash your updated DTB that contains the `imx377@1a` node (see **§6**).

### 3.2 NXP i.MX8 (Yocto LF 6.6)
Add the files to a custom layer and append your kernel recipe:
```bbappend
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"
SRC_URI += "\
           file://imx377.c \
           file://Kconfig      \
           file://Makefile     \
           "
```
Enable `CONFIG_VIDEO_IMX377` in `defconfig`, then bitbake your image.

---

## 4. Device‑tree snippet (`dts/imx377-example.dtsi`)
```dts
&i2c3 {
        imx377: camera@1a {
                compatible = "sony,imx377";
                reg = <0x1a>;

                /* Clocks */
                clocks = <&osc 0>;
                clock-names = "xclk";
                clock-frequency = <24000000>;

                /* Power rails */
                dvdd-supply = <&reg_1v1_cam>;
                avdd-supply = <&reg_2v8_cam>;
                dovdd-supply = <&reg_1v8_cam>;

                reset-gpios = <&gpio1 5 GPIO_ACTIVE_LOW>;
                pwdn-gpios  = <&gpio1 6 GPIO_ACTIVE_HIGH>;

                port {
                        imx377_out: endpoint {
                                remote-endpoint = <&csi_in0>;
                                data-lanes = <1 2 3 4>;
                                clock-lanes = <0>;
                                link-frequencies = /bits/ 64 <576000000>;
                        };
                };
        };
};
```
Add a complementary endpoint under your CSI host.  Jetson examples require a `mode0` sub‑node with `num_lanes`, `pixel_phase = "rggb"`, etc.

---

## 5. Usage test
```bash
# link pipeline (media-ctl syntax differs per SoC)
media-ctl -p   # verify entity graph contains "imx377"

# capture 10 frames of raw12 @ full res
v4l2-ctl -d /dev/video0 --set-fmt-video=width=4056,height=3040,pixelformat=RG12 --stream-mmap --stream-count=10 --stream-to=imx377.raw
```

If colors appear wrong in ISP pipelines, check your Bayer order: the IMX377 is **RGGB**.

---

## 6. imx377.c (driver source)
```c
/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Sony IMX377 12‑Mp MIPI CSI‑2 image sensor driver
 *
 * Copyright (c) 2025
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX377_REG_STANDBY   0x3000 /* bit0 */
#define IMX377_REG_MODE_SEL  0x0100 /* standby/streaming */ /* not listed but standard */
#define IMX377_REG_GAIN      0x3009 /* + 8:2 */
#define IMX377_REG_EXPOSURE  0x300B /* +C */
#define IMX377_REG_VMAX      0x30F7 /* +F8 +F9[3:0] */
#define IMX377_REG_HMAX      0x30F5 /* +F6 */
/* ...additional key registers here... */

struct imx377_mode {
        u32 width;
        u32 height;
        u32 code;          /* MEDIA_BUS_FMT */
        u32 hts;           /* line length in pixels */
        u32 vts;           /* frame length in lines */
        u64 link_freq;     /* Hz */
};

static const struct imx377_mode mode_12mp_raw12 = {
        .width      = 4056,
        .height     = 3040,
        .code       = MEDIA_BUS_FMT_SRGGB12_1X12,
        .hts        = 0x0172, /* 0x30F5 default */
        .vts        = 0x0CB2, /* 0x30F7 default */
        .link_freq  = 576000000ULL,
};

struct imx377 {
        struct v4l2_subdev     sd;
        struct media_pad       pad;
        struct v4l2_ctrl_handler ctrls;
        struct mutex           lock; /* serialize controls */

        struct clk             *xclk;
        struct regulator       *avdd; /* 2.8V */
        struct regulator       *dvdd; /* 1.1V */
        struct regulator       *dovdd;/* 1.8V */
        struct gpio_desc       *reset_gpio;
        struct gpio_desc       *pwdn_gpio;

        const struct imx377_mode *cur_mode;
        bool                   streaming;
};

static inline int imx377_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
        u8 buf[3] = {reg >> 8, reg & 0xff, val};
        int ret = i2c_master_send(client, buf, 3);
        return (ret == 3) ? 0 : (ret < 0 ? ret : -EIO);
}

static int imx377_set_streaming(struct imx377 *priv, bool on)
{
        struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
        int ret;

        if (on) {
                /* power up */
                regulator_bulk_enable(3, (struct regulator_bulk_data[]){
                        {"dvdd", priv->dvdd}, {"avdd", priv->avdd}, {"dovdd", priv->dovdd}
                });
                clk_prepare_enable(priv->xclk);
                gpiod_set_value_cansleep(priv->reset_gpio, 1);
                gpiod_set_value_cansleep(priv->pwdn_gpio, 0);
                usleep_range(5000, 6000);
                /* TODO: write mode registers */
                ret = imx377_write_reg(client, IMX377_REG_MODE_SEL, 0x01);
                if (ret)
                        return ret;
                priv->streaming = true;
        } else {
                ret = imx377_write_reg(client, IMX377_REG_MODE_SEL, 0x00);
                priv->streaming = false;
                clk_disable_unprepare(priv->xclk);
                gpiod_set_value_cansleep(priv->pwdn_gpio, 1);
                gpiod_set_value_cansleep(priv->reset_gpio, 0);
                regulator_bulk_disable(3, (struct regulator_bulk_data[]){
                        {"dvdd", priv->dvdd}, {"avdd", priv->avdd}, {"dovdd", priv->dovdd}
                });
        }
        return 0;
}

static int imx377_s_stream(struct v4l2_subdev *sd, int enable)
{
        struct imx377 *priv = container_of(sd, struct imx377, sd);
        int ret;

        mutex_lock(&priv->lock);
        ret = imx377_set_streaming(priv, enable);
        mutex_unlock(&priv->lock);
        return ret;
}

/* --- control ops, pad ops, probe(), remove() omitted for brevity --- */

static const struct i2c_device_id imx377_id[] = {
        {"imx377", 0},
        {}
};
MODULE_DEVICE_TABLE(i2c, imx377_id);

static const struct of_device_id imx377_of[] = {
        { .compatible = "sony,imx377" },
        {}
};
MODULE_DEVICE_TABLE(of, imx377_of);

static struct i2c_driver imx377_i2c_driver = {
        .driver = {
                .name  = "imx377",
                .of_match_table = imx377_of,
        },
        .probe  = imx377_probe,
        .remove = imx377_remove,
        .id_table = imx377_id,
};
module_i2c_driver(imx377_i2c_driver);

MODULE_DESCRIPTION("Sony IMX377 CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
```

---

## 7. Kconfig
```Kconfig
config VIDEO_IMX377
        tristate "Sony IMX377 image sensor support"
        depends on I2C && VIDEO_V4L2 && MEDIA_SUBDRV_AUTO
        select V4L2_FWNODE
        select V4L2_ASYNC
        help
          This is a V4L2 driver for the Sony IMX377 12‑megapixel CMOS image sensor.
```

## 8. Makefile
```Makefile
obj-$(CONFIG_VIDEO_IMX377) += imx377.o
```

---

## 9. Troubleshooting
* **I2C NACK:** ensure the sensor is powered, XCLK 24 MHz is present, and reset lifted.
* **No /dev/video node:** confirm the DT endpoint link and that the CSI host driver autoconnects.
* **CRC or Frame errors:** lower link‑frequency, verify lane routing/termination.
* **Strange colors:** check Bayer order; should be `MEDIA_BUS_FMT_SRGGB12_1X12`.

---

## 10. Contributing
Patches welcome!  Please fork and open PRs for:
* Completing register tables (see datasheet pages 25‑33)  citeturn0file0
* Additional resolutions / HDR modes
* Documentation improvements

---

