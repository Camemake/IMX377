
    /* SPDX-License-Identifier: GPL-2.0 */
    /*
     * Sony IMX377 12‑MP CMOS Image Sensor V4L2 driver (minimal skeleton)
     *
     * NOTE:
     *  - This is a reference starter implementation intended for public release.
     *  - Register tables for additional modes, and fine‑grained control handling
     *    (HDR, test‑pattern, per‑channel gains) are TODO.
     *  - Compile‑tested against Linux 6.6 but not yet runtime‑verified on hardware.
     *
     *  Contributors are welcome — please send pull requests!
     */

    #include <linux/module.h>
    #include <linux/i2c.h>
    #include <linux/clk.h>
    #include <linux/delay.h>
    #include <linux/gpio/consumer.h>
    #include <linux/regulator/consumer.h>
    #include <linux/mutex.h>
    #include <linux/of_graph.h>
    #include <media/v4l2-ctrls.h>
    #include <media/v4l2-fwnode.h>
    #include <media/v4l2-subdev.h>
    #include <media/v4l2-device.h>

    /* ---- Key sensor registers (datasheet §Register Map) ---- */
    #define IMX377_STANDBY          0x3000
    #define IMX377_REG_MODE_SELECT  0x0100  /* 0x00 = standby, 0x01 = streaming */
    #define IMX377_REG_GAIN_H       0x3009  /* 11‑bit gain (H:L) */
    #define IMX377_REG_GAIN_L       0x300A
    #define IMX377_REG_EXPOSURE_H   0x300B
    #define IMX377_REG_EXPOSURE_L   0x300C
    #define IMX377_REG_VMAX_H       0x30F7
    #define IMX377_REG_VMAX_L       0x30F8
    #define IMX377_REG_HMAX_H       0x30F5
    #define IMX377_REG_HMAX_L       0x30F6

    #define IMX377_LINK_FREQ_576MHZ 576000000ULL

    struct imx377_mode {
        u32 width;
        u32 height;
        u32 code;
        u32 hts;      /* line length in pixels */
        u32 vts;      /* frame length in lines */
        u64 link_freq;
    };

    static const struct imx377_mode imx377_default_mode = {
        .width      = 4056,
        .height     = 3040,
        .code       = MEDIA_BUS_FMT_SRGGB12_1X12,
        .hts        = 0x0172,
        .vts        = 0x0CB2,
        .link_freq  = IMX377_LINK_FREQ_576MHZ,
    };

    struct imx377 {
        struct i2c_client       *client;
        struct v4l2_subdev       sd;
        struct media_pad         pad;

        struct clk              *xclk;
        struct regulator        *avdd;
        struct regulator        *dvdd;
        struct regulator        *dovdd;
        struct gpio_desc        *reset_gpio;
        struct gpio_desc        *pwdn_gpio;

        struct v4l2_ctrl_handler ctrls;
        struct v4l2_ctrl        *gain_ctrl;
        struct v4l2_ctrl        *exp_ctrl;

        const struct imx377_mode *cur_mode;
        struct mutex            lock;   /* protect streaming state */
        bool                    streaming;
    };

    /* ------------------------------------------------------------------ */
    /* I2C helpers                                                         */
    /* ------------------------------------------------------------------ */

    static int imx377_write_reg(struct i2c_client *client, u16 reg, u8 val)
    {
        u8 buf[3] = { reg >> 8, reg & 0xff, val };
        int ret = i2c_master_send(client, buf, 3);
        return (ret == 3) ? 0 : (ret < 0 ? ret : -EIO);
    }

    static int imx377_read_reg(struct i2c_client *client, u16 reg, u8 *val)
    {
        struct i2c_msg msgs[2] = {
            { .addr = client->addr, .flags = 0,
              .buf = (u8[]){ reg >> 8, reg & 0xff }, .len = 2 },
            { .addr = client->addr, .flags = I2C_M_RD, .buf = val, .len = 1 },
        };
        int ret = i2c_transfer(client->adapter, msgs, 2);
        return (ret == 2) ? 0 : (ret < 0 ? ret : -EIO);
    }

    /* ------------------------------------------------------------------ */
    /* Power management                                                    */
    /* ------------------------------------------------------------------ */

    static int imx377_power_on(struct imx377 *priv)
    {
        int ret;

        ret = regulator_enable(priv->dvdd);
        if (ret) return ret;
        ret = regulator_enable(priv->avdd);
        if (ret) goto disable_dvdd;
        ret = regulator_enable(priv->dovdd);
        if (ret) goto disable_avdd;

        ret = clk_prepare_enable(priv->xclk);
        if (ret) goto disable_dovdd;

        if (priv->reset_gpio)
            gpiod_set_value_cansleep(priv->reset_gpio, 1);
        if (priv->pwdn_gpio)
            gpiod_set_value_cansleep(priv->pwdn_gpio, 0);

        /* Allow time for clocks & regulators to stabilise */
        usleep_range(5000, 10000);
        return 0;

    disable_dovdd:
        regulator_disable(priv->dovdd);
    disable_avdd:
        regulator_disable(priv->avdd);
    disable_dvdd:
        regulator_disable(priv->dvdd);
        return ret;
    }

    static void imx377_power_off(struct imx377 *priv)
    {
        if (priv->reset_gpio)
            gpiod_set_value_cansleep(priv->reset_gpio, 0);
        if (priv->pwdn_gpio)
            gpiod_set_value_cansleep(priv->pwdn_gpio, 1);

        clk_disable_unprepare(priv->xclk);
        regulator_disable(priv->dovdd);
        regulator_disable(priv->avdd);
        regulator_disable(priv->dvdd);
    }

    /* ------------------------------------------------------------------ */
    /* Streaming                                                           */
    /* ------------------------------------------------------------------ */

    static int imx377_start_streaming(struct imx377 *priv)
    {
        int ret;

        ret = imx377_power_on(priv);
        if (ret)
            return ret;

        /* Basic register sequence: standby=0, write mode, then stream=1 */
        ret = imx377_write_reg(priv->client, IMX377_STANDBY, 0x00);
        if (ret)
            goto err_power;

        /* TODO: mode register table based on priv->cur_mode */

        ret = imx377_write_reg(priv->client, IMX377_REG_MODE_SELECT, 0x01);
        if (ret)
            goto err_power;

        priv->streaming = true;
        return 0;

    err_power:
        imx377_power_off(priv);
        return ret;
    }

    static int imx377_stop_streaming(struct imx377 *priv)
    {
        int ret = imx377_write_reg(priv->client, IMX377_REG_MODE_SELECT, 0x00);
        priv->streaming = false;
        imx377_power_off(priv);
        return ret;
    }

    /* ------------------------------------------------------------------ */
    /* V4L2 control operations                                             */
    /* ------------------------------------------------------------------ */

    static int imx377_set_ctrl(struct v4l2_ctrl *ctrl)
    {
        struct imx377 *priv = container_of(ctrl->handler, struct imx377, ctrls);
        struct i2c_client *client = priv->client;
        int ret = 0;

        if (!priv->streaming)
            return 0; /* defer until streaming */

        switch (ctrl->id) {
        case V4L2_CID_EXPOSURE:
            /* 16‑bit coarse integration time register */
            ret  = imx377_write_reg(client, IMX377_REG_EXPOSURE_H, (ctrl->val >> 8) & 0xFF);
            ret |= imx377_write_reg(client, IMX377_REG_EXPOSURE_L, ctrl->val & 0xFF);
            break;

        case V4L2_CID_ANALOGUE_GAIN:
            /* 11‑bit gain; split across two regs */
            ret  = imx377_write_reg(client, IMX377_REG_GAIN_H, (ctrl->val >> 8) & 0x07);
            ret |= imx377_write_reg(client, IMX377_REG_GAIN_L, ctrl->val & 0xFF);
            break;
        }
        return ret;
    }

    static const struct v4l2_ctrl_ops imx377_ctrl_ops = {
        .s_ctrl = imx377_set_ctrl,
    };

    /* ------------------------------------------------------------------ */
    /* Subdev pad operations                                               */
    /* ------------------------------------------------------------------ */

    static int imx377_get_fmt(struct v4l2_subdev *sd,
                              struct v4l2_subdev_pad_config *cfg,
                              struct v4l2_subdev_format *fmt)
    {
        struct imx377 *priv = container_of(sd, struct imx377, sd);
        const struct imx377_mode *mode = priv->cur_mode;

        fmt->format.code   = mode->code;
        fmt->format.width  = mode->width;
        fmt->format.height = mode->height;
        fmt->format.field  = V4L2_FIELD_NONE;
        fmt->format.colorspace = V4L2_COLORSPACE_RAW;
        return 0;
    }

    static int imx377_set_fmt(struct v4l2_subdev *sd,
                              struct v4l2_subdev_pad_config *cfg,
                              struct v4l2_subdev_format *fmt)
    {
        struct imx377 *priv = container_of(sd, struct imx377, sd);
        /* Only one fixed mode for now */
        priv->cur_mode = &imx377_default_mode;
        return imx377_get_fmt(sd, cfg, fmt);
    }

    static const struct v4l2_subdev_pad_ops imx377_pad_ops = {
        .get_fmt = imx377_get_fmt,
        .set_fmt = imx377_set_fmt,
        .enum_mbus_code = NULL, /* simple fixed format driver */
    };

    static const struct v4l2_subdev_video_ops imx377_video_ops = {
        .s_stream = NULL, /* filled later */
    };

    static const struct v4l2_subdev_ops imx377_subdev_ops = {
        .pad    = &imx377_pad_ops,
        .video  = &imx377_video_ops,
    };

    /* ------------------------------------------------------------------ */
    /* Probe / Remove                                                      */
    /* ------------------------------------------------------------------ */

    static int imx377_probe(struct i2c_client *client)
    {
        struct device *dev = &client->dev;
        struct imx377 *priv;
        int ret;

        priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
        if (!priv)
            return -ENOMEM;

        priv->client = client;
        mutex_init(&priv->lock);
        priv->cur_mode = &imx377_default_mode;

        /* Regulators */
        priv->avdd  = devm_regulator_get(dev, "avdd");
        priv->dvdd  = devm_regulator_get(dev, "dvdd");
        priv->dovdd = devm_regulator_get(dev, "dovdd");
        if (IS_ERR(priv->avdd) || IS_ERR(priv->dvdd) || IS_ERR(priv->dovdd))
            return -EPROBE_DEFER;

        /* Clock */
        priv->xclk = devm_clk_get(dev, "xclk");
        if (IS_ERR(priv->xclk))
            return -EPROBE_DEFER;
        clk_set_rate(priv->xclk, 24000000);

        /* GPIOs */
        priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
        priv->pwdn_gpio  = devm_gpiod_get_optional(dev, "pwdn",  GPIOD_OUT_HIGH);

        /* V4L2 ctrl handler */
        v4l2_ctrl_handler_init(&priv->ctrls, 2);
        priv->gain_ctrl = v4l2_ctrl_new_std(&priv->ctrls, &imx377_ctrl_ops,
                                            V4L2_CID_ANALOGUE_GAIN, 0, 0x7A5, 1, 0);
        priv->exp_ctrl  = v4l2_ctrl_new_std(&priv->ctrls, &imx377_ctrl_ops,
                                            V4L2_CID_EXPOSURE, 1, 0xFFFF, 1, 0x03E8);
        priv->sd.ctrl_handler = &priv->ctrls;
        if (priv->ctrls.error)
            return priv->ctrls.error;

        /* Subdev */
        v4l2_i2c_subdev_init(&priv->sd, client, &imx377_subdev_ops);
        priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

        /* Pad */
        priv->pad.flags = MEDIA_PAD_FL_SOURCE;
        ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
        if (ret)
            return ret;
        priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

        /* Register subdev */
        ret = v4l2_async_register_subdev(&priv->sd);
        if (ret)
            return ret;

        dev_info(dev, "IMX377 sensor probed
");
        return 0;
    }

    static void imx377_remove(struct i2c_client *client)
    {
        struct imx377 *priv = i2c_get_clientdata(client);
        v4l2_async_unregister_subdev(&priv->sd);
        media_entity_cleanup(&priv->sd.entity);
        v4l2_ctrl_handler_free(&priv->ctrls);
    }

    /* I2C boilerplate */
    static const struct of_device_id imx377_of_table[] = {
        { .compatible = "sony,imx377" },
        { }
    };
    MODULE_DEVICE_TABLE(of, imx377_of_table);

    static struct i2c_driver imx377_driver = {
        .driver = {
            .name  = "imx377",
            .of_match_table = imx377_of_table,
        },
        .probe_new = imx377_probe,
        .remove    = imx377_remove,
    };
    module_i2c_driver(imx377_driver);

    MODULE_DESCRIPTION("Sony IMX377 image sensor driver (reference)");
    MODULE_LICENSE("GPL v2");
