/*
 * Dummy sound driver for wl18xx BT modules
 * Copyright 2016 Andrey Gusakov <andrey.gusakov@cogentembedded.com>
 *
 * Based on: Driver for the DFBM-CS320 bluetooth module
 * Copyright 2011 Lars-Peter Clausen <lars@metafoo.de>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

static struct snd_soc_dai_driver wl18xx_dai = {
	.name = "wl18xx-pcm",
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static struct snd_soc_codec_driver soc_codec_dev_wl18xx;

static int wl18xx_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_wl18xx,
			&wl18xx_dai, 1);
}

static int wl18xx_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id wl18xx_of_match[] = {
	{ .compatible = "ti,wl18xx-pcm", },
	{ }
};
MODULE_DEVICE_TABLE(of, wl18xx_of_match);

static struct platform_driver wl18xx_driver = {
	.driver = {
		.name = "wl18xx-codec",
		.of_match_table = wl18xx_of_match,
		.owner = THIS_MODULE,
	},
	.probe = wl18xx_probe,
	.remove = wl18xx_remove,
};

module_platform_driver(wl18xx_driver);

MODULE_AUTHOR("Andrey Gusakov <andrey.gusakov@cogentembedded.com>");
MODULE_DESCRIPTION("ASoC wl18xx driver");
MODULE_LICENSE("GPL");
