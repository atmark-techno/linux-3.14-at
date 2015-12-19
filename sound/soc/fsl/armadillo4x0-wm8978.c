/*
 * Copyright (C) 2015 Atmark Techno, Inc. All Rights Reserved.
 *
 * Based on imx_mc13783.c which is :
 *   Copyright 2009 Sascha Hauer, Pengutronix <s.hauer@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <asm/mach-types.h>

#include "../codecs/wm8978.h"
#include "imx-ssi.h"
#include "imx-audmux.h"

/*
 * We hope WM8978 to behave as is the case when using 24.576 MHz, even though
 * the actual frequency is 24.61MHz.
 * If we put 24.61MHz here, WM8978 driver will return error on
 * wm8978_hw_params() when sampling rate is 8KHz because it fails with
 * prerequisite checker. Refer WM8978 code for detail.
 * */
#define ARMADILLO440_WM8978_MCLOCK 24576000

#if defined(CONFIG_ARMADILLO4X0_AUD5_CON11)
#define AUDMUX_INNER_PORT MX25_AUDMUX_PORT1_SSI0
#define AUDMUX_OUTER_PORT MX25_AUDMUX_PORT5
#elif defined(CONFIG_ARMADILLO4X0_AUD6_CON9)
#define AUDMUX_INNER_PORT MX25_AUDMUX_PORT1_SSI0
#define AUDMUX_OUTER_PORT MX25_AUDMUX_PORT6
#else
#error "audmux port is not selected."
#endif

static int armadillo4x0_wm8978_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int rate = params_rate(params);
	int ssi_clk_div2, ssi_clk_psr, ssi_clk_pm;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, WM8978_MCLK,
				     ARMADILLO440_WM8978_MCLOCK,
				     SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* Output MCLK from RXC port */
	ret = snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0,
				     SND_SOC_CLOCK_OUT);
	if (ret) {
		dev_err(cpu_dai->dev,
			"Can't set the IMX_SSP_SYS_CLK CPU system clock.\n");
		return ret;
	}

	/* MCLOCK = 24610000 */
	switch (rate) {
	case 8000:
		ssi_clk_div2 = 0;
		ssi_clk_psr  = 0;
		ssi_clk_pm   = 0x17;
		break;
	case 16000:
		ssi_clk_div2 = 0;
		ssi_clk_psr  = 0;
		ssi_clk_pm   = 0x0b;
		break;
	case 32000:
		ssi_clk_div2 = 0;
		ssi_clk_psr  = 0;
		ssi_clk_pm   = 0x05;
		break;
	case 48000:
		ssi_clk_div2 = 0;
		ssi_clk_psr  = 0;
		ssi_clk_pm   = 0x03;
		break;
	default:
		pr_err("unsupported format\n");
		return -EINVAL;
	}

	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_2, ssi_clk_div2);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_RX_DIV_2, ssi_clk_div2);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PSR, ssi_clk_psr);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_RX_DIV_PSR, ssi_clk_psr);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_TX_DIV_PM, ssi_clk_pm);
	snd_soc_dai_set_clkdiv(cpu_dai, IMX_SSI_RX_DIV_PM, ssi_clk_pm);

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0xffffffc, 0xffffffc, 2, 0);
	if (ret < 0) {
		pr_err("cpu set_tdm_slot failed\n");
		return ret;
	}

	return 0;
}

static struct snd_soc_ops armadillo4x0_wm8978_ops = {
	.hw_params = armadillo4x0_wm8978_hw_params,
};

static struct snd_soc_dai_link armadillo4x0_wm8978_dai[] = {
	{
		.name = "wm8978",
		.stream_name	 = "Sound",
		.codec_dai_name	 = "wm8978-hifi",
		.codec_name	 = "wm8978.2-001a",
		.cpu_dai_name	 = "imx-ssi.0",
		.platform_name	 = "imx-ssi.0",
		.dai_fmt	 = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				   SND_SOC_DAIFMT_CBS_CFS,
		.ops		 = &armadillo4x0_wm8978_ops,
		.symmetric_rates = 1,
	},
};

static struct snd_soc_card armadillo4x0_wm8978 = {
	.name		= "armadillo4x0_wm8978",
	.owner		= THIS_MODULE,
	.dai_link	= armadillo4x0_wm8978_dai,
	.num_links	= ARRAY_SIZE(armadillo4x0_wm8978_dai),
};

static int armadillo4x0_init_audio_clk(void)
{
	struct clk *ssi1_mux_out;
	struct clk *upll_24610k;
	int ret;

	upll_24610k = clk_get(NULL, "upll_24610k");
	if (IS_ERR(upll_24610k)) {
		ret = PTR_ERR(upll_24610k);
		goto fail_upll_24610k;
	}

	ssi1_mux_out = clk_get(NULL, "ssi1_mux_out");
	if (IS_ERR(ssi1_mux_out)) {
		ret = PTR_ERR(ssi1_mux_out);
		goto fail_ssi1_mux_out;
	}

	ret = clk_set_parent(ssi1_mux_out, upll_24610k);
	if (ret < 0)
		goto fail_clk_set_parent;

	return 0;

fail_clk_set_parent:
	clk_put(ssi1_mux_out);

fail_ssi1_mux_out:
	clk_put(upll_24610k);

fail_upll_24610k:
	return ret;
}

static int armadillo4x0_wm8978_probe(struct platform_device *pdev)
{
	int ret;

	ret = armadillo4x0_init_audio_clk();
	if (ret) {
		dev_err(&pdev->dev, "armadillo4x0_init_audio_clk failed (%d)\n",
			ret);
		return ret;
	}

	ret = imx_audmux_v2_configure_port(AUDMUX_INNER_PORT,
		IMX_AUDMUX_V2_PTCR_TFSEL(AUDMUX_OUTER_PORT) |
		IMX_AUDMUX_V2_PTCR_TCSEL(AUDMUX_OUTER_PORT) |
		IMX_AUDMUX_V2_PTCR_RFSEL(AUDMUX_OUTER_PORT) |
		IMX_AUDMUX_V2_PTCR_RCSEL(AUDMUX_OUTER_PORT),
		IMX_AUDMUX_V2_PDCR_RXDSEL(AUDMUX_OUTER_PORT));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}

	ret = imx_audmux_v2_configure_port(AUDMUX_OUTER_PORT,
		IMX_AUDMUX_V2_PTCR_TFSDIR |
		IMX_AUDMUX_V2_PTCR_TFSEL(AUDMUX_INNER_PORT) |
		IMX_AUDMUX_V2_PTCR_TCLKDIR |
		IMX_AUDMUX_V2_PTCR_TCSEL(AUDMUX_INNER_PORT) |
		IMX_AUDMUX_V2_PTCR_RFSDIR |
		IMX_AUDMUX_V2_PTCR_RFSEL(RXFS_SEL | AUDMUX_INNER_PORT) |
		IMX_AUDMUX_V2_PTCR_RCLKDIR |
		IMX_AUDMUX_V2_PTCR_RCSEL(RXC_SEL | AUDMUX_INNER_PORT),
		IMX_AUDMUX_V2_PDCR_RXDSEL(AUDMUX_INNER_PORT));
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

	armadillo4x0_wm8978.dev = &pdev->dev;
	ret = snd_soc_register_card(&armadillo4x0_wm8978);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int armadillo4x0_wm8978_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&armadillo4x0_wm8978);

	return 0;
}

static struct platform_driver armadillo4x0_wm8978_audio_driver = {
	.driver = {
		.name = "armadillo4x0_wm8978",
		.owner = THIS_MODULE,
	},
	.probe = armadillo4x0_wm8978_probe,
	.remove = armadillo4x0_wm8978_remove
};

module_platform_driver(armadillo4x0_wm8978_audio_driver);

MODULE_AUTHOR("Makoto Harada <makoto.harada@atmark-techno.com");
MODULE_DESCRIPTION("Armadilo4x0 with wm8978 codec ALSA SoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:armadillo4x0_wm8978");
