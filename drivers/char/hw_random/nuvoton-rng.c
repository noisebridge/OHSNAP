/*
 * Copyright (c) 2018-2020 Nuvoton Technology Corporation, Y.C. Huang
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <mach/map.h>
#include <mach/regs-crypto.h>


volatile struct nuc980_crypto_regs  *regs;


struct device *rng_dev;

static int  is_first_read = 1;
static int  rng_data_cnt = 0;

static int nuvoton_rng_init(struct hwrng *rng)
{
	/* Dummy function. Clock should be enabled by Nuvoton crypto driver. */
	return 0;
}

static void nuvoton_rng_cleanup(struct hwrng *rng)
{
	/* Dummy function. Nothing need to be cleanup. */
}

static int nuvoton_rng_read(struct hwrng *rng, u32 *data)
{
	volatile int   i;

	if (is_first_read || (rng_data_cnt >= 0x4000000))
	{
		/* If it's the first time called for PRNG or accumulated 4G bits, we have to re-seed
		   the PRNG. Use the current jiffies to be the random seed. */
		is_first_read = 0;
		rng_data_cnt = 0;
		regs->CRPT_PRNG_SEED = jiffies;
		regs->CRPT_PRNG_CTL = PRNG_KEYSZ_64 | SEEDRLD | PRNG_START;   /* generate 64-bits random data with re-seed */
	}
	else
	{
		regs->CRPT_PRNG_CTL = PRNG_KEYSZ_64 | PRNG_START; /* generate the next 64-bits data of random sequence */
	}

	/* Wait for PRNG data available.
	   This loop is much longer than the random number generation time. */
	for (i = 0; i < 10000; i++)
	{
		if (!(regs->CRPT_PRNG_CTL & PRNG_BUSY))
		    break;
	}

	if (regs->CRPT_PRNG_CTL & PRNG_BUSY)
	{
		dev_err(rng_dev, "PRNG h/w busy!\n");
		return 0;       // no data
	}

	//printk("RNG: 0x%x\n", regs->CRPT_PRNG_KEY[0]);

	*data = regs->CRPT_PRNG_KEY[0];    /* retrieve 32-bits data */
	rng_data_cnt ++;

	return 4;
}

static struct hwrng nuvoton_rng = {
	.name		= "nuvoton-rng",
	.init       = nuvoton_rng_init,
	.cleanup    = nuvoton_rng_cleanup,
	.data_read  = nuvoton_rng_read,
};

static int nuvoton_rng_probe(struct platform_device *pdev)
{
	int ret;
	
	regs = (volatile struct nuc980_crypto_regs *)NUC980_VA_CRYPTO;

	ret = hwrng_register(&nuvoton_rng);
	if (ret)
		return ret;

	rng_dev = &pdev->dev;
	dev_info(&pdev->dev, "nuvoton PRNG active\n");

	return 0;
}

static int nuvoton_rng_remove(struct platform_device *pdev)
{
	hwrng_unregister(&nuvoton_rng);
	return 0;
}

static const struct of_device_id nuvoton_rng_of_match[] =
{
    { .compatible = "nuvoton,nuvoton-rng" },
    {},
};
MODULE_DEVICE_TABLE(of, nuvoton_rng_of_match);


static struct platform_driver nuvoton_rng_driver = {
	.probe		= nuvoton_rng_probe,
	.remove		= nuvoton_rng_remove,
	.driver		= {
		.name	= "nuvoton-rng",
		.owner	= THIS_MODULE,
        .of_match_table = of_match_ptr(nuvoton_rng_of_match),
	},
};

module_platform_driver(nuvoton_rng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nuvoton Technology Corporation");
MODULE_DESCRIPTION("Nuvoton NUC980 PRNG driver");
