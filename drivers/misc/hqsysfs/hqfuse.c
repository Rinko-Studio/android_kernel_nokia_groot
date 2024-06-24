/*
 * Copyright (C) 2016-2017 Hisense, Inc.
 *
 * Author:
 *   qiuxudong <qiuxudong@hisense.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/utsname.h>
#include <linux/export.h>
#include <linux/debugfs.h>
#include <linux/atomic.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/hqsysfs.h>

#define SECBOOT_BUF_SIZE       32
char fuse_id_product_info[6];
#define EFUSE_STAG2_VALID      0xc10340
#define EFUSE_ANTI_VALID       0xf
struct cpu_fused_status {
	int fuse_reg;
	int fuse_anti;
	int fuse_stag2;
	int fuse_valid_num;
};

static struct cpu_fused_status fuse_status;

static uint cpu_reg_read(u32 add)
{
	void __iomem *base;
	uint r_value;

	base = ioremap(add, 4);
	if (!base) {
		pr_err("%s: Error read cpu register\n", __func__);
		return 0;
	}
	r_value = __raw_readl(base);
	iounmap(base);

	return r_value;
}

static void fuse_set_into_productinfo(void)
{
	uint fuse = 0, anti = 0, stag2 = 0;
	bool fuse_is_stag1 = false,fuse_is_stag2 = false, fuse_is_anti = false;

	u32 valid_num = fuse_status.fuse_valid_num;

	fuse = cpu_reg_read(fuse_status.fuse_reg);
	anti = cpu_reg_read(fuse_status.fuse_anti);
	stag2 = cpu_reg_read(fuse_status.fuse_stag2);
    pr_err("efuse state fuse = %u anti = %u stag2 = %d valid_num = %u/n", fuse, anti, stag2, valid_num);
        fuse_is_stag1 = (valid_num == fuse) ? true : false;
	fuse_is_stag2 = ((stag2 & EFUSE_STAG2_VALID) == EFUSE_STAG2_VALID) ? true : false;
	fuse_is_anti = (anti == EFUSE_ANTI_VALID) ? true : false;

	if (fuse_is_anti && fuse_is_stag2 && fuse_is_stag1)
	    snprintf(fuse_id_product_info,
				sizeof(fuse_id_product_info), "Anti");
	else if (fuse_is_stag2 && fuse_is_stag1)
	    snprintf(fuse_id_product_info,
				sizeof(fuse_id_product_info), "Stag2");
	else if (fuse_is_stag1)
		snprintf(fuse_id_product_info,
				sizeof(fuse_id_product_info), "Stag1");
	else
		snprintf(fuse_id_product_info,
				sizeof(fuse_id_product_info), "False");

	hq_register_hw_info(HWID_EFUSE, fuse_id_product_info);
}

static int cpu_fused_parse_dt(struct device *dev)
{
	int ret = 0;
	struct device_node *pnode;

	pnode = dev->of_node;

	if (ret && ret != -EINVAL) {
		pr_err("%s: read cpuid-reg error\n", __func__);
		goto error;
	}

	ret = of_property_read_u32(pnode, "qcom,fuse-reg",
			&fuse_status.fuse_reg);
	if (ret && ret != -EINVAL) {
		pr_err("%s: read fuse-reg error\n", __func__);
		goto error;
	}

	ret = of_property_read_u32(pnode, "qcom,fuse-anti",
			&fuse_status.fuse_anti);
	if (ret && ret != -EINVAL) {
		pr_err("%s: read fuse-anti error\n", __func__);
		goto error;
	}

	ret = of_property_read_u32(pnode, "qcom,fuse-stag2",
			&fuse_status.fuse_stag2);
	if (ret && ret != -EINVAL) {
		pr_err("%s: read fuse-stag2 error\n", __func__);
		goto error;
	}

	ret = of_property_read_u32(pnode, "qcom,fuse-valid-num",
			&fuse_status.fuse_valid_num);
	if (ret && ret != -EINVAL) {
		pr_err("%s: read fuse-valid-num error\n", __func__);
		goto error;
	}

	return 0;
error:
	return ret;
}

static int cpu_fused_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_err("enter %s\n", __func__);
	if (!pdev->dev.of_node) {
		pr_err("can not find of_node of device\n");
		return -EINVAL;
	}

	ret = cpu_fused_parse_dt(&pdev->dev);
	if (ret < 0)
		return -EINVAL;

	fuse_set_into_productinfo();

	return 0;
}

static int cpu_fused_remove(struct platform_device *pdev)
{
	pr_info("done fuse_status_exit exit\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id cpu_fused_of_match[] = {
	{ .compatible = "qcom,cpu-fused", },
	{},
};
#endif

static struct platform_driver cpu_fused_driver = {
	.probe = cpu_fused_probe,
	.remove = cpu_fused_remove,
	.driver = {
		.name = "fused_status_driver",
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(cpu_fused_of_match),
#endif
	}
};

static int __init fuse_status_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&cpu_fused_driver);
	return ret;
}

static void fuse_status_exit(void)
{
	platform_driver_unregister(&cpu_fused_driver);
}

module_init(fuse_status_init);
module_exit(fuse_status_exit);


