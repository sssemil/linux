/*
 * Hisilicon Platforms CPUFREQ-DT support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/cpufreq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/cpumask.h>

#define VERSION_ELEMENTS	1
static unsigned int cpufreq_dt_version = 0;

int hisi_cpufreq_set_supported_hw(struct cpufreq_policy *policy)
{
	int ret, cpu;
	struct device *cpu_dev;

	/* find first cpu of policy->cpus */
	cpu = cpumask_any(policy->cpus);
	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("%s Failed to get cpu %d device!\n", __func__, cpu);
		return -ENODEV;
	}

	ret = dev_pm_opp_set_supported_hw(cpu_dev, &cpufreq_dt_version, VERSION_ELEMENTS);
	if (ret)
		pr_err("%s Failed to set supported hardware\n", __func__);

	return ret;
}

void hisi_cpufreq_put_supported_hw(struct cpufreq_policy *policy)
{
	int ret, cpu, j;
	struct device *cpu_dev;

	/* find last cpu of policy->related_cpus */
	for_each_cpu(j, policy->related_cpus) {
		cpu = j;
	}
	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("%s Failed to get cpu %d device!\n", __func__, cpu);
		return;
	}

	dev_pm_opp_put_supported_hw(cpu_dev);
}

static int hisi_cpufreq_get_dt_version(void)
{
	const char *target_cpu;
	int ret, index;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "hisi,targetcpu");
	if (!np) {
		pr_err("%s Failed to find compatible node:targetcpu\n", __func__);
		return -ENODEV;
	}

	ret = of_property_read_string(np, "target_cpu", &target_cpu);
	if (ret) {
		pr_err("%s Failed to read target_cpu\n", __func__);
		of_node_put(np);
		return ret;
	}
	of_node_put(np);

	np = of_find_compatible_node(NULL, NULL, "hisi,supportedtarget");
	if (!np) {
		pr_err("%s Failed to find compatible node:supportedtarget\n", __func__);
		return -ENODEV;
	}

	ret = of_property_match_string(np, "support_name", target_cpu);
	if (ret < 0) {
		pr_err("%s Failed to get support_name\n", __func__);
		of_node_put(np);
		return ret;
	}
	of_node_put(np);

	index = ret;
	cpufreq_dt_version = BIT(index);

	return 0;
}

static int hisi_cpufreq_init(void)
{
	int ret = 0;
	struct platform_device *pdev;

	if (!of_find_compatible_node(NULL, NULL, "arm,generic-bL-cpufreq"))
		return -ENODEV;

	ret = hisi_cpufreq_get_dt_version();
	if (ret)
		return -EINVAL;

	pdev = platform_device_register_simple("cpufreq-dt", -1, NULL, 0);
	if (IS_ERR(pdev))
		return PTR_ERR(pdev);

	return ret;
}
module_init(hisi_cpufreq_init);
