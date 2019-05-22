#include <soc_sctrl_interface.h>
#include <soc_crgperiph_interface.h>
#include <soc_pctrl_interface.h>
#include <soc_usb31_misc_ctrl_interface.h>
#include <soc_usb31_tca_interface.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/hisi/usb/hisi_usb.h>
#include <linux/hisi/usb/dwc3-kirin970.h>
#include <linux/hisi/contexthub/tca.h>
#include "inputhub_route.h"
#include "common.h"
#ifdef COMBOPHY_ES_BUGFIX
#include "firmware.h"
#endif
extern int hisi_dptx_triger(bool benable);
extern int hisi_dptx_hpd_trigger(TCA_IRQ_TYPE_E irq_type, TCPC_MUX_CTRL_TYPE mode);
extern int hisi_dptx_notify_switch(void);
extern int hisi_usb_otg_event_sync(enum otg_dev_event_type event);
#ifdef COMBOPHY_ES_BUGFIX
extern int usb31phy_cr_write(u32 addr, u16 value);
#endif
#define HISI_TCA_DEBUG              KERN_ERR
#define PD_PLATFORM_INIT_OK (0X56781234)
#define FIFO_SIZE (128)
/*lint -749*/
typedef struct pd_event_member_s {
	TCA_IRQ_TYPE_E irq_type;
	TCPC_MUX_CTRL_TYPE mode_type;
	TCA_DEV_TYPE_E dev_type;
	TYPEC_PLUG_ORIEN_E typec_orien;
}pd_event_t;

typedef enum {
	TCA_POWEROFF = 0,
	TCA_POWERON = 1,
	TCA_POWER_MAX
}TCA_POWER_TYPE;

struct tca_device_s {
	int init;
	TCA_POWER_TYPE tca_poweron;
	TCPC_MUX_CTRL_TYPE tca_cur_mode;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct kfifo_rec_ptr_1 kfifo;
	struct clk *gt_hclk_usb3otg;
	struct clk *gt_clk_usb3_tcxo_en;
	void __iomem *crgperi_reg_base;
	void __iomem *sctrl_reg_base;
	void __iomem *pctrl_reg_base;
	void __iomem *usb_misc_base;
	void __iomem *tca_base;
};

static DEFINE_MUTEX(tca_mutex);
static struct tca_device_s tca_dev;

__weak void dp_dfp_u_notify_dp_configuration_done(TCPC_MUX_CTRL_TYPE mode_type, int ack)
{
	printk(HISI_TCA_DEBUG"[%s] curmode[%d]ack[%d]\n", __func__, mode_type, ack);
}

static int pd_get_resource(struct tca_device_s *res, struct device *dev)
{
	struct device_node *np;
	/*
	 * map PERI CRG region
	 */
	np = of_find_compatible_node(NULL, NULL, "hisilicon,crgctrl");
	if (!np) {
		pr_err("[%s]get peri cfg node failed!\n", __func__);
		return -EINVAL;
	}
	res->crgperi_reg_base = of_iomap(np, 0);
	if (!res->crgperi_reg_base) {
		pr_err("[%s]iomap crgperi_reg_base failed!\n", __func__);
		return -EINVAL;
	}

	/*
	 * map SCTRL region
	 */
	np = of_find_compatible_node(NULL, NULL, "hisilicon,sysctrl");
	if (!np) {
		pr_err("[%s]get sysctrl node failed!\n", __func__);
		goto CRGCTRL_MAP_REL;
	}
	res->sctrl_reg_base = of_iomap(np, 0);
	if (!res->sctrl_reg_base) {
		pr_err("[%s]iomap sctrl_reg_base failed!\n", __func__);
		goto CRGCTRL_MAP_REL;
	}

	/*
	 * map PCTRL region
	 */
	np = of_find_compatible_node(NULL, NULL, "hisilicon,pctrl");
	if (!np) {
		pr_err("[%s]get pctrl node failed!\n",__func__);
		goto SCTRL_MAP_REL;
	}
	res->pctrl_reg_base = of_iomap(np, 0);
	if (!res->pctrl_reg_base) {
		pr_err("[%s]iomap pctrl_reg_base failed!\n",__func__);
		goto SCTRL_MAP_REL;
	}

	/*
	 * map USB2OTG bc region
	 */
	np = of_find_compatible_node(NULL, NULL, "hisilicon,usb3otg_bc");
	if (!np) {
		pr_err("[%s]get usb3otg_bc failed!\n",__func__);
		goto PCTRL_MAP_REL;
	}
	res->usb_misc_base = of_iomap(np, 0);
	if (!res->usb_misc_base) {
		pr_err("[%s]iomap usb3otg_bc failed!\n",__func__);
		goto PCTRL_MAP_REL;
	}

	res->tca_base = res->usb_misc_base + 0x200;

	res->gt_hclk_usb3otg = devm_clk_get(dev, "hclk_usb3otg");
	if (IS_ERR_OR_NULL(res->gt_hclk_usb3otg)) {
		pr_err("[%s]devm_clk_get  hclk_usb3otg failed!\n",__func__);
		goto USB_MISC_REL;
	}

	res->gt_clk_usb3_tcxo_en = devm_clk_get(dev, "clk_usb3_tcxo_en");
	if (IS_ERR_OR_NULL(res->gt_clk_usb3_tcxo_en)) {
		pr_err("[%s]dgt_clk_usb3_tcxo_enfailed!\n",__func__);
		goto USB_MISC_REL;
	}

	return 0;

USB_MISC_REL:
	iounmap(res->usb_misc_base);
PCTRL_MAP_REL:
	iounmap(res->pctrl_reg_base);
SCTRL_MAP_REL:
	iounmap(res->sctrl_reg_base);
CRGCTRL_MAP_REL:
	iounmap(res->crgperi_reg_base);
	return -1;
}

static TCPC_MUX_CTRL_TYPE _tca_mode_switch(TCPC_MUX_CTRL_TYPE old_mode,
	TCPC_MUX_CTRL_TYPE new_mode, TYPEC_PLUG_ORIEN_E typec_orien)
{
	int ret = 0, cnt;
	struct timeval tv;
	volatile unsigned int reg_data = 0x10;
	writel(0xFFFF, SOC_USB31_TCA_TCA_INTR_STS_ADDR(tca_dev.tca_base));
	udelay(1);
	/*set_bits(0x3, SOC_USB31_TCA_TCA_INTR_EN_ADDR(tca_dev.tca_base));  for irq mode,but we use poll waitting */
	reg_data |= new_mode;
	reg_data |= (0x01&typec_orien)<<SOC_USB31_TCA_TCA_TCPC_tcpc_connector_orientation_START;
	writel_mask(0x1F,reg_data, SOC_USB31_TCA_TCA_TCPC_ADDR(tca_dev.tca_base));
	udelay(1);
	pr_info("[%s]old[%d]new[%d]TCPC[0x%x][0x%x]\n", __func__,
		old_mode, new_mode, reg_data, readl(SOC_USB31_TCA_TCA_TCPC_ADDR(tca_dev.tca_base)));

	cnt = 2000;
	do_gettimeofday(&tv);
	pr_info("s:tv_sec %ld,tv_usec: %06ld\n", tv.tv_sec, tv.tv_usec);
CTRLSYNCMODE_DBG0:
	while(is_bits_clr(BIT(SOC_USB31_TCA_TCA_INTR_STS_xa_ack_evt_START),
		SOC_USB31_TCA_TCA_INTR_STS_ADDR(tca_dev.tca_base))) {
			cnt--;
			if(is_bits_set(BIT(SOC_USB31_TCA_TCA_INTR_STS_xa_timeout_evt_START),
				SOC_USB31_TCA_TCA_INTR_STS_ADDR(tca_dev.tca_base))) {
				unsigned int a,b;
				reg_data= readl(SOC_USB31_TCA_TCA_CTRLSYNCMODE_DBG0_ADDR(tca_dev.tca_base));
				a = 0x1&(reg_data>>SOC_USB31_TCA_TCA_CTRLSYNCMODE_DBG0_ss_rxdetect_disable_START);
				b = 0x1&(reg_data>>SOC_USB31_TCA_TCA_CTRLSYNCMODE_DBG0_ss_rxdetect_disable_ack_START);
				if(cnt >0) {
					if(a != b) {
						goto CTRLSYNCMODE_DBG0;
					}else{
						// cppcheck-suppress *
						_tca_mode_switch(old_mode, new_mode, typec_orien);
					}
					msleep(2);
				}else {
					pr_err("[%s]CTRLSYNCMODE_DBG0 TIMEOUT\n",__func__);
					ret = -EMLINK;
					goto PD_FIN;
				}
			}else if(cnt>0){
				msleep(2);
			}else {
				pr_err("[%s]soc timeout not set;soft timeout\n",__func__);
				ret = -ERANGE;
				goto PD_FIN;
			}
	}

	tca_dev.tca_cur_mode = new_mode;
PD_FIN:
	do_gettimeofday(&tv);
	pr_info("e:tv_sec %ld,tv_usec: %06ld\n", tv.tv_sec, tv.tv_usec);
	pr_info("0x28:[%x]0x2c:[%x]0x30:[%x]0x34:[%x]0x08:[%x]\n",
		readl(0x28+tca_dev.tca_base), readl(0x2c+tca_dev.tca_base), readl(0x30+tca_dev.tca_base),
		readl(0x34+tca_dev.tca_base),readl(0x8+tca_dev.tca_base));
	return ret;
}
extern int kirin970_usb31_controller_dump(void); /* debug */

void tca_dump(void)
{
	int i;
	pr_err("[%s]+\n", __func__);
	for(i=0;i <= 0x40; i+=4) {
		pr_err("[%x]:[%x]\n", i, readl(i+tca_dev.tca_base));
	}
	pr_err("[%s]-\n", __func__);
}

void usb_misc_dump(void)
{
	int i;
	pr_err("[%s]++++\n", __func__);
	for(i=0;i <= 0x250; i+=4) {
		pr_err("[%x]:[%x]\n", i,readl(i+tca_dev.usb_misc_base));
	}
	pr_err("[%s]-----\n", __func__);
}

extern u16 usb31phy_cr_read(u32 addr);
void cr_dump(void)
{
	int i;
	pr_err("[%s]++++\n", __func__);
	for(i=0;i <= 0x006f; i++) {
		pr_err("[%x]:[%x]\n", i, usb31phy_cr_read(i));
	}

	for(i=0x1000;i <= 0x010D8; i++) {
		pr_err("[%x]:[%x]\n", i, usb31phy_cr_read(i));
	}

	for(i=0x1100;i <= 0x011D8; i++) {
		pr_err("[%x]:[%x]\n", i, usb31phy_cr_read(i));
	}

	for(i=0x1200;i <= 0x12D8; i++) {
		pr_err("[%x]:[%x]\n", i, usb31phy_cr_read(i));
	}

	for(i=0x1300;i <= 0x13D8; i++) {
		pr_err("[%x]:[%x]\n", i, usb31phy_cr_read(i));
	}

	for(i=0x2000;i <= 0x203b; i++) {
		pr_err("[%x]:[%x]\n", i, usb31phy_cr_read(i));
	}

	for(i=0x3000;i <= 0x30e4; i++) {
		pr_err("[%x]:[%x]\n", i, usb31phy_cr_read(i));
	}

	for(i=0x6000;i < 0x6000+2737; i++) {
		pr_err("[%x]:[%x]\n", i, usb31phy_cr_read(i));
	}

	pr_err("[%s]-----\n", __func__);
}

int tca_mode_switch(TCPC_MUX_CTRL_TYPE new_mode, TYPEC_PLUG_ORIEN_E typec_orien)
{
	int old_mode = tca_dev.tca_cur_mode;
	int ret = 0;
	int cnt = 1000;

	if(TCPC_DP == old_mode) {
		usb31phy_cr_write(0x05, 0x8199);
		usb31phy_cr_write(0x05, 0x8199);
		udelay(100);
	}

	ret = kirin970_usb31_phy_notify(PHY_MODE_CHANGE_BEGIN);
	if (ret) {
		pr_err("[%s]kirin970_usb31_phy_notify  err\n", __func__);
		goto TCA_MODE_SW_FIN;
	}

	while(cnt--) {
		if(TCPC_NC == new_mode) {
			if(0x333333 == readl(SOC_USB31_TCA_TCA_PSTATE_ADDR(tca_dev.tca_base)))
				break;
		}else
			break;
		msleep(1);
	}

	printk(HISI_TCA_DEBUG"[%s]TCA_PSTATE[%x]cnt[%d]\n", __func__,
		readl(SOC_USB31_TCA_TCA_PSTATE_ADDR(tca_dev.tca_base)),cnt);

	if(old_mode == TCPC_NC && TCPC_DP == new_mode) {
		ret = _tca_mode_switch(TCPC_NC, TCPC_USB31_CONNECTED,typec_orien);
		if (ret) {
			pr_err("[%s]_tca_mode_switch  err1 [%d]\n", __func__, __LINE__);
			goto TCA_MODE_SW_FIN;
		}
		set_bits(BIT(SOC_USB31_TCA_TCA_CTRLSYNCMODE_CFG0_block_ss_op_START),
			SOC_USB31_TCA_TCA_CTRLSYNCMODE_CFG0_ADDR(tca_dev.tca_base));
		msleep(1);
	}

	ret = _tca_mode_switch(tca_dev.tca_cur_mode, new_mode, typec_orien);
	if(ret) {
		pr_err("[%s]_tca_mode_switch  err2 [%d]\n", __func__, __LINE__);
		goto TCA_MODE_SW_FIN;
	}

	if(old_mode == TCPC_NC && TCPC_DP == new_mode) {
		clr_bits(BIT(SOC_USB31_TCA_TCA_CTRLSYNCMODE_CFG0_block_ss_op_START),
			SOC_USB31_TCA_TCA_CTRLSYNCMODE_CFG0_ADDR(tca_dev.tca_base));
	}

	if(TCPC_DP == old_mode) {
		usb31phy_cr_write(0x05, 0x198);
		udelay(100);
	}

	ret = kirin970_usb31_phy_notify(PHY_MODE_CHANGE_END);
	if (ret) {
		pr_err("[%s]kirin970_usb31_phy_notify END  err\n", __func__);
		goto TCA_MODE_SW_FIN;
	}

TCA_MODE_SW_FIN:
	return ret;
}

static void toggle_clock(void)
{
	int i;
	for(i=0;i<32;i++) {
		set_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG54_usb3_phy0_cr_para_clk_START),
			SOC_USB31_MISC_CTRL_USB_MISC_CFG54_ADDR(tca_dev.usb_misc_base));
		clr_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG54_usb3_phy0_cr_para_clk_START),
			SOC_USB31_MISC_CTRL_USB_MISC_CFG54_ADDR(tca_dev.usb_misc_base));
	}
}

static void combophy_firmware_update(void)
{
#ifdef COMBOPHY_ES_BUGFIX
	int i,cnt;
	int fw_size = sizeof(firmware)/sizeof(firmware[0]);
	printk(HISI_TCA_DEBUG"[%s]fw_size[%d]\n", __func__, fw_size);
//选择CR 接口： MISC54[4] =  1
	set_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG54_usb3_phy0_cr_para_sel_START),
		SOC_USB31_MISC_CTRL_USB_MISC_CFG54_ADDR(tca_dev.usb_misc_base));
//toggle clock * 32次： MISC54[2] =  1； MISC54[2] =  0；循环32次
	toggle_clock();
/*
3、等待PHY准备好
 wait for sram_init_done：MISC5c[12]  ==1
*/
	cnt = 20;
	while(cnt--) {
		if(is_bits_set(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG5C_phy0_sram_init_done_START),
				SOC_USB31_MISC_CTRL_USB_MISC_CFG5C_ADDR(tca_dev.usb_misc_base)))
				break;
		msleep(1);
	}
/*
4、更新firmware:
将获得的firmware，从地址0x6000开始， 依次写入（调用CR写函数）
*/
	i = 0;
	usb31phy_cr_write(0x6000+i,firmware[i]);
	for(i = 0; i< fw_size; i++) {
		usb31phy_cr_write(0x6000+i,firmware[i]);
	}

//toggle clock * 32次： MISC54[2] =  1； MISC54[2] =  0；循环32次
	toggle_clock();
//5、通知PHY读取数据
//sram_ext_ld_done =1: MISC5c[3]  =1
	set_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG54_usb3_phy0_cr_para_rd_en_START),
		SOC_USB31_MISC_CTRL_USB_MISC_CFG5C_ADDR(tca_dev.usb_misc_base));
//toggle clock * 32次： MISC54[2] =  1； MISC54[2] =  0；循环32次
	toggle_clock();
//6、延迟1mS，等PHY OK。
	msleep(1);
#endif
}

/*lint  -e838 -e747*/
int combophy_poweron(void)
{
	int cnt = 50;
	int ret = 0;
	int i = 0;

	printk(HISI_TCA_DEBUG"[%s]tca_poweron[%d]\n", __func__, tca_dev.tca_poweron);
	if (TCA_POWERON == tca_dev.tca_poweron)
		return 0;

/*	writel(BIT(SOC_CRGPERIPH_PEREN0_gt_hclk_usb3otg_misc_START),
		SOC_CRGPERIPH_PEREN0_ADDR(tca_dev.crgperi_reg_base));*/
	ret = clk_prepare_enable(tca_dev.gt_hclk_usb3otg);
	if (ret) {
		pr_err("[%s]clk_prepare_enable  gt_hclk_usb3otg err\n", __func__);
		return -EIO;
	}

	writel(BIT(SOC_CRGPERIPH_PERRSTEN4_ip_rst_usb3otg_32k_START)|
		BIT(SOC_CRGPERIPH_PERRSTEN4_ip_hrst_usb3otg_misc_START),
		SOC_CRGPERIPH_PERRSTEN4_ADDR(tca_dev.crgperi_reg_base));
	udelay(1);
	writel(BIT(SOC_CRGPERIPH_PERRSTDIS4_ip_rst_usb3otg_32k_START)|
		BIT(SOC_CRGPERIPH_PERRSTDIS4_ip_hrst_usb3otg_misc_START),
		SOC_CRGPERIPH_PERRSTDIS4_ADDR(tca_dev.crgperi_reg_base));

	printk(HISI_TCA_DEBUG"PERCLKEN0[%x]PERSTAT0[%x]PERRSTSTAT4[%x]\n",
		readl(SOC_CRGPERIPH_PERCLKEN0_ADDR(tca_dev.crgperi_reg_base)),
		readl(SOC_CRGPERIPH_PERSTAT0_ADDR(tca_dev.crgperi_reg_base)),
		readl(SOC_CRGPERIPH_PERRSTSTAT4_ADDR(tca_dev.crgperi_reg_base)));

	writel((2* 0x0927C), SOC_USB31_TCA_TCA_CTRLSYNCMODE_CFG1_ADDR(tca_dev.tca_base));
/*4	open combo phy */
	writel(BIT(SOC_CRGPERIPH_ISODIS_usb_refclk_iso_en_START),
		SOC_CRGPERIPH_ISODIS_ADDR(tca_dev.crgperi_reg_base));

/*	writel(HM_EN(SOC_PCTRL_PERI_CTRL3_usb_tcxo_en_START),
			SOC_PCTRL_PERI_CTRL3_ADDR(tca_dev.pctrl_reg_base));*/
	ret = clk_prepare_enable(tca_dev.gt_clk_usb3_tcxo_en);
	if (ret) {
		pr_err("[%s]clk_prepare_enable  clk_usb3_tcxo_en err\n", __func__);
		return -EIO;
	}

	msleep(1);
	clr_bits(BIT(SOC_PCTRL_PERI_CTRL24_sc_clk_usb3phy_3mux1_sel_START),
			SOC_PCTRL_PERI_CTRL24_ADDR(tca_dev.pctrl_reg_base));
/*5	dp-->p3 mode*/
	ret = hisi_dptx_triger(1);
	if (ret) {
		pr_err("[%s]hisi_dptx_triger  err\n", __func__);
		return -EFAULT;
	}


/*5.5 release USB31 PHY out of TestPowerDown mode*/
	clr_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG50_usb3_phy_test_powerdown_START),
	SOC_USB31_MISC_CTRL_USB_MISC_CFG50_ADDR(tca_dev.usb_misc_base));
	udelay(50);
	set_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG54_usb3_phy0_ana_pwr_en_START)|
		BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG54_phy0_pcs_pwr_stable_START)|
		BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG54_phy0_pma_pwr_stable_START),
	SOC_USB31_MISC_CTRL_USB_MISC_CFG54_ADDR(tca_dev.usb_misc_base));

#ifdef COMBOPHY_ES_BUGFIX
//e.	配置sram_bypass = 0:  MISC_CTRL 5c[2] = 0
	clr_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFG5C_usb3_phy0_sram_bypass_START),
	SOC_USB31_MISC_CTRL_USB_MISC_CFG5C_ADDR(tca_dev.usb_misc_base));
#endif

/*6	unreset combo phy*/
	set_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFGA0_usb3phy_reset_n_START),
	SOC_USB31_MISC_CTRL_USB_MISC_CFGA0_ADDR(tca_dev.usb_misc_base));

	combophy_firmware_update();

	tca_mode_switch(TCPC_NC, TYPEC_ORIEN_POSITIVE);
	writel(BIT(SOC_USB31_TCA_TCA_INTR_STS_xa_ack_evt_START)|
		BIT(SOC_USB31_TCA_TCA_INTR_STS_xa_timeout_evt_START),
		SOC_USB31_TCA_TCA_INTR_STS_ADDR(tca_dev.tca_base));
	tca_dev.tca_poweron = TCA_POWERON;
	return ret;
}

/*lint  +e838*/
/*lint -e124 */
void combophy_poweroff(TCPC_MUX_CTRL_TYPE cur_mode, TCA_DEV_TYPE_E dev_type)
{
	int ret, flag;
	volatile unsigned int reg;

	printk(HISI_TCA_DEBUG"[%s]who want off[%d]power[%d]\n", __func__, cur_mode, tca_dev.tca_poweron);

	if(TCA_POWEROFF == tca_dev.tca_poweron)
		return;

	if (TCPC_NC == cur_mode)
		flag = 1;
	else if ((TCPC_USB31_CONNECTED == cur_mode)&&(TCPC_USB31_CONNECTED == tca_dev.tca_cur_mode))
		flag = 1;
	else
		flag = 0;

	if(0 == flag)
		return;

	reg = readl(SOC_CRGPERIPH_PERRSTSTAT4_ADDR(tca_dev.crgperi_reg_base));
	reg &= BIT(SOC_CRGPERIPH_PERRSTSTAT4_ip_rst_usb3otg_32k_START)|
			BIT(SOC_CRGPERIPH_PERRSTSTAT4_ip_hrst_usb3otg_misc_START);
	if(reg)
		goto USB_MISC_CTRL_FIN;


	if(is_bits_clr(BIT(SOC_CRGPERIPH_PERSTAT0_st_hclk_usb3otg_misc_START),
		SOC_CRGPERIPH_PERSTAT0_ADDR(tca_dev.crgperi_reg_base))){
		goto USB_MISC_CTRL_FIN;
	}

	reg = readl(SOC_CRGPERIPH_PERSTAT4_ADDR(tca_dev.crgperi_reg_base));
	reg &= BIT(SOC_CRGPERIPH_PERSTAT4_st_clk_usb3otg_ref_START)|
			BIT(SOC_CRGPERIPH_PERSTAT4_st_aclk_usb3otg_START);
	if(0x03 != reg)
		goto USB_MISC_CTRL_FIN;

	if (dev_type <= ID_RISE_EVENT) {
		ret = hisi_usb_otg_event_sync((enum otg_dev_event_type)dev_type);
		if (ret) {
			pr_err("hisi_usb_otg_event_sync  err\n");
			return;
		}
	}

	clr_bits(BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFGA0_usb2phy_por_n_START)|
		BIT(SOC_USB31_MISC_CTRL_USB_MISC_CFGA0_usb3phy_reset_n_START),
	SOC_USB31_MISC_CTRL_USB_MISC_CFGA0_ADDR(tca_dev.usb_misc_base));

	if(is_bits_clr((unsigned int)BIT(20), SOC_SCTRL_SCDEEPSLEEPED_ADDR(tca_dev.sctrl_reg_base))) {
		/*writel(HM_DIS(SOC_PCTRL_PERI_CTRL3_usb_tcxo_en_START),
			SOC_PCTRL_PERI_CTRL3_ADDR(tca_dev.pctrl_reg_base));*/
			clk_disable_unprepare(tca_dev.gt_clk_usb3_tcxo_en);

	}else {
		/*writel(BIT(SOC_CRGPERIPH_PERDIS6_gt_clk_usb2phy_ref_START),
			SOC_CRGPERIPH_PERDIS6_ADDR(tca_dev.crgperi_reg_base));
		clk_gate_usb2phy_ref: clk_usb2phy_ref {
		compatible = "hisilicon,hi3xxx-clk-gate";
		#clock-cells = <0>;
		clocks = <&clkin_sys>;
		hisilicon,hi3xxx-clkgate = <0x410 0x80000>;
		clock-output-names = "clk_usb2phy_ref";
		*/
	}

USB_MISC_CTRL_FIN:
	writel(BIT(SOC_CRGPERIPH_PERRSTEN4_ip_rst_usb3otg_32k_START)|
		BIT(SOC_CRGPERIPH_PERRSTEN4_ip_hrst_usb3otg_misc_START),
		SOC_CRGPERIPH_PERRSTEN4_ADDR(tca_dev.crgperi_reg_base));
	/*gt_clk_usb3otg_ref、Gt_aclk_usb3otg  这两个时钟由USB模块负责 前进2017/3/17 */
	/*writel(BIT(SOC_CRGPERIPH_PERDIS0_gt_hclk_usb3otg_misc_START),
		SOC_CRGPERIPH_PERDIS0_ADDR(tca_dev.crgperi_reg_base));*/
	clk_disable_unprepare(tca_dev.gt_hclk_usb3otg);
	tca_dev.tca_poweron = TCA_POWEROFF;
}
/*lint +e124 +e747*/
int pd_event_notify(TCA_IRQ_TYPE_E irq_type, TCPC_MUX_CTRL_TYPE mode_type, TCA_DEV_TYPE_E dev_type, TYPEC_PLUG_ORIEN_E typec_orien)
{
	pd_event_t pd_event;
	printk(HISI_TCA_DEBUG"[%s]IRQ[%d]MODEcur[%d]new[%d]DEV[%d]ORIEN[%d]\n",
		__func__, irq_type, tca_dev.tca_cur_mode,mode_type, dev_type, typec_orien);
	if(PD_PLATFORM_INIT_OK != tca_dev.init) {
		pr_err("[%s]probe not init fin pls wait\n", __func__);
		return -EIO;
	}

	if (irq_type >=TCA_IRQ_MAX_NUM || mode_type >= TCPC_MUX_MODE_MAX || dev_type >= TCA_DEV_MAX || typec_orien >= TYPEC_ORIEN_MAX)
		return -EPERM;

	pd_event.irq_type = irq_type;
	pd_event.mode_type = mode_type;
	pd_event.dev_type = dev_type;
	pd_event.typec_orien = typec_orien;
	kfifo_in(&tca_dev.kfifo, &pd_event, (unsigned int)sizeof(pd_event_t));
	queue_work(tca_dev.wq, &tca_dev.work);
	return 0;
}

/*lint  -e655*/
void pd_event_hander(pd_event_t *event)
{
	int ret =0;
	printk(HISI_TCA_DEBUG"[%s]IRQ[%d]MODEcur[%d]new[%d]DEV[%d]ORIEN[%d]\n",
		__func__, event->irq_type, tca_dev.tca_cur_mode,event->mode_type, event->dev_type, event->typec_orien);
	if(TCPC_NC == event->mode_type) {
		if (tca_dev.tca_cur_mode&TCPC_DP) {
			hisi_dptx_hpd_trigger(event->irq_type, tca_dev.tca_cur_mode);
			ret = hisi_dptx_triger(0);
			if (ret) {
				pr_err("[%s]hisi_dptx_triger err[%d]\n",__func__, ret);
				ret = -EACCES;
				goto TCA_SW_FIN;
			}
		}

		combophy_poweroff(TCPC_NC, event->dev_type);
		tca_dev.tca_cur_mode = TCPC_NC;
	}else {
		if(TCPC_NC == tca_dev.tca_cur_mode) {
			ret = combophy_poweron();
			if(ret) {
				ret = -ENODEV;
				goto TCA_SW_FIN;
			}
		}

		if(TCA_IRQ_SHORT == event->irq_type)
			hisi_dptx_hpd_trigger(event->irq_type, tca_dev.tca_cur_mode);
		else if (tca_dev.tca_cur_mode == event->mode_type) {
			if (event->dev_type <= ID_RISE_EVENT)
				hisi_usb_otg_event_sync((enum otg_dev_event_type)event->dev_type);
			if ((tca_dev.tca_cur_mode&TCPC_DP)&&(event->dev_type > ID_RISE_EVENT))
				hisi_dptx_hpd_trigger(event->irq_type, tca_dev.tca_cur_mode);
		}else {
			TCPC_MUX_CTRL_TYPE tca_old_mode = tca_dev.tca_cur_mode;
			ret = hisi_dptx_notify_switch();
			if (ret) {
				pr_err("[%s] hisi_dptx_notify_switch err\n", __func__);
				ret = -EIO;
				goto TCA_SW_FIN;
			}

			ret = tca_mode_switch(event->mode_type, event->typec_orien);
			if (ret) {
				pr_err("[%s] tca_mode_switch err\n", __func__);
				goto TCA_SW_FIN;
			}
/*
2）USB在位状态，考虑到USB可能存在数传，
PD不能直接做PHY的模式切换，必须先做一下拔出，再到新的状态。
USB->DP4:  USB->NC->DP4
USB->USB+DP4:USB->NC->USB+DP4
*/
			if (event->dev_type <= ID_RISE_EVENT) {
				hisi_usb_otg_event_sync((enum otg_dev_event_type)event->dev_type);
			}

			if (TCPC_DP & (tca_old_mode|tca_dev.tca_cur_mode))
				hisi_dptx_hpd_trigger(event->irq_type, tca_dev.tca_cur_mode);
			else {
				ret = hisi_dptx_triger(0);
				if (ret) {
					pr_err("[%s]hisi_dptx_triger err[%d][%d]\n",__func__, __LINE__,ret);
					ret = -ERANGE;
					goto TCA_SW_FIN;
				}
			}
		}
	}

TCA_SW_FIN:
	printk(HISI_TCA_DEBUG"\n[%s]++++\nret[%d]tca_cur_mode[%d]\n", __func__,
		ret, tca_dev.tca_cur_mode);
	dp_dfp_u_notify_dp_configuration_done(tca_dev.tca_cur_mode, ret);
}
/*lint -e715 +e655*/
void  tca_wq(struct work_struct *data)
{
	pd_event_t pd_event;
	unsigned long len;
	while (!kfifo_is_empty(&tca_dev.kfifo)) {
		memset((void*)&pd_event, 0, sizeof(pd_event_t));
		len = kfifo_out(&tca_dev.kfifo, &pd_event, (unsigned int)sizeof(pd_event_t));
		if (len != sizeof(pd_event_t))
			pr_err("[%s]kfifo_out  err\n", __func__);
		mutex_lock(&tca_mutex);
		pd_event_hander(&pd_event);
		mutex_unlock(&tca_mutex);
	}
}
/*lint +e715*/
static int __init pd_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *dev_node = dev->of_node;
	if (!of_device_is_available(dev_node))
		return -ENODEV;

	ret = pd_get_resource(&tca_dev, dev);
	if (ret) {
		pr_err("[%s] pd_get_resource err\n", __func__);
		return -EINVAL;
	}

	tca_dev.tca_poweron =  TCA_POWEROFF;
	tca_dev.wq = create_singlethread_workqueue("tca_wq");
	if (NULL == tca_dev.wq) {
		pr_err("[%s]tca_wq  err\n", __func__);
		return -EPIPE;
	}

	ret = kfifo_alloc(&tca_dev.kfifo, FIFO_SIZE, GFP_KERNEL);
	if (ret) {
		pr_err("[%s]kfifo_alloc  err[%d]\n", __func__, ret);
		return ret;
	}

	INIT_WORK(&tca_dev.work, tca_wq);
	tca_dev.init = PD_PLATFORM_INIT_OK;
	return ret;
}

static void tca_devouces_rel(void)
{
	iounmap(tca_dev.usb_misc_base);
	iounmap(tca_dev.pctrl_reg_base);
	iounmap(tca_dev.sctrl_reg_base);
	iounmap(tca_dev.crgperi_reg_base);
}
/*lint -e705*/
static int pd_remove(struct platform_device *pdev)
{
	tca_devouces_rel();
	kfifo_free(&tca_dev.kfifo);
	return 0;
}
/*lint +e705*/
/*lint -e785 -e64*/
static struct of_device_id hisi_pd_of_match[] = {
	{ .compatible = "hisilicon,pd"},
	{},
};

MODULE_DEVICE_TABLE(of, hisi_pd_of_match);
static struct platform_driver pd_platdrv = {
	.driver = {
		.name		= "hisi-pd",
		.owner		= THIS_MODULE,
		.of_match_table = hisi_pd_of_match,
	},
	.probe	= pd_probe,
	.remove	= pd_remove,
};
/*lint +e785*/
/*lint -e528  -e721*/
module_platform_driver(pd_platdrv);
/*lint +e528 +749 +64 +e721*/

