


#include <linux/module.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>

#include "hwsensor.h"
#include "sensor_commom.h"
#include "hw_csi.h"

#define I2S(i) container_of(i, sensor_t, intf)

/*lint -save -e846 -e514 -e866 -e30 -e84 -e785 -e64*/
/*lint -save -e826 -e838 -e715 -e747 -e778 -e774 -e732*/
/*lint -save -e650 -e31 -e731 -e528 -e753 -e737*/

static bool s_ov20880_bac_power_on = false;/*false: power off, true:power on*/
extern struct hw_csi_pad hw_csi_pad;

//#define POWER_SETTING_DELAY_5 5 now POWER_SETTING_DELAY_5 and POWER_SETTING_DELAY_0 is not used
#define POWER_SETTING_DELAY_1 1
//#define POWER_SETTING_DELAY_0 0
char const* ov20880_bac_get_name(hwsensor_intf_t* si);
int ov20880_bac_config(hwsensor_intf_t* si, void  *argp);
int ov20880_bac_power_up(hwsensor_intf_t* si);
int ov20880_bac_power_down(hwsensor_intf_t* si);
int ov20880_bac_match_id(hwsensor_intf_t* si, void * data);
int ov20880_bac_csi_enable(hwsensor_intf_t* si);
int ov20880_bac_csi_disable(hwsensor_intf_t* si);

static hwsensor_vtbl_t s_ov20880_bac_vtbl=
{
    .get_name = ov20880_bac_get_name,
    .config = ov20880_bac_config,
    .power_up = ov20880_bac_power_up,
    .power_down = ov20880_bac_power_down,
    .match_id = ov20880_bac_match_id,
    .csi_enable = ov20880_bac_csi_enable,
    .csi_disable = ov20880_bac_csi_disable,
};
struct sensor_power_setting hw_ov20880_bac_power_setting[] = {

    /*MCAM AVDD 2.85V*/
    {
        .seq_type = SENSOR_AVDD2,
        .config_val = LDO_VOLTAGE_V2P8V,
        .sensor_index = SENSOR_INDEX_INVALID,
        .delay = POWER_SETTING_DELAY_1,
    },
    
    /*MCAM1 IOVDD 1.80V*/
    {
        .seq_type = SENSOR_IOVDD_EN,
        .config_val = SENSOR_GPIO_LOW,
        .sensor_index = SENSOR_INDEX_INVALID,
        .delay = POWER_SETTING_DELAY_1,
    },

    /*MCAM1 DVDD 1.05V*/
    {
        .seq_type = SENSOR_DVDD2,
        .config_val = LDO_VOLTAGE_1P05V,
        .sensor_index = SENSOR_INDEX_INVALID,
        .delay = POWER_SETTING_DELAY_1,
    },
/*	
    //SCAM AVDD2 1.8V gpio36
    {
        .seq_type = SENSOR_AVDD1_EN,
        .config_val = SENSOR_GPIO_LOW,
        .sensor_index = SENSOR_INDEX_INVALID,
        .delay = POWER_SETTING_DELAY_0,
    },

	
    //SCAM AVDD 2.85v gpio17
    {
        .seq_type = SENSOR_AVDD2_EN,
        .config_val = SENSOR_GPIO_LOW,
        .sensor_index = SENSOR_INDEX_INVALID,
        .delay = POWER_SETTING_DELAY_0,
    },
*/	

    /*MCAM MCLK*/
    {
        .seq_type = SENSOR_MCLK,
        .sensor_index = SENSOR_INDEX_INVALID,
        .delay = POWER_SETTING_DELAY_1,
    },
	
     /*MCAM RST*/
    {
        .seq_type = SENSOR_RST,
        .config_val = SENSOR_GPIO_LOW,
        .sensor_index = SENSOR_INDEX_INVALID,
        .delay = POWER_SETTING_DELAY_1,
    },

};


atomic_t volatile ov20880_bac_powered = ATOMIC_INIT(0);/*init*/
sensor_t s_ov20880_bac =
{
    .intf = { .vtbl = &s_ov20880_bac_vtbl, },
    .power_setting_array = {
            .size = ARRAY_SIZE(hw_ov20880_bac_power_setting),
            .power_setting = hw_ov20880_bac_power_setting,
    },
    .p_atpowercnt = &ov20880_bac_powered,
};

const struct of_device_id
s_ov20880_bac_dt_match[] =
{
    {
        .compatible = "huawei,ov20880_bac",
        .data = &s_ov20880_bac.intf,
    },
    {
    },/* terminate list */
};

MODULE_DEVICE_TABLE(of, s_ov20880_bac_dt_match);

struct platform_driver
s_ov20880_bac_driver =
{
    .driver =
    {
        .name = "huawei,ov20880_bac",
        .owner = THIS_MODULE,
        .of_match_table = s_ov20880_bac_dt_match,
    },
};

char const* ov20880_bac_get_name(hwsensor_intf_t* si)
{
    sensor_t* sensor = NULL;

    if (NULL == si) {
        cam_err("%s. si is NULL.", __func__);
        return NULL;
    }

    sensor = I2S(si);
    if(NULL == sensor->board_info->name){
        cam_err("%s. sensor->board_info->name is NULL.", __func__);
        return NULL;
    }
    return sensor->board_info->name;
}

int ov20880_bac_power_up(hwsensor_intf_t* si)
{
    int ret = 0;
    sensor_t* sensor = NULL;

    if (NULL == si) {
        cam_err("%s. si is NULL.", __func__);
        return -EINVAL;
    }

    sensor = I2S(si);
    if ((NULL == sensor->board_info) || (NULL == sensor->board_info->name)){
        cam_err("%s. sensor->board_info or sensor->board_info->name is NULL .", __func__);
        return -EINVAL;
    }
    cam_info("enter %s. index = %d name = %s", __func__, sensor->board_info->sensor_index, sensor->board_info->name);
    if (hw_is_fpga_board()) {
        ret = do_sensor_power_on(sensor->board_info->sensor_index, sensor->board_info->name);
    } else {
        ret = hw_sensor_power_up(sensor);
    }
    if (0 == ret )
    {
        cam_info("%s. power up sensor success.", __func__);
    }
    else
    {
        cam_err("%s. power up sensor fail.", __func__);
    }
    return ret;
}

int ov20880_bac_power_down(hwsensor_intf_t* si)
{
    int ret = 0;
    sensor_t* sensor = NULL;

    if (NULL == si) {
        cam_err("%s. si is NULL.", __func__);
        return -EINVAL;
    }

    sensor = I2S(si);
    if ((NULL == sensor->board_info) || (NULL == sensor->board_info->name)){
         cam_err("%s. sensor->board_info or sensor->board_info->name is NULL .", __func__);
         return -EINVAL;
    }
    cam_info("enter %s. index = %d name = %s", __func__, sensor->board_info->sensor_index, sensor->board_info->name);
    if (hw_is_fpga_board()) {
        ret = do_sensor_power_off(sensor->board_info->sensor_index, sensor->board_info->name);
    } else {
        ret = hw_sensor_power_down(sensor);
    }
    if (0 == ret )
    {
        cam_info("%s. power down sensor success.", __func__);
    }
    else
    {
        cam_err("%s. power down sensor fail.", __func__);
    }
    return ret;
}

int ov20880_bac_csi_enable(hwsensor_intf_t* si)
{
    return 0;
}

int ov20880_bac_csi_disable(hwsensor_intf_t* si)
{
    return 0;
}

int ov20880_bac_match_id(hwsensor_intf_t* si, void * data)
{
    sensor_t* sensor = NULL;
    struct sensor_cfg_data *cdata = NULL;

    cam_info("%s enter.", __func__);

    if ((NULL == si) || ( NULL == data)) {
        cam_err("%s. si is NULL.", __func__);
        return -EINVAL;
    }

    sensor = I2S(si);
    if ((NULL == sensor->board_info) || (NULL == sensor->board_info->name)){
        cam_err("%s. sensor->board_info or sensor->board_info->name is NULL .", __func__);
        return -EINVAL;
    }
    cdata  = (struct sensor_cfg_data *)data;
    cdata->data = sensor->board_info->sensor_index;

    cam_info("%s name:%s", __func__, sensor->board_info->name);
    return 0;

}

int
ov20880_bac_config(
        hwsensor_intf_t* si,
        void  *argp)
{
    struct sensor_cfg_data *data;
    int ret =0;

    if (NULL == si || NULL == argp){
        cam_err("%s : si or argp is null", __func__);
        return -EINVAL;
    }

    data = (struct sensor_cfg_data *)argp;
    cam_debug("ov20880_bac cfgtype = %d",data->cfgtype);
    if (NULL == si->vtbl) {
	cam_err("%s :  si->vtbl is null", __func__);
        return -EINVAL;
    }
    switch(data->cfgtype){
        case SEN_CONFIG_POWER_ON:
            if (!s_ov20880_bac_power_on)
            {
                if(NULL == si->vtbl->power_up){
                    cam_err("%s. si->vtbl->power_up is null.", __func__);
                    ret=-EINVAL;
                }else{
                    ret = si->vtbl->power_up(si);
                    if (0 == ret)
                    {
                        s_ov20880_bac_power_on = true;
                    }else{
                        cam_err("%s. power up fail.", __func__);
                    }
                }
            }   
            break;
        case SEN_CONFIG_POWER_OFF:
            if (s_ov20880_bac_power_on)
            {
                if(NULL == si->vtbl->power_down){
                    cam_err("%s. si->vtbl->power_down is null.", __func__);
                    ret=-EINVAL;
                }else{
                    ret = si->vtbl->power_down(si);
                    if (0 == ret)
                    {
                        s_ov20880_bac_power_on = false;
                    }else{
                        cam_err("%s. power down fail.", __func__);
                    }
                }
            }

            break;
        case SEN_CONFIG_WRITE_REG:
            break;
        case SEN_CONFIG_READ_REG:
            break;
        case SEN_CONFIG_WRITE_REG_SETTINGS:
            break;
        case SEN_CONFIG_READ_REG_SETTINGS:
            break;
        case SEN_CONFIG_ENABLE_CSI:
            break;
        case SEN_CONFIG_DISABLE_CSI:
            break;
        case SEN_CONFIG_MATCH_ID:
            if(NULL == si->vtbl->match_id){
                cam_err("%s. si->vtbl->match_id is null.", __func__);
            }else{
                ret = si->vtbl->match_id(si,argp);
            }
            break;
        default:
            cam_err("%s cfgtype(%d) is error", __func__, data->cfgtype);
            break;
    }
    return ret;
}

int32_t
ov20880_bac_platform_probe(
        struct platform_device* pdev)
{
    int rc = 0;
    if(NULL == pdev){
        cam_err("%s platform_device is NULL.\n", __func__);
        goto ov20880_bac_sensor_probe_fail;
    }
    cam_notice("enter %s",__func__);
    if (pdev->dev.of_node) {
        rc = hw_sensor_get_dt_data(pdev, &s_ov20880_bac);
        if (rc < 0) {
            cam_err("%s failed line %d\n", __func__, __LINE__);
            goto ov20880_bac_sensor_probe_fail;
        }
    } else {
        cam_err("%s ov20880_bac of_node is NULL.\n", __func__);
        goto ov20880_bac_sensor_probe_fail;
    }

    s_ov20880_bac.dev = &pdev->dev;

    rc = hwsensor_register(pdev, &s_ov20880_bac.intf);
    if (rc < 0)
    {
        hwsensor_unregister(&s_ov20880_bac.intf);
        cam_err("%s hwsensor_register fail.\n", __func__);
        goto ov20880_bac_sensor_probe_fail;
    }
    rc = rpmsg_sensor_register(pdev, (void*)&s_ov20880_bac);
    if (rc < 0)
    {
        rpmsg_sensor_unregister((void*)&s_ov20880_bac);
        cam_err("%s rpmsg_sensor_register fail.\n", __func__);
        goto ov20880_bac_sensor_probe_fail;
    }

ov20880_bac_sensor_probe_fail:
    return rc;
}

int __init
ov20880_bac_init_module(void)
{
    cam_notice("enter %s",__func__);
    return platform_driver_probe(&s_ov20880_bac_driver,
            ov20880_bac_platform_probe);
}

void __exit
ov20880_bac_exit_module(void)
{
    rpmsg_sensor_unregister((void*)&s_ov20880_bac);
    hwsensor_unregister(&s_ov20880_bac.intf);
    platform_driver_unregister(&s_ov20880_bac_driver);
	return;
}
//lint -restore

/*lint -e528 -esym(528,*)*/
module_init(ov20880_bac_init_module);
module_exit(ov20880_bac_exit_module);
/*lint -e528 +esym(528,*)*/
/*lint -e753 -esym(753,*)*/
MODULE_DESCRIPTION("ov20880_bac");
MODULE_LICENSE("GPL v2");
/*lint -e753 +esym(753,*)*/
