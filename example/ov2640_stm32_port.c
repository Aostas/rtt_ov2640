#include <rtthread.h>
#include "ov2640.h"
#include "pcf8574.h"

#define DBG_TAG               "OV2640_PORT"
#ifdef RT_OV2640_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

#define I2C_BUS_NAME    "i2c2"
#define PCF8574_ADDR    0x20
#define PWDN_PIN_PCF8574 2

/* pcf8574 hardware init */
pcf8574_device_t pwdn_hw_init(char * dev_name, int addr)
{
    static pcf8574_device_t dev;

    if (dev == RT_NULL)
    {
        dev = pcf8574_init(dev_name, addr);
        if (dev == RT_NULL)
        {
            LOG_E("i2c_bus %s for PCF8574 not found!", dev_name);
            return RT_NULL;
        } else {
            return dev;
        }
    } else
    {
        return dev;
    }
}

int ov2640_port_init(void){
    rt_uint16_t chip_mid=0;
    rt_uint16_t chip_pid=0;
    ov2640_device_t ov2640_dev = RT_NULL;
    pcf8574_device_t pwdn_dev = RT_NULL;

    ov2640_dev=ov2640_hw_init("i2c1", OV2640_ADDR_DEFAULT);
    if (ov2640_dev == RT_NULL)
    {
        LOG_E("ov2640 no found");
        return 0;
    }

    pwdn_dev = pwdn_hw_init(I2C_BUS_NAME, PCF8574_ADDR);
    if (pwdn_dev == RT_NULL)
    {
        LOG_E("pcf8574 device no found");
        return 0;
    }

    rt_pin_mode(OV2640_RESET_PIN, PIN_MODE_OUTPUT);

    pcf8574_pin_write(pwdn_dev, PWDN_PIN_PCF8574, 0);
    rt_thread_mdelay(10);
    rt_pin_write(OV2640_RESET_PIN, PIN_LOW);
    rt_thread_mdelay(10);
    rt_pin_write(OV2640_RESET_PIN, PIN_HIGH);

    rt_thread_mdelay(50);
    ov2640_write_reg(ov2640_dev, OV2640_DSP_RA_DLMT, 0x01);
    ov2640_write_reg(ov2640_dev, OV2640_SENSOR_COM7, 0x80);
    rt_thread_mdelay(50);

    chip_mid=ov2640_read_reg(ov2640_dev,OV2640_SENSOR_MIDH);
    chip_mid<<=8;
    chip_mid|=ov2640_read_reg(ov2640_dev,OV2640_SENSOR_MIDL);
    rt_kprintf("\nov2640  chip_mid: [0x%02X]\n", chip_mid);

    chip_pid=ov2640_read_reg(ov2640_dev,OV2640_SENSOR_PIDH);
    chip_pid<<=8;
    chip_pid|=ov2640_read_reg(ov2640_dev,OV2640_SENSOR_PIDL);
    rt_kprintf("\nov2640  chip_pid: [0x%02X]\n", chip_pid);
    return 0;
}

INIT_APP_EXPORT(ov2640_port_init);
