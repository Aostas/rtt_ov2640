#include <ov2640.h>

#define DBG_TAG               "OV2640"
#ifdef RT_OV2640_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>


void ov2640_write_reg(ov2640_device_t dev, rt_uint8_t reg_addr,rt_uint8_t *data){
    struct rt_i2c_msg msg[2];
    msg[0].addr     = dev->i2c_addr;
    msg[0].flags    = RT_I2C_WR;
    msg[0].len      = 1;
    msg[0].buf      = &reg_addr;
    msg[1].addr     = dev->i2c_addr;
    msg[1].flags    = RT_I2C_WR | RT_I2C_NO_START;
    msg[1].len      = 1;
    msg[1].buf      = data;
    rt_i2c_transfer(dev->bus, msg, 2);
//    rt_uint8_t buf[2]={0};
//    buf[0]=reg_addr;
//    buf[1]=data;
//    rt_device_write(&dev->bus->parent, dev->i2c_addr, &buf, RT_NULL);
}

rt_uint8_t ov2640_read_reg(ov2640_device_t dev, rt_uint8_t reg_addr){
    struct rt_i2c_msg msg[2];
    rt_uint8_t data=0;
    msg[0].addr  = dev->i2c_addr;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = &reg_addr;
    msg[1].addr  = dev->i2c_addr;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = 1;
    msg[1].buf   = &data;

    rt_i2c_transfer(dev->bus, msg, 2);
    return data;


//    rt_uint8_t buf[2]={0};
//    buf[0]=reg_addr;
//    rt_device_read(&dev->bus->parent, dev->i2c_addr, &buf, RT_NULL);
//    return buf[1];
}


ov2640_device_t ov2640_hw_init(const char *dev_name, rt_uint8_t i2c_addr)
{
//    sccb_device_t dev=RT_NULL;
//    RT_ASSERT(dev_name);
//
//    dev = rt_calloc(1, sizeof(struct sccb_device));
//    if (dev == RT_NULL)
//    {
//        LOG_E("Can't allocate memory for sccb device on '%s' ", dev_name);
//        goto __exit;
//    }
//
//    dev->bus = (struct rt_sccb_bus_device *)rt_device_find(dev_name);
//    if (dev->bus == RT_NULL)
//    {
//        LOG_E("%s bus not found!", dev_name);
//        goto __exit;
//    }
//
//    if (sccb_addr != RT_NULL)
//        dev->sccb_addr = sccb_addr;
//    else
//        dev->sccb_addr = OV2640_ADDR_DEFAULT;
//
//    if (rt_device_open(&dev->bus->parent, RT_NULL) != RT_EOK)
//    {
//        LOG_E("%s bus opened failed!", dev_name);
//        goto __exit;
//    }
//
//    return dev;
//
//__exit:
//    if (dev != RT_NULL)
//        rt_free(dev);
//
//    return RT_NULL;


    ov2640_device_t dev = RT_NULL;

    RT_ASSERT(dev_name);

    dev = rt_calloc(1, sizeof(struct ov2640_device));
    if (dev == RT_NULL)
    {
        LOG_E("Can't allocate memory for ov2640 device on '%s' ", dev_name);
        goto __exit;
    }

    dev->bus = (struct rt_i2c_bus_device *)rt_device_find(dev_name);
    if (dev->bus == RT_NULL)
    {
        LOG_E("i2c_bus %s for ov2640 not found!", dev_name);
        goto __exit;
    }

    if (i2c_addr != RT_NULL)
        dev->i2c_addr = i2c_addr;
    else
        dev->i2c_addr = OV2640_ADDR_DEFAULT;

    if (rt_device_open(&dev->bus->parent, RT_NULL) != RT_EOK)
    {
        LOG_D("i2c_bus %s for ov2640 opened failed!", dev_name);
        goto __exit;
    }

    LOG_D("ov2640 init done", dev_name);
    return dev;

__exit:
    if (dev != RT_NULL)
        rt_free(dev);

    return RT_NULL;
}
