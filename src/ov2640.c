#include "ov2640.h"
#include "ov2640_cfg.h"
#include <string.h>

#define RT_OV2640_DEBUG

#define DBG_TAG               "OV2640"
#ifdef RT_OV2640_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

struct cam_device _cam;

static rt_err_t cam_init(struct rt_device *device){
    cam_device_t cam = (cam_device_t)device;
    /* nothing, right now */
    cam = cam;
    return RT_EOK;
}

static rt_err_t cam_control(struct rt_device *device, int cmd, void *args){
    switch (cmd)
    {
        case CAM_JPEG_Mode:
        {
            //设置:YUV422格式
            for(int i=0;i<(sizeof(ov2640_yuv422_init)/2);i++)
            {
                _cam.ops->write_reg(ov2640_yuv422_init[i][0],ov2640_yuv422_init[i][1]);
            }

            //设置:输出JPEG数据
            for(int i=0;i<(sizeof(ov2640_jpeg_init)/2);i++)
            {
                _cam.ops->write_reg(ov2640_jpeg_init[i][0],ov2640_jpeg_init[i][1]);
            }
            break;
        }
        case CAM_RGB565_Mode:
        {
            //设置:RGB565输出
            for(int i=0;i<(sizeof(ov2640_rgb565_init)/2);i++)
            {
                _cam.ops->write_reg(ov2640_rgb565_init[i][0],ov2640_rgb565_init[i][1]);
            }
            break;
        }
        case CAM_Auto_Exposure:
        {
            RT_ASSERT(args);
            rt_uint8_t *p=(rt_uint8_t*)ov2640_autoexpo_level[*(rt_uint16_t *)args];
            for(int i=0;i<4;i++)
            {
                _cam.ops->write_reg(p[i*2],p[i*2+1]);
            }
            break;
        }
        case CAM_Light_Mode:
        {
            RT_ASSERT(args);
            rt_uint8_t regccval=0X5E;//Sunny
            rt_uint8_t regcdval=0X41;
            rt_uint8_t regceval=0X54;
            switch(*(rt_uint16_t *)args)
            {
                case 0://auto
                    _cam.ops->write_reg(0XFF,0X00);
                    _cam.ops->write_reg(0XC7,0X10);//AWB ON
                    return RT_EOK;
                case 2://cloudy
                    regccval=0X65;
                    regcdval=0X41;
                    regceval=0X4F;
                    break;
                case 3://office
                    regccval=0X52;
                    regcdval=0X41;
                    regceval=0X66;
                    break;
                case 4://home
                    regccval=0X42;
                    regcdval=0X3F;
                    regceval=0X71;
                    break;
            }
            _cam.ops->write_reg(0XFF,0X00);
            _cam.ops->write_reg(0XC7,0X40); //AWB OFF
            _cam.ops->write_reg(0XCC,regccval);
            _cam.ops->write_reg(0XCD,regcdval);
            _cam.ops->write_reg(0XCE,regceval);
            break;
        }
        case CAM_Color_Saturation:
        {
            RT_ASSERT(args);
            rt_uint8_t reg7dval=((*(rt_uint16_t *)args+2)<<4)|0X08;
            _cam.ops->write_reg(0XFF,0X00);
            _cam.ops->write_reg(0X7C,0X00);
            _cam.ops->write_reg(0X7D,0X02);
            _cam.ops->write_reg(0X7C,0X03);
            _cam.ops->write_reg(0X7D,reg7dval);
            _cam.ops->write_reg(0X7D,reg7dval);
            break;
        }
        case CAM_Brightness:
        {
            RT_ASSERT(args);
            _cam.ops->write_reg(0xff, 0x00);
            _cam.ops->write_reg(0x7c, 0x00);
            _cam.ops->write_reg(0x7d, 0x04);
            _cam.ops->write_reg(0x7c, 0x09);
            _cam.ops->write_reg(0x7d, ((rt_uint8_t)(*(rt_uint16_t *)args))<<4);
            _cam.ops->write_reg(0x7d, 0x00);
            break;
        }
        case CAM_Contrast:
        {
            RT_ASSERT(args);
            rt_uint8_t reg7d0val=0X20;//默认为普通模式
            rt_uint8_t reg7d1val=0X20;
            switch(*(rt_uint16_t *)args)
            {
                case 0://-2
                    reg7d0val=0X18;
                    reg7d1val=0X34;
                    break;
                case 1://-1
                    reg7d0val=0X1C;
                    reg7d1val=0X2A;
                    break;
                case 3://1
                    reg7d0val=0X24;
                    reg7d1val=0X16;
                    break;
                case 4://2
                    reg7d0val=0X28;
                    reg7d1val=0X0C;
                    break;
            }
            _cam.ops->write_reg(0xff,0x00);
            _cam.ops->write_reg(0x7c,0x00);
            _cam.ops->write_reg(0x7d,0x04);
            _cam.ops->write_reg(0x7c,0x07);
            _cam.ops->write_reg(0x7d,0x20);
            _cam.ops->write_reg(0x7d,reg7d0val);
            _cam.ops->write_reg(0x7d,reg7d1val);
            _cam.ops->write_reg(0x7d,0x06);
            break;
        }
        case CAM_Special_Effects:
        {
            RT_ASSERT(args);
            rt_uint8_t reg7d0val=0X00;//默认为普通模式
            rt_uint8_t reg7d1val=0X80;
            rt_uint8_t reg7d2val=0X80;
            switch(*(rt_uint16_t *)args)
            {
                case 1://负片
                    reg7d0val=0X40;
                    break;
                case 2://黑白
                    reg7d0val=0X18;
                    break;
                case 3://偏红色
                    reg7d0val=0X18;
                    reg7d1val=0X40;
                    reg7d2val=0XC0;
                    break;
                case 4://偏绿色
                    reg7d0val=0X18;
                    reg7d1val=0X40;
                    reg7d2val=0X40;
                    break;
                case 5://偏蓝色
                    reg7d0val=0X18;
                    reg7d1val=0XA0;
                    reg7d2val=0X40;
                    break;
                case 6://复古
                    reg7d0val=0X18;
                    reg7d1val=0X40;
                    reg7d2val=0XA6;
                    break;
            }
            _cam.ops->write_reg(0xff,0x00);
            _cam.ops->write_reg(0x7c,0x00);
            _cam.ops->write_reg(0x7d,reg7d0val);
            _cam.ops->write_reg(0x7c,0x05);
            _cam.ops->write_reg(0x7d,reg7d1val);
            _cam.ops->write_reg(0x7d,reg7d2val);
            break;
        }
        case CAM_Color_Bar:
        {
            RT_ASSERT(args);
            rt_uint8_t reg;
            _cam.ops->write_reg(0XFF,0X01);
            reg=_cam.ops->read_reg(0X12);
            reg&=~(1<<1);
            if(*(rt_uint16_t *)args)reg|=1<<1;
            _cam.ops->write_reg(0X12,reg);
            break;
        }
        case CAM_Window_Set:
        {
            rt_uint16_t endx;
            rt_uint16_t endy;
            rt_uint8_t temp;
            RT_ASSERT(args);
            rt_uint16_t *para=(rt_uint16_t *)args;
            endx=para[0]+para[2]/2;    //V*2
            endy=para[1]+para[3]/2;

            _cam.ops->write_reg(0XFF,0X01);
            temp=_cam.ops->read_reg(0X03);             //读取Vref之前的值
            temp&=0XF0;
            temp|=((endy&0X03)<<2)|(para[1]&0X03);
            _cam.ops->write_reg(0X03,temp);             //设置Vref的start和end的最低2位
            _cam.ops->write_reg(0X19,para[1]>>2);            //设置Vref的start高8位
            _cam.ops->write_reg(0X1A,endy>>2);          //设置Vref的end的高8位

            temp=_cam.ops->read_reg(0X32);             //读取Href之前的值
            temp&=0XC0;
            temp|=((endx&0X07)<<3)|(para[0]&0X07);
            _cam.ops->write_reg(0X32,temp);             //设置Href的start和end的最低3位
            _cam.ops->write_reg(0X17,para[0]>>3);            //设置Href的start高8位
            _cam.ops->write_reg(0X18,endx>>3);          //设置Href的end的高8位
            break;
        }
        case CAM_OutSize_Set:
        {
            rt_uint16_t outh;
            rt_uint16_t outw;
            rt_uint8_t temp;
            RT_ASSERT(args);
            rt_uint16_t *para=(rt_uint16_t *)args;
            RT_ASSERT(para[0]%4==0);
            RT_ASSERT(para[1]%4==0);
            outw=para[0]/4;
            outh=para[1]/4;
            _cam.ops->write_reg(0XFF,0X00);
            _cam.ops->write_reg(0XE0,0X04);
            _cam.ops->write_reg(0X5A,outw&0XFF);        //设置OUTW的低八位
            _cam.ops->write_reg(0X5B,outh&0XFF);        //设置OUTH的低八位
            temp=(outw>>8)&0X03;
            temp|=(outh>>6)&0X04;
            _cam.ops->write_reg(0X5C,temp);             //设置OUTH/OUTW的高位
            _cam.ops->write_reg(0XE0,0X00);
            break;
        }
        case CAM_ImageWin_Set:
        {
            rt_uint16_t hsize;
            rt_uint16_t vsize;
            rt_uint8_t temp;
            RT_ASSERT(args);
            rt_uint16_t *para=(rt_uint16_t *)args;
            RT_ASSERT(para[2]%4==0);
            RT_ASSERT(para[3]%4==0);
            hsize=para[2]/4;
            vsize=para[3]/4;
            _cam.ops->write_reg(0XFF,0X00);
            _cam.ops->write_reg(0XE0,0X04);
            _cam.ops->write_reg(0X51,hsize&0XFF);       //设置H_SIZE的低八位
            _cam.ops->write_reg(0X52,vsize&0XFF);       //设置V_SIZE的低八位
            _cam.ops->write_reg(0X53,para[0]&0XFF);        //设置para[0]的低八位
            _cam.ops->write_reg(0X54,para[1]&0XFF);        //设置para[1]的低八位
            temp=(vsize>>1)&0X80;
            temp|=(para[1]>>4)&0X70;
            temp|=(hsize>>5)&0X08;
            temp|=(para[0]>>8)&0X07;
            _cam.ops->write_reg(0X55,temp);             //设置H_SIZE/V_SIZE/para[0],para[1]的高位
            _cam.ops->write_reg(0X57,(hsize>>2)&0X80);  //设置H_SIZE/V_SIZE/para[0],para[1]的高位
            _cam.ops->write_reg(0XE0,0X00);
            break;
        }
        case CAM_ImageSize_Set:
        {
            rt_uint8_t temp;
            RT_ASSERT(args);
            rt_uint16_t *para=(rt_uint16_t *)args;
            _cam.ops->write_reg(0XFF,0X00);
            _cam.ops->write_reg(0XE0,0X04);
            _cam.ops->write_reg(0XC0,(para[0])>>3&0XFF);      //设置HSIZE的10:3位
            _cam.ops->write_reg(0XC1,(para[1])>>3&0XFF);     //设置VSIZE的10:3位
            temp=(para[0]&0X07)<<3;
            temp|=para[1]&0X07;
            temp|=(para[0]>>4)&0X80;
            _cam.ops->write_reg(0X8C,temp);
            _cam.ops->write_reg(0XE0,0X00);
            break;
        }
    }
    return RT_EOK;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops cam_ops =
{
    cam_init,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    cam_control
};
#endif

rt_err_t cam_hw_init(struct cam_ops *ops)
{
    rt_err_t result = RT_EOK;
    struct rt_device *device = &_cam.parent;
    rt_uint16_t chip_mid=0;
    rt_uint16_t chip_pid=0;
    RT_ASSERT(ops);
    memset(&_cam, 0x00, sizeof(_cam));
    result = rt_sem_init(&_cam.cam_lock, "cam_lock", 0, RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        LOG_E("init semaphore failed!\n");
        result = -RT_ENOMEM;
        goto __exit;
    }
    device->type    = RT_Device_Class_Graphic;
#ifdef RT_USING_DEVICE_OPS
    device->ops     = &cam_ops;
#else
    device->init    = cam_init;
    device->control = cam_control;
#endif
    rt_device_register(device, "cam", RT_DEVICE_FLAG_RDWR);

    _cam.ops=ops;
    if (_cam.ops->hal_init() != RT_EOK)
    {
        result = -RT_ERROR;
        goto __exit;
    }

    _cam.ops->pwdn_pin_write(PIN_LOW);
    _cam.ops->hw_delayms(10);
    _cam.ops->reset_pin_write(PIN_LOW);
    _cam.ops->hw_delayms(10);
    _cam.ops->reset_pin_write(PIN_HIGH);

    _cam.ops->write_reg(OV2640_DSP_RA_DLMT, 0x01);
    _cam.ops->write_reg(OV2640_SENSOR_COM7, 0x80);

    chip_mid=_cam.ops->read_reg(OV2640_SENSOR_MIDH);
    chip_mid<<=8;
    chip_mid|=_cam.ops->read_reg(OV2640_SENSOR_MIDL);

    chip_pid=_cam.ops->read_reg(OV2640_SENSOR_PIDH);
    chip_pid<<=8;
    chip_pid|=_cam.ops->read_reg(OV2640_SENSOR_PIDL);
    if (chip_mid!=OV2640_MID||chip_pid!=OV2640_PID) {
        LOG_E("ov2640 id error:mid = 0x%04X [should be 0x%04X] , pid = 0x%04X [should be 0x%04X].", chip_mid,OV2640_MID,chip_pid,OV2640_PID);
        result=-RT_ERROR;
        goto __exit;
    }

    for(int i=0;i<sizeof(ov2640_sxga_init)/2;i++){
        _cam.ops->write_reg(ov2640_sxga_init[i][0],ov2640_sxga_init[i][1]);
    }

    LOG_D("ov2640 init done");

__exit:
    if (result != RT_EOK)
    {
        rt_sem_delete(&_cam.cam_lock);
    }
    return result;
}

