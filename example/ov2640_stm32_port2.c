#include <rtthread.h>
#include <drv_dcmi.h>
#include "ov2640.h"
#include "pcf8574.h"
#include "lwip/api.h"

#define DBG_TAG               "OV2640"

//#define PKG_OV2640_DEBUG

#ifdef PKG_OV2640_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

#define PCF8574_BUS_NAME    "i2c2"
#define PCF8574_ADDR    0x20
#define PWDN_PIN_PCF8574 2

void reset_pin_init(void)
{
    rt_pin_mode(OV2640_RESET_PIN, PIN_MODE_OUTPUT);
}

void reset_pin_write(rt_uint8_t value)
{
    rt_pin_write(OV2640_RESET_PIN, value);
}

void pwdn_pin_init()
{
    return;
}

void pwdn_pin_write(rt_uint8_t value)
{
    static pcf8574_device_t dev;
    if (dev == RT_NULL)
    {
        dev = pcf8574_init(PCF8574_BUS_NAME, PCF8574_ADDR);
        if (dev == RT_NULL)
        {
            LOG_E("i2c_bus %s for PCF8574 not found!", PCF8574_BUS_NAME);
            return ;
        }
    }
    pcf8574_pin_write(dev, PWDN_PIN_PCF8574, value);
}

static struct ov2640_ops stm32_ops={
    .dev_addr       = OV2640_ADDR_DEFAULT,
    .reset_pin_init = reset_pin_init,
    .reset_pin_write= reset_pin_write,
    .pwdn_pin_init  = pwdn_pin_init,
    .pwdn_pin_write = pwdn_pin_write,
};
#define NETCAM_FIFO_NUM         100         //定义FIFO数量
#define NETCAM_LINE_SIZE        4096        //定义行大小(*4字节)
rt_uint32_t *line_buf;
rt_uint16_t fifo_rd_pos=0;
rt_uint16_t fifo_wr_pos=0;
rt_uint32_t *netcam_fifo[NETCAM_FIFO_NUM];
rt_uint32_t *netcam_tx_buff;
rt_err_t netcam_fifo_write(rt_uint32_t *buf){
    rt_uint16_t pos=fifo_wr_pos++;
    if(fifo_wr_pos>=NETCAM_FIFO_NUM)
        fifo_wr_pos=0;
    if(fifo_wr_pos==fifo_rd_pos){
        fifo_wr_pos=pos;
        return RT_EBUSY;
    }
    for(int i=0;i<NETCAM_LINE_SIZE;i++){
        netcam_fifo[fifo_wr_pos][i]=buf[i];
    }
    return RT_EOK;
}

rt_err_t netcam_fifo_read(rt_uint32_t **buf){
    if(fifo_rd_pos==fifo_wr_pos)
        return RT_EEMPTY;
    LOG_I("if(fifo_rd_pos==fifo_wr_pos)");
    if(fifo_rd_pos>=NETCAM_FIFO_NUM){
        fifo_rd_pos=0;
    }
    LOG_I("if(fifo_rd_pos>=NETCAM_FIFO_NUM)");
    *buf=netcam_fifo[fifo_rd_pos];
    LOG_I("*buf=netcam_fifo[fifo_rd_pos];");
    return RT_EOK;
}

void dcmi_rx_callback(void){
    rt_err_t err=netcam_fifo_write(line_buf);
    if(err==RT_EOK)
        LOG_D("fifo_write_ok");
    else
        LOG_E("fifo_write_error:%d",err);
}
struct netconn *newconn=RT_NULL;
int ov2640_port_init(void);
int ov2640_deinit(void);
static void netcam_thread_entry(void *param){
    rt_err_t err=0;
    struct netconn *conn;
    static ip_addr_t client_ip;
    static rt_uint16_t client_port;

    LWIP_UNUSED_ARG(param);
    conn=netconn_new(NETCONN_TCP);
    netconn_bind(conn, IP_ADDR_ANY, 8088);
    netconn_listen(conn);
    while(1){
        err=netconn_accept(conn, &newconn);
        if(err==RT_EOK){
            netconn_getaddr(newconn, &client_ip, &client_port, 0);
            LOG_I("ip:0x%X,port:%d",client_ip,client_port);
            //stm32_dcmi_dma_start(line_buf,NETCAM_LINE_SIZE);
            ov2640_port_init();
            LOG_I("ov2640_port_init");
            rt_thread_mdelay(2000);
            LOG_I("rt_thread_mdelay(2000)");
            while(1){
                err=netcam_fifo_read(&netcam_tx_buff);

                if(err==RT_EOK){
                    //err=netconn_write(newconn,netcam_tx_buff,NETCAM_LINE_SIZE*4,NETCONN_COPY);
                    err=netconn_write(newconn,"test\r\n",6,NETCONN_COPY);
                    if(err==ERR_CLSD||err==ERR_RST){
                        LOG_E("netcam_send_err");
                        //stm32_dcmi_dma_stop();
                        ov2640_deinit();
                        netconn_close(newconn);
                        netconn_delete(newconn);
                        break;
                    }
                    if(err==ERR_OK){
                        LOG_I("netcam_send_ok");
                    }else {
                        LOG_E("netcam_send_err:%d",err);
                    }
                }else{
                    rt_thread_mdelay(10);
                }
            }
        }
        rt_thread_mdelay(200);
    }

}

static rt_thread_t tid = RT_NULL;
int ov2640_port_init(void){
    static ov2640_device_t ov2640_dev = RT_NULL;
    if(ov2640_dev==RT_NULL){
        ov2640_dev=ov2640_hw_init("i2c1", &stm32_ops);
        if (ov2640_dev == RT_NULL)
        {
            LOG_E("ov2640 no found");
            return -RT_ERROR;
        }
    }

    OV2640_RGB565_Mode(ov2640_dev);   //RGB565模式
    OV2640_Light_Mode(ov2640_dev,0);   //自动模式
    OV2640_Color_Saturation(ov2640_dev,3);//色彩饱和度0
    OV2640_Brightness(ov2640_dev,4);   //亮度0
    OV2640_Contrast(ov2640_dev,3);     //对比度0
    OV2640_JPEG_Mode(ov2640_dev);     //JPEG模式
    OV2640_ImageWin_Set(ov2640_dev,0,0,1600,1200);             //全尺寸缩放
    OV2640_OutSize_Set(ov2640_dev,1600,1200);       //设置输出尺寸(1280*800)

//    tid=rt_thread_create("netcam", netcam_thread_entry, RT_NULL, 1500, 25, 5);
//    if(tid!=RT_NULL){
//        rt_thread_startup(tid);
//    }
    stm32_dcmi_dma_start(line_buf,NETCAM_LINE_SIZE);
    return RT_EOK;
}
int ov2640_deinit(void){
    stm32_dcmi_dma_stop();
    rt_thread_mdelay(20);
    rt_free(line_buf);
    for(int i=0;i<NETCAM_FIFO_NUM;i++){
        rt_free(netcam_fifo[i]);
    }
    fifo_rd_pos=0;
    fifo_wr_pos=0;
    return RT_EOK;
}
int netcam_init(void){
    tid=rt_thread_create("netcam", netcam_thread_entry, RT_NULL, 1500, 25, 5);
    if(tid!=RT_NULL){
        rt_thread_startup(tid);
    }
    return RT_EOK;
}

INIT_APP_EXPORT(netcam_init);

