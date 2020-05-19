#include <rtthread.h>
#include <ov2640.h>
#include <pcf8574.h>
#include <ILI9341_stm32_port.h>

#define DBG_TAG               "OV2640"

//#define PKG_OV2640_DEBUG

#ifdef PKG_OV2640_DEBUG
#define DBG_LVL               DBG_LOG
#else
#define DBG_LVL               DBG_INFO
#endif
#include <rtdbg.h>

rt_device_t sccb=RT_NULL;
pcf8574_device_t pcf;

DCMI_HandleTypeDef hdcmi;
DMA_HandleTypeDef hdma_dcmi;

void hw_delayms(rt_uint32_t ms){
    for(int i=0;i<ms;i++){
        rt_hw_us_delay(1000);
    }
}

void reset_pin_write(rt_uint8_t value)
{
    rt_pin_write(OV2640_RESET_PIN, value);
}

void pwdn_pin_write(rt_uint8_t value)
{
    pcf8574_pin_write(pcf, 2, value);
}

void write_reg(rt_uint8_t addr,rt_uint8_t val){
    rt_uint8_t data[2]={0};
    data[0]=addr;
    data[1]=val;
    rt_device_write(sccb,OV2640_ADDR_DEFAULT, &data, 2);
}

rt_uint8_t read_reg(rt_uint8_t addr){
    rt_uint8_t val=0;
    rt_device_write(sccb,OV2640_ADDR_DEFAULT, &addr, 1);
    rt_device_read(sccb,OV2640_ADDR_DEFAULT, &val, 1);
    return val;
}

rt_err_t dcmi_dma_init(rt_uint16_t memsize,rt_uint32_t meminc){
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_dcmi.Instance = DMA2_Stream1;
    hdma_dcmi.Init.Channel = DMA_CHANNEL_1;
    hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dcmi.Init.MemInc = meminc;
    hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_dcmi.Init.MemDataAlignment = memsize;
    hdma_dcmi.Init.Mode = DMA_CIRCULAR;
    hdma_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_dcmi.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_dcmi) != HAL_OK)
    {
        return -RT_ERROR;
    }
    __HAL_LINKDMA(&hdcmi,DMA_Handle,hdma_dcmi);

//    HAL_NVIC_SetPriority(DCMI_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(DCMI_IRQn);
    return RT_EOK;
}

rt_err_t dcmi_init(uint32_t JPEGMode){
    hdcmi.Instance = DCMI;
    hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
    hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
    hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
    hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
    hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
    hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
    hdcmi.Init.JPEGMode = JPEGMode;
    hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
    hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
    hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
    hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
    if (HAL_DCMI_Init(&hdcmi) != HAL_OK)
    {
        return -RT_ERROR;
    }
//    __HAL_DCMI_DISABLE_IT(&hdcmi,DCMI_IT_LINE|DCMI_IT_VSYNC|DCMI_IT_ERR|DCMI_IT_OVR);
//    __HAL_DCMI_ENABLE_IT(&hdcmi,DCMI_IT_FRAME);
    return RT_EOK;
}

void DCMI_IRQHandler(void)
{
    HAL_DCMI_IRQHandler(&hdcmi);
}

//void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
//{
////    __HAL_DCMI_ENABLE_IT(&DCMI_Handler,DCMI_IT_FRAME);
//}

//void dcmi_rx_callback(void){
//
//}

void DMA2_Stream1_IRQHandler(void)
{
    if(__HAL_DMA_GET_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5)!=RESET)
    {
        __HAL_DMA_CLEAR_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5);
    }
}

rt_err_t hal_init(void){
    if (dcmi_dma_init(DMA_MDATAALIGN_HALFWORD,DMA_MINC_DISABLE) != RT_EOK)
    {
        LOG_E("dcmi_dma init fail!");
        return -RT_ERROR;
    }
    if (dcmi_init(DCMI_JPEG_DISABLE) != RT_EOK)
    {
        LOG_E("dcmi init failed");
        return -RT_ERROR;
    }
    sccb=rt_device_find("i2c1");
    if(sccb == RT_NULL){
        LOG_E("sccb device not found!");
        return -RT_EEMPTY;
    }
    if (rt_device_open(sccb, RT_NULL) != RT_EOK)
    {
        LOG_E("sccb device opened failed!");
        return -RT_ERROR;
    }

    pcf = pcf8574_init("i2c2", 0x20);
    if (pcf == RT_NULL)
    {
        LOG_E("PCF8574 not found!");
        return -RT_ERROR;
    }

    rt_pin_mode(OV2640_RESET_PIN, PIN_MODE_OUTPUT);
    return RT_EOK;
}

static struct cam_ops stm32_ops={
    .hal_init = hal_init,
    .write_reg = write_reg,
    .read_reg = read_reg,
    .reset_pin_write = reset_pin_write,
    .pwdn_pin_write = pwdn_pin_write,
    .hw_delayms = hw_delayms
};

int cam_dev_init(void){
    return cam_hw_init(&stm32_ops);
}
INIT_DEVICE_EXPORT(cam_dev_init);

int cam_test_init(void){
    rt_uint16_t cam_para[4];
    rt_device_t cam=rt_device_find("cam");
    rt_uint8_t bk_en=0;
    rt_uint32_t color=0;
    rt_device_t lcd=rt_device_find("lcd");
    if(cam == RT_NULL){
        LOG_E("cam device not found!");
        return -RT_EEMPTY;
    }
    if (rt_device_open(cam, RT_NULL) != RT_EOK)
    {
        LOG_E("cam device opened failed!");
        return -RT_ERROR;
    }
    rt_device_control(cam, CAM_RGB565_Mode, RT_NULL);
    cam_para[0]=0;
    rt_device_control(cam, CAM_Light_Mode, cam_para);
    cam_para[0]=3;
    rt_device_control(cam, CAM_Color_Saturation, cam_para);
    cam_para[0]=4;
    rt_device_control(cam, CAM_Brightness, cam_para);
    cam_para[0]=3;
    rt_device_control(cam, CAM_Contrast, cam_para);
    cam_para[0]=0;
    cam_para[1]=0;
    cam_para[2]=1200;
    cam_para[3]=1200;
    rt_device_control(cam, CAM_ImageWin_Set, cam_para);
    cam_para[0]=240;
    cam_para[1]=240;
    rt_device_control(cam, CAM_OutSize_Set, cam_para);

    if(lcd == RT_NULL){
        LOG_E("lcd device not found!");
        return -RT_EEMPTY;
    }
    if (rt_device_open(lcd, RT_NULL) != RT_EOK)
    {
        LOG_E("lcd device opened failed!");
        return -RT_ERROR;
    }
    bk_en=1;
    rt_device_control(lcd, LCD_CTRL_SET_BK, &bk_en);
    color=RED;
    rt_device_control(lcd, LCD_CTRL_CLEAR, &color);

    rt_device_control(lcd, LCD_CTRL_SET_CURSOR, RT_NULL);
    rt_device_control(lcd, LCD_CTRL_START_GRAM, RT_NULL);
    HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (rt_uint32_t)&LCD->LCD_RAM, 1);
    return RT_EOK;
}
INIT_APP_EXPORT(cam_test_init);
