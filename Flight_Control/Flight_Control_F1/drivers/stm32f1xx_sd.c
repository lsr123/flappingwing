/*
 * File      : stm32f1xx_sd.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author        Notes
 * 2012-08-18     weety         first version
 * 2013-04-27     jiezhi320     modified for stm32f1xx
 */
#include <rtthread.h>
#include <stm32f10x.h>
#include <rthw.h>
#include <drivers/mmcsd_core.h>
#include <drivers/sdio.h>
#include "stm32f1xx_sd.h"

#define REQ_ST_INIT (1U << 0)
#define REQ_ST_CMD  (1U << 1)
#define REQ_ST_STOP (1U << 2)

//#define RT_STM32F1XX_SDIO_DBG

#ifdef RT_STM32F1XX_SDIO_DBG
#define sdio_dbg(fmt, ...)  rt_kprintf(fmt, ##__VA_ARGS__)
#else
#define sdio_dbg(fmt, ...)
#endif

#define STM32F1XX_SDIO_ERRORS (SDIO_IT_CCRCFAIL | SDIO_IT_CTIMEOUT | \
        SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_RXOVERR | \
        SDIO_IT_TXUNDERR | SDIO_IT_STBITERR)

#define SDIO_CMD0TIMEOUT                ((uint32_t)0x00002710)
#define SD_DATATIMEOUT                  ((uint32_t)0xFFFFFFFF)

#define SDIOCLK     36000000//stm32f1µÄsdioÊ±ÖÓÎªhclk/2

struct stm32f1xx_sdio
{
    struct rt_mmcsd_host *host;
    struct rt_mmcsd_req *req;
    struct rt_mmcsd_cmd *cmd;
    rt_uint32_t current_status;
};

static struct stm32f1xx_sdio *stm32f1_sdio;

SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;


ALIGN(8)
uint8_t rx_buf[4096];

static void stm32f1xx_sdio_process_next(struct stm32f1xx_sdio *sdio);


/**
  * @brief  DeInitializes the SDIO interface.
  * @param  None
  * @retval None
  */
void SD_LowLevel_DeInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /*!< Disable SDIO Clock */
    SDIO_ClockCmd(DISABLE);

    /*!< Set Power State to OFF */
    SDIO_SetPowerState(SDIO_PowerState_OFF);

    /*!< DeInitializes the SDIO peripheral */
    SDIO_DeInit();
    /* Disable the SDIO  Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, DISABLE);

    /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PD.02 CMD line */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* Configure PC.12 pin: CLK pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for
  *         data transfer).
  * @param  None
  * @retval None
  */
void SD_LowLevel_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIOC and GPIOD Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |SD_DETECT_GPIO_CLK, ENABLE);

    /* Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PD.02 CMD line */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
    GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

    /* Enable the SDIO AHB Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, ENABLE);

    /* Enable the DMA2 Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    SDIO_ClockCmd(ENABLE);
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Tx request.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

    /* DMA2 Channel4 disable */
    DMA_Cmd(DMA2_Channel4, DISABLE);

    /* DMA2 Channel4 Config */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferSRC;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel4, &DMA_InitStructure);

    /* DMA2 Channel4 enable */
    DMA_Cmd(DMA2_Channel4, ENABLE);

}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

    /* DMA2 Channel4 disable */
    DMA_Cmd(DMA2_Channel4, DISABLE);

    /* DMA2 Channel4 Config */
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel4, &DMA_InitStructure);

    /* DMA2 Channel4 enable */
    DMA_Cmd(DMA2_Channel4, ENABLE);
}


/*
 * Handle a command that has been completed
 */
static void stm32f1xx_sdio_completed_command(struct stm32f1xx_sdio *sdio, rt_uint32_t status)
{
    struct rt_mmcsd_cmd *cmd = sdio->cmd;
    struct rt_mmcsd_data *data = cmd->data;

    cmd->resp[0] = SDIO_GetResponse(SDIO_RESP1);
    cmd->resp[1] = SDIO_GetResponse(SDIO_RESP2);
    cmd->resp[2] = SDIO_GetResponse(SDIO_RESP3);
    cmd->resp[3] = SDIO_GetResponse(SDIO_RESP4);

    sdio_dbg("Status = %08X/%08x [%08X %08X %08X %08X]\n",
             status, SDIO->MASK,
             cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);

    if (status & STM32F1XX_SDIO_ERRORS)
    {
        if ((status & SDIO_IT_CCRCFAIL) && (resp_type(cmd) & (RESP_R3 | RESP_R4)))
        {
            cmd->err = 0;
        }
        else
        {
            if (status & (SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT |
                          SDIO_IT_RXOVERR | SDIO_IT_TXUNDERR))
            {
                if (data)
                {
                    if (status & SDIO_IT_DTIMEOUT)
                        data->err = -RT_ETIMEOUT;
                    else if (status & SDIO_IT_DCRCFAIL)
                        data->err = -RT_ERROR;
                }
            }
            else
            {
                if (status & SDIO_IT_CTIMEOUT)
                    cmd->err = -RT_ETIMEOUT;
                else if (status & SDIO_IT_CCRCFAIL)
                    cmd->err = -RT_ERROR;
                else
                    cmd->err = -RT_ERROR;
            }

            sdio_dbg("error detected and set to %d/%d (cmd = %d),0x%08x\n",
                     cmd->err, data ? data->err : 0,
                     cmd->cmd_code, status);
        }
    }
    else
        cmd->err = 0;

    stm32f1xx_sdio_process_next(sdio);
}


/**
  * @brief  Allows to process all the interrupts that are high.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
void SD_ProcessIRQSrc(void)
{
    int complete = 0;
    rt_uint32_t intstatus = SDIO->STA;

    if (intstatus & STM32F1XX_SDIO_ERRORS)
    {
        SDIO_ClearITPendingBit(STM32F1XX_SDIO_ERRORS);
        complete = 1;
    }
    else
    {
        if (SDIO_GetITStatus(SDIO_IT_CMDREND) != RESET)
        {
            SDIO_ClearITPendingBit(SDIO_IT_CMDREND);
            if (!stm32f1_sdio->cmd->data)
                complete = 1;
        }

        if (SDIO_GetITStatus(SDIO_IT_CMDSENT) != RESET)
        {
            SDIO_ClearITPendingBit(SDIO_IT_CMDSENT);
            complete = 1;
        }

        if (SDIO_GetITStatus(SDIO_IT_DATAEND) != RESET)
        {
            SDIO_ClearITPendingBit(SDIO_IT_DATAEND);
#if 0
            if( (stm32f1_sdio->cmd->data->flags & DATA_DIR_READ) &&
                    ((uint32_t)stm32f1_sdio->cmd->data->buf & 0x03) )
            {
                rt_memcpy(stm32f1_sdio->cmd->data->buf, rx_buf,
                          stm32f1_sdio->cmd->data->blksize * stm32f1_sdio->cmd->data->blks);
            }
#else
            if (stm32f1_sdio->cmd->data->flags & DATA_DIR_READ)
            {
                rt_memcpy(stm32f1_sdio->cmd->data->buf, rx_buf,
                          stm32f1_sdio->cmd->data->blksize * stm32f1_sdio->cmd->data->blks);
            }
#endif
            complete = 1;
        }

        if (SDIO_GetITStatus(SDIO_IT_SDIOIT) != RESET)
        {
            SDIO_ClearITPendingBit(SDIO_IT_SDIOIT);
            sdio_irq_wakeup(stm32f1_sdio->host);
        }
    }
    if (complete)
    {
        /*SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
                  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
                  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE); */

        SDIO_ITConfig(STM32F1XX_SDIO_ERRORS, DISABLE);
        stm32f1xx_sdio_completed_command(stm32f1_sdio, intstatus);
    }

}

rt_uint32_t get_order(rt_uint32_t data)
{
    rt_uint32_t order = 0;
    while (data)
    {
        data >>= 1;
        order++;
    }

    return order - 1;
}

static void stm32f1xx_sdio_send_command(struct stm32f1xx_sdio *sdio, struct rt_mmcsd_cmd *cmd)
{
    struct rt_mmcsd_data *data = cmd->data;

    sdio->cmd = cmd;

    if (resp_type(cmd) == RESP_NONE)
        SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;
    else
    {
        /* set 136 bit response for R2, 48 bit response otherwise */
        if (resp_type(cmd) == RESP_R2)
            SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
        else
            SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    }

    SDIO_CmdInitStructure.SDIO_Argument = cmd->arg;
    SDIO_CmdInitStructure.SDIO_CmdIndex = cmd->cmd_code;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;

    if (cmd->cmd_code == GO_IDLE_STATE)
        SDIO_ITConfig(SDIO_IT_CMDSENT, ENABLE);
    else
        SDIO_ITConfig(SDIO_IT_CCRCFAIL | SDIO_IT_CMDREND | SDIO_IT_CTIMEOUT, ENABLE);
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    if (data)
    {
        SDIO->DCTRL = 0x0;
        SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
        SDIO_DataInitStructure.SDIO_DataLength = data->blksize * data->blks;
        SDIO_DataInitStructure.SDIO_DataBlockSize = get_order(data->blksize) << 4;
        if (data->flags & DATA_DIR_WRITE)
            SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
        else if (data->flags & DATA_DIR_READ)
            SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
        SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
        SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
        SDIO_DataConfig(&SDIO_DataInitStructure);

        if (data->flags & DATA_DIR_WRITE)
        {
            //rt_kprintf("SDIO W 0x%08X %u\r\n", (uint32_t)data->buf, data->blksize * data->blks);
            rt_memcpy(rx_buf, data->buf, data->blksize * data->blks);
            SD_LowLevel_DMA_TxConfig((uint32_t *)rx_buf, data->blksize * data->blks);
            // SD_LowLevel_DMA_TxConfig((uint32_t *)data->buf, data->blksize * data->blks);
        }
        else if (data->flags & DATA_DIR_READ)
        {
            SD_LowLevel_DMA_RxConfig((uint32_t *)rx_buf, data->blksize * data->blks);

#if 0
            //rt_kprintf("SDIO R 0x%08X %u\r\n", (uint32_t)data->buf, data->blksize * data->blks);
            if((uint32_t)data->buf & 0x03)
            {
                //rt_kprintf("bad addr %08X\r\n", data->buf);
                SD_LowLevel_DMA_RxConfig((uint32_t *)rx_buf, data->blksize * data->blks);
            }
            else
            {
                SD_LowLevel_DMA_RxConfig((uint32_t *)data->buf, data->blksize * data->blks);
            }
#endif
        }
        SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
        SDIO_DMACmd(ENABLE);
    }
}
/*
 * Process the next step in the request
 */
static void stm32f1xx_sdio_process_next(struct stm32f1xx_sdio *sdio)
{
    if (sdio->current_status == REQ_ST_INIT)
    {
        sdio->current_status = REQ_ST_CMD;
        stm32f1xx_sdio_send_command(sdio, sdio->req->cmd);
    }
    else if ((sdio->current_status == REQ_ST_CMD) && sdio->req->stop)
    {
        sdio->current_status = REQ_ST_STOP;
        stm32f1xx_sdio_send_command(sdio, sdio->req->stop);
    }
    else
    {
        mmcsd_req_complete(sdio->host);
    }
}


static void stm32f1xx_sdio_request(struct rt_mmcsd_host *host, struct rt_mmcsd_req *req)
{
    struct stm32f1xx_sdio *sdio = host->private_data;
    sdio->req = req;
    sdio->current_status = REQ_ST_INIT;

    stm32f1xx_sdio_process_next(sdio);
}


/*
 * Set the IOCFG
 */
static void stm32f1xx_sdio_set_iocfg(struct rt_mmcsd_host *host, struct rt_mmcsd_io_cfg *io_cfg)
{
    rt_uint32_t clkdiv;

    if (io_cfg->clock == 0)
    {
        SDIO_ClockCmd(DISABLE);
        clkdiv = 0;
    }
    else
    {
        clkdiv = SDIOCLK / io_cfg->clock;
        if (clkdiv < 2)
        {
            clkdiv = SDIO_TRANSFER_CLK_DIV;
        }
        else
        {
            clkdiv -= 2;
        }
        SDIO_InitStructure.SDIO_ClockDiv = clkdiv;

        SDIO_Init(&SDIO_InitStructure);
        SDIO_ClockCmd(ENABLE);
    }
    if (io_cfg->bus_width == MMCSD_BUS_WIDTH_4)
    {
        sdio_dbg("MMC: Setting controller bus width to 4\n");
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;
    }
    else
    {
        sdio_dbg("MMC: Setting controller bus width to 1\n");
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
    }

    /* maybe switch power to the card */
    switch (io_cfg->power_mode)
    {
    case MMCSD_POWER_OFF:
        SDIO_SetPowerState(SDIO_PowerState_OFF);
        break;
    case MMCSD_POWER_UP:
        break;
    case MMCSD_POWER_ON:
        SDIO_SetPowerState(SDIO_PowerState_ON);
        break;
    default:
        rt_kprintf("unknown power_mode %d\n", io_cfg->power_mode);
        break;
    }

    SDIO_Init(&SDIO_InitStructure);

}


static void stm32f1xx_sdio_enable_sdio_irq(struct rt_mmcsd_host *host, rt_int32_t enable)
{
    if (enable)
        SDIO_ITConfig(SDIO_IT_SDIOIT, ENABLE);
    else
        SDIO_ITConfig(SDIO_IT_SDIOIT, DISABLE);
}

/**
 * @brief  Detect if SD card is correctly plugged in the memory slot.
 * @param  None
 * @retval Return if SD is detected or not
 */
rt_int32_t stm32f1xx_SD_Detect(struct rt_mmcsd_host *host)
{
    __IO rt_int32_t status = SD_PRESENT;

    /*!< Check GPIO to detect SD */
    if (GPIO_ReadInputDataBit(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != Bit_RESET)
    {
        status = SD_NOT_PRESENT;
    }
    return status;
}


static const struct rt_mmcsd_host_ops ops =
{
    stm32f1xx_sdio_request,
    stm32f1xx_sdio_set_iocfg,
    stm32f1xx_SD_Detect,
    stm32f1xx_sdio_enable_sdio_irq,
};


rt_int32_t stm32f1xx_sdio_init(void)
{
    struct rt_mmcsd_host *host;
    NVIC_InitTypeDef NVIC_InitStructure;

    host = mmcsd_alloc_host();
    if (!host)
    {
        return -RT_ERROR;
    }

    stm32f1_sdio = rt_malloc(sizeof(struct stm32f1xx_sdio));
    if (!stm32f1_sdio)
    {
        rt_kprintf("alloc stm32f1_sdio failed\n");
        goto err;
    }

    rt_memset(stm32f1_sdio, 0, sizeof(struct stm32f1xx_sdio));

    host->ops = &ops;
    host->freq_min = 375000;
    host->freq_max = 25000000;
    host->valid_ocr = VDD_31_32|VDD_32_33 | VDD_33_34;
    host->flags = MMCSD_BUSWIDTH_4 | MMCSD_MUTBLKWRITE;
    host->max_seg_size = 65535;
    host->max_dma_segs = 2;
    host->max_blk_size = 512;
    host->max_blk_count = 4096;

    stm32f1_sdio->host = host;

    SDIO_StructInit(&SDIO_InitStructure);
    /* SDIO Peripheral Low Level Init */
    SD_LowLevel_Init();

    NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    host->private_data = stm32f1_sdio;

    mmcsd_change(host);

    return 0;

err:
    mmcsd_free_host(host);

    return -RT_ENOMEM;
}


/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* Process All SDIO Interrupt Sources */
    SD_ProcessIRQSrc();

    /* leave interrupt */
    rt_interrupt_leave();
}
