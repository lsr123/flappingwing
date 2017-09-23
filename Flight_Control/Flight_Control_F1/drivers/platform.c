#include <rtthread.h>
#include "board.h"
#include <components.h>

#ifdef RT_USING_SPI
#include "stm32f10xx_spi.h"
#include "spi_flash_w25qxx.h"


/*
SPI2_MOSI: PB15
SPI2_MISO: PB14
SPI2_SCK : PB13

CS0: PG10  SPI FLASH
CS1: PB12  TOUCH
CS2: PG7   WIFI
*/
int rt_hw_spi1_init(void)
{
    {
        static struct stm32_spi_bus stm32_spi;
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO ,ENABLE);
#ifdef SPI_USE_DMA
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        stm32_spi_register(SPI1, &stm32_spi, "spi1");
    }
    {   //spi flash cs
        static struct rt_spi_device spi_device;
        static struct stm32_spi_cs  spi_cs;
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE ,ENABLE);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        spi_cs.GPIOx = GPIOE;
        spi_cs.GPIO_Pin = GPIO_Pin_5;
        GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;

        GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);
        GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);

        rt_spi_bus_attach_device(&spi_device, "spi10", "spi1", (void*)&spi_cs);
    }
    /* attach cs */
    {   //touch cs
        static struct rt_spi_device spi_device;
        static struct stm32_spi_cs  spi_cs;
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC ,ENABLE);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        spi_cs.GPIOx = GPIOC;
        spi_cs.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
        GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);
        GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);

        rt_spi_bus_attach_device(&spi_device, "spi11", "spi1", (void*)&spi_cs);
    }

    /* attach cs */
    {   //wifi cs
        static struct rt_spi_device spi_device;
        static struct stm32_spi_cs  spi_cs;
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE ,ENABLE);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        spi_cs.GPIOx = GPIOE;
        spi_cs.GPIO_Pin = GPIO_Pin_2;
        GPIO_InitStructure.GPIO_Pin = spi_cs.GPIO_Pin;
        GPIO_Init(spi_cs.GPIOx, &GPIO_InitStructure);
        GPIO_SetBits(spi_cs.GPIOx, spi_cs.GPIO_Pin);

        rt_spi_bus_attach_device(&spi_device, "spi12", "spi1", (void*)&spi_cs);
    }

    return 0;
}

INIT_BOARD_EXPORT(rt_hw_spi1_init);

#endif /* RT_USING_SPI */


void rt_platform_init(void)
{
#ifdef RT_USING_SPI
#ifdef RT_USING_DFS
    w25qxx_init("flash0", "spi10");
#endif /* RT_USING_DFS */
#endif /* RT_USING_SPI */

#ifdef RT_USING_DFS
    /* initilize sd card */
#ifdef RT_USING_SDIO
    rt_mmcsd_core_init();
    rt_mmcsd_blk_init();
    {
        extern rt_int32_t stm32f1xx_sdio_init(void);
        stm32f1xx_sdio_init();
    }
    rt_thread_delay(RT_TICK_PER_SECOND);
#else
    rt_hw_sdcard_init();
#endif
#endif /* RT_USING_DFS */

    rt_thread_delay(50);
    rt_device_init_all();

}

