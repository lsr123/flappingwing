#ifndef __STM32F1XX_SD_H__
#define __STM32F1XX_SD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

//#define SD_POLLING_MODE
/**
  * @brief  SD FLASH SDIO Interface
  */
#define SD_DETECT_PIN                    GPIO_Pin_7                 /* PC.7 */
#define SD_DETECT_GPIO_PORT              GPIOC                       /* GPIOC */
#define SD_DETECT_GPIO_CLK               RCC_APB2Periph_GPIOC

#define SDIO_FIFO_Address               ((uint32_t)0x40018080)

#define SDIO_INIT_CLK_DIV                ((uint8_t)0xB2)//b2
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x1)//1

#define SD_PRESENT                        ((uint8_t)0x01)
#define SD_NOT_PRESENT                    ((uint8_t)0x00)

#ifdef __cplusplus
}
#endif

#endif
