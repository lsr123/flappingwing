/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 * 2013-07-12     aozima       update for auto initial.
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <board.h>
#include <rtthread.h>

#ifdef  RT_USING_COMPONENTS_INIT
#include <components.h>
#endif  /* RT_USING_COMPONENTS_INIT */

#ifdef RT_USING_DFS
/* dfs filesystem:ELM filesystem init */
#include <dfs_elm.h>
/* dfs Filesystem APIs */
#include <dfs_fs.h>
#endif

#ifdef RT_USING_RTGUI
#include <rtgui/rtgui.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
#include <rtgui/driver.h>
#include <rtgui/calibration.h>
#endif

#include "led.h"
#include "I2C_soft.h"

ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t led_stack[ 512 ];
static struct rt_thread led_thread;
static rt_uint8_t mpu6050_stack[512];
static struct rt_thread mpu6050_thread;

static void led_thread_entry(void* parameter)
{
    unsigned int count=0;

    rt_hw_led_init();
	
		
    while (1)
    {
			  rt_kprintf("aaa");
        /* led1 on */
#ifndef RT_USING_FINSH
        rt_kprintf("led on, count : %d\r\n",count);
		
#endif
        count++;
        rt_hw_led_on(0);
        rt_thread_delay( RT_TICK_PER_SECOND/2 ); /* sleep 0.5 second and switch to other thread */

        /* led1 off */
#ifndef RT_USING_FINSH
        rt_kprintf("led off\r\n");
#endif
        rt_hw_led_off(0);
        rt_thread_delay( RT_TICK_PER_SECOND/2 );
    }
}

static void mpu6050_thread_entry(void* parameter)
{
    unsigned int count=0;

   // rt_hw_mpu6050_init();   //在此添加mpu6050的初始化函数
	
		
    while (1)
    {
			  //rt_kprintf("aaa");
#ifndef RT_USING_FINSH
        rt_kprintf("mpu6050, count : %d\r\n",count);
#endif
        count++;
        
				//在此添加mpu6050相关的操作函数
			
       
    }
}

void rt_init_thread_entry(void* parameter)
{
		
	  //I2C_soft_Init();
	
    {
        extern void rt_platform_init(void);
        rt_platform_init();
    }

    /* initialization RT-Thread Components */
    rt_components_init();
}

int rt_application_init(void)
{
		rt_thread_t init_thread;
		rt_err_t mpu6050_result;
		rt_err_t led_result;
	
	  /*系统线程*/
	  init_thread = rt_thread_create("init",
                                   rt_init_thread_entry, RT_NULL,
                                   2048, RT_THREAD_PRIORITY_MAX/3, 20);

    if (init_thread != RT_NULL)
        rt_thread_startup(init_thread);

    /* led 线程 */
	  
    led_result = rt_thread_init(&led_thread,                  
                            "led",
                            led_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&led_stack[0],
                            sizeof(led_stack),
                            20,
                            5);
    if (led_result == RT_EOK)
    {
        rt_thread_startup(&led_thread);
    }
		
		/*MPU6050线程*/
    mpu6050_result = rt_thread_init(&mpu6050_thread,                  
                            "mpu6050",
                            mpu6050_thread_entry,
                            RT_NULL,
                            (rt_uint8_t*)&mpu6050_stack[0],
                            sizeof(mpu6050_stack),
                            20,
                            5);
    if (mpu6050_result == RT_EOK)
    {
        rt_thread_startup(&mpu6050_thread);
    }
		

    

    return 0;
}

/*@}*/
