/*
 * The Clear BSD License
 * Copyright 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_gpio.h"

#include "pin_mux.h"
#include "fsl_debug_console.h" //この行を足します
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_LED_PORT 0U
#define BOARD_LED_PIN_GREEN  9U //グリーンLED
#define BOARD_LED_PIN_RED  17U //赤LED
#define BOARD_LED_PIN_BLUE  8U //青LED
#define UserButton 12U//ISPボタン

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_systickCounter;
char ch; //この行を付け加えます

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    { 
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while(g_systickCounter != 0U)
    {
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
	uint32_t pState=0;


    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };

    /* Board pin init */
    /* Attach 12 MHz clock to USART0 (debug console) */
    CLOCK_Select(kUART0_Clk_From_MainClk);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Init output LED GPIO. */
    GPIO_PortInit(GPIO, BOARD_LED_PORT);
    GPIO_PinInit(GPIO, BOARD_LED_PORT, BOARD_LED_PIN_GREEN, &led_config);
    GPIO_PinInit(GPIO, BOARD_LED_PORT, BOARD_LED_PIN_RED, &led_config);
    GPIO_PinInit(GPIO, BOARD_LED_PORT, BOARD_LED_PIN_BLUE, &led_config);

    /* Set systick reload value to generate 1ms interrupt */
    if(SysTick_Config(SystemCoreClock / 1000U))
    {
        while(1)
        {
        }
    }
    
    while (1)
    {
        /* Delay 1000 ms */
        //SysTick_DelayTicks(1000U);
        //GPIO_PortToggle(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN);

    	//pState = GPIO_PortRead(GPIO, 0);
    	    	      //  if (!(pState & (1<<UserButton)))
    	    	      //  {
    	    	      //      GPIO_PortSet(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_RED); //赤色LED ISPボタンを押している間オン
    	    	      //  }else{
    	    	      //  	GPIO_PortClear(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_RED); //ISPボタンを話したら赤色LEDをオフ
    	    	      //  }*/
    			ch = GETCHAR();

    		    switch (ch){
    				 case 'r':
    					 GPIO_SetPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_RED);
    					 GPIO_SetPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_BLUE);
    					 GPIO_SetPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_GREEN);
    					 break;
    				 case 'b':
    					 GPIO_ClearPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_BLUE);
    					 GPIO_ClearPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_RED);
    					 GPIO_SetPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_GREEN);
    					 break;
    				 case 'g':
    					 GPIO_ClearPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_GREEN);
    					 GPIO_ClearPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_RED);
    					 GPIO_SetPinsOutput(GPIO, BOARD_LED_PORT, 1u << BOARD_LED_PIN_BLUE);
    					 break;
    				 default:
    					 break;
    		    }
    }
}
