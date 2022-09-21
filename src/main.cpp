/**
  ******************************************************************************
  * @file    LCD_Example/src/main.c
  * @author  Milandr Application Team
  * @version V1.0.1
  * @date    21/12/2020
  * @brief   Main program body.
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, MILANDR SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  */

/* Includes ------------------------------------------------------------------*/
//#include <intrinsics.h>
#include <stdio.h>
//#include <LowLevelIOInterface.h>
//#include "csr.h"
#include "MLDR187_lib.h"
#include "MLDR187.h"
#include "MLDR187_gpio.h"
#include "MLDR187_rst_clk.h"
#include "MLDR187_eeprom.h"
#include "MLDR187_bkp.h"
#include "MLDR187_uart.h"
#include <MLDR187_adcui.h>

#include "interrupt.h"
#include "lcd.h"
#include "text.h"
#include "Logo.h"
#include "init.h"
#include "test.h"
#include "test2.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define	LED_PORT 		MDR_PORTD
#define	LED_PORT_CLK 	RST_CLK_PORTD
#define LED_PIN_0		PORT_Pin_0
#define LED_PIN_1		PORT_Pin_1
#define LED_PIN_2		PORT_Pin_2
#define LED_PIN_3		PORT_Pin_3

#define	KEY_PORT 		MDR_PORTA
#define	KEY_PORT_CLK 	RST_CLK_PORTA
#define KEY_PIN_0		PORT_Pin_0
#define KEY_PIN_1		PORT_Pin_1
#define KEY_PIN_2		PORT_Pin_2
#define KEY_PIN_3		PORT_Pin_3

#define UART_MODULE     MDR_UART1
#define UART_PORT       MDR_PORTB
#define UART_PORT_CLK   RST_CLK_PORTB
#define UART_TX_PIN     PORT_Pin_0
#define UART_RX_PIN     PORT_Pin_1

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
PORT_InitTypeDef        PORT_InitStructure;
UART_InitTypeDef        UART_InitStructure;
RST_CLK_CpuSpeed_InitTypeDef    RST_CLK_CpuSpeed_InitStructure;

uint32_t DelayCnt = 0;
uint8_t TickTock = 0;

char Logotip1[]	= 	{
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x20,0xE0,0xF0,0xF0,0xF0,0xF8,0xF8,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0x00,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xFC,0xF0,0xF0,0xF0,0xF0,0xE0,0xE0,0xC0,0xE0,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0xE0,0xF8,0xFC,0xFC,0xFE,0xFF,0xFF,0xFE,0xF8,0xF0,0xC0,0x03,0x07,0x1F,0x3F,0xFF,0xFF,0xFF,0x7F,0xFF,0x7F,0x3F,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x9F,0x9F,0xFF,0xFF,0xFF,0xFF,0xBF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFE,0xBC,0xD8,0xE0,0xE0,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x80,0xC0,0xF0,0xF8,0xF8,0xF0,0xE1,0xE3,0xE7,0xE3,0x87,0x8F,0x0F,0x3F,0x5F,0x3F,0x1F,0x07,0x03,0x02,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x3F,0x9F,0xDF,0xE7,0xF3,0xFB,0xFD,0xFE,0x7F,0xFF,0x7F,0x3F,0x7F,0x7F,0x7F,0x7E,0x78,0x60,0x00,0x00,0x00,0x00,0x00,
		0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0xAD,0xFD,0xDD,0x9D,0x9D,0x9D,0x9D,0x9F,0x9E,0x9E,0x9F,0x9F,0x8F,0xCF,0xD7,0xFB,0xF7,0xFC,0xF9,0xFD,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xC1,0x00,0x00,0x00,
		0x7E,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x77,0x77,0x73,0x7B,0x77,0x73,0x73,0xF3,0xF3,0xF7,0xFB,0xE7,0xF7,0xEF,0x5F,0xBF,0xBF,0x7F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0x3F,0xBF,0xBF,0x3F,0xE1,0x00,0x00,0x00,
		0x00,0x01,0x07,0x0F,0x07,0x9F,0xCF,0xC7,0xE5,0xF3,0xF3,0xF8,0xF8,0xFC,0xFF,0xFF,0xF8,0xF0,0x60,0x20,0x10,0x00,0x80,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x7F,0x7F,0xFF,0xFE,0xFD,0xF9,0xFF,0xE7,0xCF,0xBE,0x7E,0x7E,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0xFE,0x3E,0x5E,0x9F,0xCD,0xE7,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x02,0x03,0x0F,0x0F,0x3F,0x7F,0xFF,0x7F,0x3F,0x1F,0x0F,0x03,0x81,0xE0,0xF0,0xFC,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFF,0x0E,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE,0xFE,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x7E,0xBD,0xBB,0xD7,0xEF,0x07,0x0B,0x09,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x0F,0x0F,0x0F,0x07,0x07,0x07,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,0x07,0x07,0x07,0x07,0x0F,0x07,0x0D,0x02,0x0E,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};

/* Private function prototypes -----------------------------------------------*/
void Setup_LED_Port(void);
void Setup_CPU_Clock(void);
void Setup_UART(void);
void Setup_Interrupts(void);
void Setup_Key(void);

/* Private functions ---------------------------------------------------------*/

/* --- */
void SysTick_Handler(void)
{
    // Set up time for next interrupt 
    MDR_CLIC_MTIMECMP_Reg = MDR_CLIC_MTIME_Reg + 32768;
    
    if (TickTock)
    {
        TickTock = 0;
    }
    else
    {
        TickTock = 1;
    }

}


#define SHIFT 13

#define N 512
int32_t buf[N][2];
uint32_t buf2[N][4];

char *ptr;

/* Main Function */
int main()
{

    init_clock();
    init_uart();
    init_ADC();
    ptr = TestPrint();
    ptr = TestPrint2();

    for (int i = 0; i < N; i++) {
        while (ADCUI_GetFlag(adcuiCh0, adcuiFlagVoltageFifoEmpty));
        while (ADCUI_GetFlag(adcuiCh0, adcuiFlagCurrentFifoEmpty));
        buf[i][0] = (int32_t)ADCUI_GetNextVoltageFifoValue(adcuiCh0) << 8 >> 8;
        buf[i][1] = (int32_t)ADCUI_GetNextCurrentFifoValue(adcuiCurCh0) << 8 >> 8;
        buf2[i][0] = ADCUI_GetEnergyAccumulator(adcuiCh0, adcuiActivePositiveEnergy) >> SHIFT;
        buf2[i][1] = ADCUI_GetEnergyAccumulator(adcuiCh0, adcuiActiveNegativeEnergy) >> SHIFT;
        buf2[i][2] = ADCUI_GetEnergyAccumulator(adcuiCh0, adcuiReactivePositiveEnergy) >> SHIFT;
        buf2[i][3] = ADCUI_GetEnergyAccumulator(adcuiCh0, adcuiReactiveNegativeEnergy) >> SHIFT;
    }

//    for (int i = 0; i < N; i++)
//        // printf("%ld\t%ld\t%lu\t%lu\t%lu\t%lu\n",
//               buf[i][0], buf[i][1],
//               buf2[i][0], buf2[i][1],
//               buf2[i][2], buf2[i][3]
//        );

    // printf("K1986BK025 Init end!\n");

    while (1)
    {
        LCD_CurrentMethod = MET_AND;

        /* Key down*/
        if (PORT_ReadPin(KEY_PORT, KEY_PIN_0))
            LCD_PUTC(24, 52, 0);
        else
            LCD_PUTC(24, 52, 0x19);

        /* Key left*/
        if (PORT_ReadPin(KEY_PORT, KEY_PIN_1))
            LCD_PUTC(16, 44, 0);
        else
            LCD_PUTC(16, 44, 0x1B);

        /* Key right*/
        if (PORT_ReadPin(KEY_PORT, KEY_PIN_2))
            LCD_PUTC(32, 44, 0);
        else
            LCD_PUTC(32, 44, 0x1A);

        /* Key up*/
        if (PORT_ReadPin(KEY_PORT, KEY_PIN_3))
            LCD_PUTC(24, 37, 0);
        else
            LCD_PUTC(24, 37, 0x18);

        /* 1 second */
        if (TickTock)
        {
            LCD_PUTC(80, 52, 0x5C);
            PORT_SetReset(LED_PORT, LED_PIN_3, RESET);
        }
        else
        {
            LCD_PUTC(80, 52, 0x2F);
            PORT_SetReset(LED_PORT, LED_PIN_3, SET);
        }

        /* Program delay */
        if (DelayCnt++ >= 0x00000100)
        {
            DelayCnt = 0;
            if (PORT_ReadPin(LED_PORT, LED_PIN_0) != 0)
                PORT_SetReset(LED_PORT, LED_PIN_0, RESET);
            else
                PORT_SetReset(LED_PORT, LED_PIN_0, SET);
        }
    }
}

/* --- */
void Setup_Interrupts(void)
{
    /* Disable SysTick interrupt */
    disable_SysTick_Handler();
    /* Safely set up SysTick cmp value */
    MDR_CLIC_MSIP_Reg = 0;
    MDR_CLIC_MTIMECMP_Reg = MDR_CLIC_MTIMECMP_Reg + 32768;
    /* Enable SysTick interrupt */
    enable_SysTick_Handler();
    RST_CLK_EnablePeripheralClock(RST_CLK_BKP, RST_CLK_Div1);
    MDR_BKP->WPR = 0x8555AAA1;
    MDR_BKP->CLK |= 0x00000003;

}

/* Frequencies setup */
void Setup_CPU_Clock(void)
{
    MDR_RST_CLK->CPU_CLOCK &= RST_CLK_CPU_CLOCK_HCLKSEL_Msk;    // HCLK = HSI

    RST_CLK_EnablePeripheralClock(RST_CLK_EEPROM, RST_CLK_Div1);
    EEPROM_SetLatency(flashCoreSpeedUpTo60MHz);
    
    RST_CLK_CpuSpeed_InitStructure.hseState = RST_CLK_HseOscillator;
    RST_CLK_CpuSpeed_InitStructure.hseSpeed = 8000000;
    RST_CLK_CpuSpeed_InitStructure.cpuC1Src = RST_CLK_CpuC1SelHse;
    RST_CLK_CpuSpeed_InitStructure.pllState = RST_CLK_PllFromHse;
    RST_CLK_CpuSpeed_InitStructure.pllMult = 6;
    RST_CLK_CpuSpeed_InitStructure.cpuC2Src = RST_CLK_CpuC2SelPllCpu;
    RST_CLK_CpuSpeed_InitStructure.cpuC3Div = RST_CLK_Div1;
    RST_CLK_CpuSpeed_InitStructure.hclkSrc = RST_CLK_HclkSelCpuC3;
    RST_CLK_SetupCpuSpeed(&RST_CLK_CpuSpeed_InitStructure);
    RST_CLK_SetupHsPeripheralClock(RST_CLK_Clk_PER1_C2, RST_CLK_ClkSrc_PLLCPU);
}

/* --- */
void Setup_UART(void)
{
    PORT_InitStructure.PORT_OE    = PORT_OE_IN;
    PORT_InitStructure.PORT_FUNC  = PORT_FUNC_MAIN;
    PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
    PORT_InitStructure.PORT_SPEED = PORT_SPEED_FAST_2mA;
    PORT_InitStructure.PORT_PULL_DOWN   = PORT_PULL_DOWN_OFF;
    PORT_Init(UART_PORT, (UART_RX_PIN), &PORT_InitStructure);
    PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
    PORT_Init(UART_PORT, (UART_TX_PIN), &PORT_InitStructure);
    
    UART_StructInitDefault(&UART_InitStructure);
    RST_CLK_EnablePeripheralClock(UART_PORT_CLK, RST_CLK_Div1);
    UART_DeInit(UART_MODULE);

    UART_InitStructure.clkDivisor = RST_CLK_Div1;
    UART_InitStructure.baudRate = 115200;
    UART_InitStructure.wordLength = UART_WordLength8b;
    UART_InitStructure.stopBits = UART_StopBits1;
    UART_InitStructure.parity = UART_ParityNone;
    UART_InitStructure.fifoEn = ENABLE;
    UART_Init(UART_MODULE, &UART_InitStructure);
    UART_Cmd(UART_MODULE, ENABLE);
}

/* --- */
void Setup_Key(void)
{
    PORT_InitStructure.PORT_OE    = PORT_OE_IN;
    PORT_InitStructure.PORT_FUNC  = PORT_FUNC_PORT;
    PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
    PORT_InitStructure.PORT_SPEED = PORT_SPEED_SLOW_4mA;
    PORT_InitStructure.PORT_PULL_DOWN   = PORT_PULL_DOWN_OFF;
    PORT_Init(KEY_PORT, (KEY_PIN_0 | KEY_PIN_1 | KEY_PIN_2 | KEY_PIN_3), &PORT_InitStructure);
}

/* --- */
void Setup_LED_Port(void)
{
    PORT_InitStructure.PORT_OE    = PORT_OE_OUT;
    PORT_InitStructure.PORT_FUNC  = PORT_FUNC_PORT;
    PORT_InitStructure.PORT_MODE  = PORT_MODE_DIGITAL;
    PORT_InitStructure.PORT_SPEED = PORT_SPEED_SLOW_4mA;
    PORT_InitStructure.PORT_PULL_DOWN   = PORT_PULL_DOWN_OFF;
    PORT_Init(LED_PORT, (LED_PIN_0 | LED_PIN_1 | LED_PIN_2 | LED_PIN_3), &PORT_InitStructure);
    PORT_SetReset(LED_PORT, LED_PIN_0, SET);
    PORT_SetReset(LED_PORT, LED_PIN_1, SET);
    PORT_SetReset(LED_PORT, LED_PIN_2, SET);
    PORT_SetReset(LED_PORT, LED_PIN_3, SET);
}

/* retarget the C library printf function to the UART */
int	puts (const char * buf)
{
    uint32_t i;
    uint32_t nChars = 0;

    for (i=0; buf[i]; i++)
    {
        UART_Send(UART_MODULE, buf[i] );
        while ( UART_GetFlagStatus(UART_MODULE, UART_FLAG_TXFE)== RESET){}
        ++nChars;
    }

    return nChars;
}


/* retarget the C library printf function to the UART */
int _write(int fd, char * ptr, int len)
{
    size_t nChars = 0;

    if (fd == -1)
    {
        return 0;
    }

    for (; len > 0; --len)
    {
        UART_Send(UART_MODULE, (uint8_t) *ptr );
        while ( UART_GetFlagStatus(UART_MODULE, UART_FLAG_TXFE)== RESET){}
        ++ptr;
        ++nChars;
    }

    return nChars;
}

