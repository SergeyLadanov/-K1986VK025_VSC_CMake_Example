//
// Created by panukov.a on 17.03.2021.
//

#ifndef USE_INIT_H
#define USE_INIT_H

#include <MLDR187_lib.h>

#ifdef __cplusplus
extern "C"
{
#endif


#define	LED_PORT 		MDR_PORTD
#define LED_PIN_0		PORT_Pin_0
#define LED_PIN_1		PORT_Pin_1

void init_clock();

void init_uart();

void init_bkp();

void init_leds();

#ifdef __cplusplus
}
#endif

#endif //USE_INIT_H
