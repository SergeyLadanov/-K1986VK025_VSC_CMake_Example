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

void init_clock();

void init_uart();

void init_ADC();


#ifdef __cplusplus
}
#endif

#endif //USE_INIT_H
