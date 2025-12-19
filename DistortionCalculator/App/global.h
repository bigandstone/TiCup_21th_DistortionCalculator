#ifndef __global_h
#define __global_h

#include "main.h"

#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "lcd.h"

#define SIMULATE 0
//！！！修改对应mx中tim2的预分频和分频
#define FS 102040 // 8-1 98-1
//#define FS 128000 // 5-1   25-1

#define FFT_NUM 1024
#define DELTA_FREQ (FS / FFT_NUM)

void Main_Task_Init(void);
void Main_Task(void);

void GetADC_Task_Init(void);
void GetADC_Task(void);

void FFT_Task_Init(void);
void FFT_Task(void);

void Show_Task_Init(void);
void Show_Task(void);

void Uart_Task(void);
void KEY_Task(void);

#endif
