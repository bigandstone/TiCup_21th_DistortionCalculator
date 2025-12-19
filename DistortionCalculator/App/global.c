#include "global.h"

// gloabal
#if SIMULATE == 1
float moni_sin[FFT_NUM];
float moni_base_freq = 10000;
float moni_mag[6] = {1.2, 1.0, 0.8, 0.5, 0.1, 0.3};
float moni_thd;
#endif


// main task=====================================================================
void Main_Task_Init(void)
{
    GetADC_Task_Init();
    Show_Task_Init();
    FFT_Task_Init();
}

void Main_Task(void)
{
    GetADC_Task();
    FFT_Task();
    Uart_Task();
    while(1) {
        Show_Task();
        KEY_Task();
    }
}



// ADC task=======================================================================
int16_t adc_input[FFT_NUM];
volatile uint8_t flag_adc_done = 0;

void GetADC_Task_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

void GetADC_Task(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_input, FFT_NUM);
    while(flag_adc_done == 0);
    flag_adc_done = 0;
}



// FFT task=======================================================================
float mean = 0;
const arm_cfft_instance_f32 *S_ptr;
float fft_input[FFT_NUM * 2];
float fft_mag[FFT_NUM / 2];

float fft_base_freq;
uint16_t fft_base_freq_n;
float fft_main_mag[5];

float thd;

void FFT_Task_Init(void)
{
#if SIMULATE == 1
    for (uint16_t i = 0; i < FFT_NUM; i++)
    {
        moni_sin[i] = moni_mag[0] 
                    + moni_mag[1] * sin(2 * 3.14 * 1 * moni_base_freq / FS * i) 
                    + moni_mag[2] * sin(2 * 3.14 * 2 * moni_base_freq / FS * i) 
                    + moni_mag[3] * sin(2 * 3.14 * 3 * moni_base_freq / FS * i) 
                    + moni_mag[4] * sin(2 * 3.14 * 4 * moni_base_freq / FS * i) 
                    + moni_mag[5] * sin(2 * 3.14 * 5 * moni_base_freq / FS * i);
    }
#endif
#if FFT_NUM == 64
    S_ptr = &arm_cfft_sR_f32_len64;
#elif FFT_NUM == 128
    S_ptr = &arm_cfft_sR_f32_len128;
#elif FFT_NUM == 256
    S_ptr = &arm_cfft_sR_f32_len256;
#elif FFT_NUM == 512
    S_ptr = &arm_cfft_sR_f32_len512;
#elif FFT_NUM == 1024
    S_ptr = &arm_cfft_sR_f32_len1024;
#elif FFT_NUM == 2048
    S_ptr = &arm_cfft_sR_f32_len2048;
#elif FFT_NUM == 4096
    S_ptr = &arm_cfft_sR_f32_len4096;
#else
    #error "Unsupported FFT length"
#endif
}

void FFT_Task(void)
{
    for (int i = 0; i < FFT_NUM; i++)
    {
#if SIMULATE == 1
        mean += moni_sin[i];
#elif SIMULATE == 0
        mean += adc_input[i];
#endif
    }
    mean /= FFT_NUM;
    for (int i = 0; i < FFT_NUM; i++)
    {
#if SIMULATE == 1
        moni_sin[i] -= mean;
#elif SIMULATE == 0
        adc_input[i] -= mean;
#endif
    }

    for (int i = 0; i < FFT_NUM; i++)
    {
#if SIMULATE == 1
        fft_input[i * 2] = moni_sin[i] * 0.5f * (1.0f - cosf(2 * 3.14159265f * i / (FFT_NUM - 1))); // 实部
#elif SIMULATE == 0
        fft_input[i * 2] = adc_input[i] * 0.5f * (1.0f - cosf(2 * 3.14159265f * i / (FFT_NUM - 1))); // 实部
        //fft_input[i * 2] = adc_input[i]; // 实部
#endif
        fft_input[i * 2 + 1] = 0; // 虚部设为0
    }
    // fft
    arm_cfft_f32(S_ptr, fft_input, 0, 1);
    arm_cmplx_mag_f32(fft_input, fft_mag, FFT_NUM / 2);
    fft_mag[0] = fft_mag[0] / 2; // actually can drop it

    // search basic freq
    for (uint16_t i = 0; i < FFT_NUM / 2; i++)
    {
        if (fft_mag[i] > fft_main_mag[0])
        {
            fft_main_mag[0] = fft_mag[i];
            fft_base_freq_n = i;
        }
    }
    // 频谱归一化
    for (int i = 0; i < FFT_NUM / 2; i++)
    {
        fft_mag[i] = fft_mag[i] / fft_main_mag[0];
    }
    fft_base_freq = DELTA_FREQ * fft_base_freq_n;

    /* 寻找高次谐波 */
    for (uint8_t j = 1; j <= 4; j++)
    {
        for (uint16_t i = fft_base_freq_n * (j + 1) - 3; i < fft_base_freq_n * (j + 1) + 3; i++)
        {
            if (fft_mag[i] > fft_main_mag[j])
            {
                fft_main_mag[j] = fft_mag[i];
            }
        }
    }
    //calculate thd
    for (uint8_t i = 1; i < 5; i++)
    {
        thd += fft_main_mag[i] * fft_main_mag[i];
    }
    thd = sqrt(thd);
#if SIMULATE
    // calculate thd
    for (uint8_t i = 1; i < 5; i++)
    {
        moni_thd += moni_mag[i + 1] * moni_mag[i + 1];
    }
    moni_thd = sqrt(moni_thd);
#endif
}



// Show task=======================================================================
char tx_buff[50];
uint8_t key_flag;
float thd_bias = 0.8;
float k_a;

void Show_Task_Init(void)
{
    GPIOC->ODR |= 0xff << 8;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

    LCD_Init();
    LCD_Clear(Black);
    LCD_SetBackColor(Black);
    LCD_SetTextColor(Yellow);
}

void Show_Task(void)
{
    static char temp[50];
    if(key_flag == 0) {
        sprintf(temp, "NAME   ID");
        LCD_DisplayStringLine(Line1, (u8 *)temp);
        
        sprintf(temp, "MRJJ   20233902017");
        LCD_DisplayStringLine(Line3, (u8 *)temp);

        sprintf(temp, "LZH    20233902013");
        LCD_DisplayStringLine(Line4, (u8 *)temp);
    }
    else if(key_flag == 1) {
        sprintf(temp, "2th = %.3f", fft_main_mag[1]);
        LCD_DisplayStringLine(Line0, (u8 *)temp);
        
        sprintf(temp, "3th = %.3f", fft_main_mag[2]);
        LCD_DisplayStringLine(Line1, (u8 *)temp);
        
        sprintf(temp, "4th = %.3f", fft_main_mag[3]);
        LCD_DisplayStringLine(Line2, (u8 *)temp);
        
        sprintf(temp, "5th = %.3f", fft_main_mag[4]);
        LCD_DisplayStringLine(Line3, (u8 *)temp);

        sprintf(temp, "base_f=%.2fHz", fft_base_freq);
        LCD_DisplayStringLine(Line4, (u8 *)temp);

        sprintf(temp, "thd = %.3f%%", thd * 100);
        LCD_DisplayStringLine(Line5, (u8 *)temp);
#if SIMULATE
        sprintf(temp, "delta_thd=%.2f%%", fabs(thd - moni_thd) * 100);
        LCD_DisplayStringLine(Line3, (u8 *)temp);
#endif
    }
    else if(key_flag == 2){
        sprintf(temp, "wave form:");
        LCD_DisplayStringLine(Line0, (u8 *)temp);
        
        uint8_t bias = 5;
        uint16_t rect_h = 200, rect_w = 320 - 2*bias;//240x320
        LCD_DrawRect(240 - bias - rect_h, 320 -  bias, rect_h, rect_w);//y,x
        
        uint16_t pot_x = 0, pot_y = 0;
        uint16_t pot_num = (uint16_t)(FS * 2 / fft_base_freq);
        uint8_t step = rect_w / pot_num;
        
        for(uint16_t i = 0;i < pot_num;i ++) {
#if SIMULATE == 1
            LCD_DrawRect(240 - 2*bias - rect_h/2 - moni_sin[i] * rect_h / 7, 320 - (i*step + bias), 2,2);
            for(uint16_t j = 0;j < step - 1;j++) {
                LCD_DrawRect(240 - 2*bias - rect_h/2 - (moni_sin[i] + (moni_sin[i+1] - moni_sin[i])/step * j) * rect_h / 7, 
                            320 - (i*step + j + bias), 2,2);
            
            }
#elif SIMULATE == 0
            LCD_DrawRect(240 - 2*bias - rect_h/2 - adc_input[i] * rect_h / 4000, 320 - (i*step + bias), 2,2);
            for(uint16_t j = 0;j < step - 1;j++) {
                LCD_DrawRect(240 - 2*bias - rect_h/2 - (adc_input[i] + (adc_input[i+1] - adc_input[i])/step * j) * rect_h / 4000, 
                            320 - (i*step + j + bias), 2,2);
            }
#endif
        }
    }
}


void Uart_Task(void) {
    for (uint16_t i = 0; i < FFT_NUM/2; i++)
    {
        sprintf(tx_buff, "%d,%d,%d,%f\n", i,
#if SIMULATE == 1
                moni_sin[i],
#elif SIMULATE == 0
                adc_input[i],
#endif
        i * DELTA_FREQ, i < FFT_NUM/2?  fft_mag[i]: 0);
        HAL_UART_Transmit(&huart1, (u8 *)tx_buff, strlen(tx_buff), 10);
        HAL_Delay(1);
    }
}

void KEY_Task(void)
{
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == GPIO_PIN_RESET) {
        HAL_Delay(20);
        while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == GPIO_PIN_RESET);
        if(++key_flag == 3) key_flag = 0;
        LCD_Clear(Black);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Stop_IT(&htim2);
        flag_adc_done = 1;
    }
}
