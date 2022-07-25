#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"

#define PWM_GPIO       GPIO_NUM_4
#define DUTY_CYCLE     512
#define TIMER_DIVIDER  (16)                                //  Hardware timer clock divider
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)    // convert counter value to seconds

// Functions
void config_pwm(void);
void change_dutyCycle(void);


//Task
void pwm_task(void *p){

    config_pwm();
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,DUTY_CYCLE);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);

    while (1)
    {
    // for polling functions
    change_dutyCycle();
    }
}

void timer_task(void *p){

    config_timer();

    uint64_t stopTime=0;
    uint64_t startTime=0;
    uint64_t deltaTime=0;

    while (1)
    {



    }
    
}

void app_main(void)
{
    // create task
    xTaskCreate(pwm_task,"PWM Task",1024,NULL,1,NULL);
    xTaskCreate(timer_task,"Timer Task",2024,NULL,1,NULL);
}


// Functions
void config_pwm(void){
    ledc_timer_config_t config_timer={
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution =LEDC_TIMER_10_BIT,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_channel_config_t config_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_GPIO,
        .duty = 0,
        .hpoint = 0,
    };

    ledc_timer_config(&config_timer);
    ledc_channel_config(&config_channel);
}

void change_dutyCycle(void){
    uint32_t i;
    for(i=0;i<1023;i++){
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,i);
        ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);   
        vTaskDelay(pdMS_TO_TICKS(50));     
    }
    for(i=1023;i>=0;i++){
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,i);
        ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);       
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void config_timer(void){

    timer_config_t config_timer = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };

    timer_init(TIMER_GROUP_0,TIMER_1,&config_timer);
}