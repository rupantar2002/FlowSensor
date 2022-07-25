#pragma once
#include "driver/ledc.h"

void config_timer(void){
    ledc_timer_config_t config_timer = {
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq
        
    }
}