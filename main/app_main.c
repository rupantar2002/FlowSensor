#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#define PWM_GPIO       GPIO_NUM_10
#define INTR_GPIO      GPIO_NUM_4
#define LED_GPIO       GPIO_NUM_2
#define DUTY_CYCLE     (1024/2)
#define FREQUENCY       101   //HZ
#define TIMER_DIVIDER  (80)    //  1 milisecond
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
#define TIMER_INTERRUPT_INTERVAL 1000 // 1 ms
#define ESP_INR_FLAG_DEFAULT  0  
#define BUFFER_SIZE 10

// Global vars
const char *TAG="app_main";
volatile uint8_t state = 0;
volatile int64_t timerCounter=0;
volatile int64_t startTime=0;
volatile int64_t deltaTime=0;

// queue
QueueHandle_t myQueue;

// int64_t array_avg(int64_t *ar){
//     int i;
//     uint64_t sum=0;
//     for(i=0;i<BUFFER_SIZE;i++){
//         sum=sum+ar[i];
//     }

//     return (sum/BUFFER_SIZE);
// }

// void store_buffer(int64_t data){

//     if(index1==(BUFFER_SIZE-1)){
//         filled1=true;
//         avg=array_avg(buffer1);
//         filled1=false;
//         index1=0;
//     }

//     if(index1<BUFFER_SIZE && !filled1)
//     {
//         buffer1[index1]=data;
//         index1++;
//     }
// }
// uint64_t g_timeDiff=0;
// uint64_t ga_timeDiffArr[BUFFER_SIZE];
// uint16_t g_currentIndex=0;
// uint8_t index=0;

void IRAM_ATTR gpio_interrupt_handler(void *arg){
    static uint64_t l_startTime=0;
    uint64_t l_timeDiff=0;
    uint64_t l_timeNow=esp_timer_get_time();  
    if(l_startTime==0){
        l_startTime= l_timeNow;
    }
    else{
        //store_buffer(l_timeNow-l_startTime);   
        l_timeDiff= l_timeNow-l_startTime;
        l_startTime=l_timeNow;
        // send data
        xQueueSendFromISR(myQueue,&l_timeDiff,0);

    }
}


static bool IRAM_ATTR timer_interrupt_handler(void *arg){
    //state=!state;
    //gpio_set_level(LED_GPIO,state);
   // timerCounter++;
    return pdTRUE;
}

void config_gpio(void){
    gpio_set_direction(INTR_GPIO,GPIO_MODE_INPUT);
    gpio_set_direction(PWM_GPIO,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GPIO,GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(INTR_GPIO, GPIO_PULLDOWN_ONLY);
    // interrupt specific
    gpio_set_intr_type(INTR_GPIO,GPIO_PIN_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INR_FLAG_DEFAULT);
    gpio_isr_handler_add(INTR_GPIO,gpio_interrupt_handler,NULL);
}

void config_pwm(void){

    ledc_timer_config_t config_timer={
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution =LEDC_TIMER_10_BIT,
        .freq_hz = FREQUENCY,
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
    // start pwm
    ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,DUTY_CYCLE);
    ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);

}


void config_timer(void){
    timer_group_t group= TIMER_GROUP_1;
    timer_idx_t index= TIMER_1;
    timer_config_t config= {
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
        .divider = TIMER_DIVIDER,
    };

    timer_init(group,index,&config);
    timer_set_counter_value(group,index,0);
    timer_set_alarm_value(group,index,TIMER_INTERRUPT_INTERVAL);
    timer_isr_callback_add(group,index,timer_interrupt_handler,(void *) NULL,ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL2);
    timer_start(group,index);

}

//task
void running_avg_task(void * p){
    int64_t buffer[BUFFER_SIZE];
    int8_t index;
    
}

void app_main(void)
{
    config_gpio();
    config_pwm();
    //config_timer();
    myQueue=xQueueCreate(BUFFER_SIZE,sizeof(int64_t)*10);

    while (1){
        ESP_LOGI(TAG,"main Task Started");
    }
}



