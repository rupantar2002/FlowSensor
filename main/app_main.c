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
#define FREQUENCY       900   //HZ
#define TIMER_DIVIDER  (80)    //  1 milisecond
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
#define TIMER_INTERRUPT_INTERVAL 1000 // 1 ms
#define ESP_INR_FLAG_DEFAULT  0  
#define BUFFER_SIZE 10

// Global vars
const char *TAG="app_main";

// queue
QueueHandle_t myQueue;
bool queueFull=false;

//float get_average(uint64_t *ar);

static void IRAM_ATTR gpio_interrupt_handler(void *arg){

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

    if(xQueueIsQueueFullFromISR(myQueue)==pdTRUE){
        queueFull=true;
    }
    if(xQueueIsQueueEmptyFromISR(myQueue)==pdTRUE){
        queueFull=false;
    }
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

float get_average(uint64_t *ar,uint16_t len){
    float sum=0;
    uint8_t i;

    sum=ar[0];
    for(i=1;i<BUFFER_SIZE;i++){
    //ESP_LOGI(TAG,"The value is %llu aT INDEX %u",ar[i],i);
     sum=(sum*i)+ar[i];
     sum=sum/(i+1);
    }
    //ESP_LOGI(TAG,"Sum is %llu",sum);
    return (sum);
}

uint64_t get_frequency(uint64_t counterVal){
    uint64_t freq = (1/(counterVal / 1000000));
    return freq;
}

void task_average(void *p){
    uint64_t buffer[BUFFER_SIZE]={0,};
    uint16_t index=0;
    uint64_t val=0;
    uint16_t arrCount=0;
    while(1){

        if(xQueueReceive(myQueue,&val,portMAX_DELAY)==pdTRUE){
            buffer[index]=val;
            //ESP_LOGI(TAG,"Recoved Data is %llu at index %u",buffer[index],index);
            if(arrCount<BUFFER_SIZE)
                arrCount++;
            if(index<BUFFER_SIZE){
                index++;
            }else{
                index=0; 
            }
        }else{
            ESP_LOGI(TAG,"Failed to read queue");
        }
        ESP_LOGI(TAG,"TThe average value is %0.2f",get_average(buffer,arrCount));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}

void app_main(void)
{
    config_gpio();
    config_pwm();

    myQueue=xQueueCreate(BUFFER_SIZE,(sizeof(uint64_t)*BUFFER_SIZE));
    xTaskCreate(task_average,"Task Average",2024,NULL,1,NULL);

}



