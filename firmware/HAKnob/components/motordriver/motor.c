#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/mcpwm_prelude.h"
#include "driver/mcpwm_gen.h"
#include "driver/gpio.h"
#include "math.h"


#define GPIO_LCD_RESET  GPIO_NUM_1
#define GPIO_LCD_SDA    GPIO_NUM_2
#define GPIO_IO_OFF     GPIO_NUM_3
#define GPIO_RGB        GPIO_NUM_4
#define GPIO_BAT_ADC    GPIO_NUM_5
#define GPIO_BAT_ADC_EN GPIO_NUM_6
#define GPIO_PUSH_UP    GPIO_NUM_9
#define GPIO_PUSH       GPIO_NUM_10
#define GPIO_MOTOR_UH   GPIO_NUM_21
#define GPIO_MOTOR_UL   GPIO_NUM_35
#define GPIO_MOTOR_VH   GPIO_NUM_47
#define GPIO_MOTOR_VL   GPIO_NUM_37
#define GPIO_MOTOR_WL   GPIO_NUM_36
#define GPIO_MOTOR_WH   GPIO_NUM_48
#define GPIO_MOTOR_DIAG GPIO_NUM_14
#define GPIO_MM_SDA     GPIO_NUM_13
#define GPIO_MM_SCL     GPIO_NUM_12
#define GPIO_MM_CS      -1
#define GPIO_LCD_BL     GPIO_NUM_39
#define GPIO_LCD_DC     GPIO_NUM_40
#define GPIO_LCD_CS     GPIO_NUM_41
#define GPIO_LCD_SCL    GPIO_NUM_42  

#define BLDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BLDC_MCPWM_PERIOD              500      // 50us, 20KHz
#define BLDC_SPIN_DIRECTION_CCW        false    // define the spin direction
#define BLDC_SPEED_UPDATE_PERIOD_US    333   

#define BLDC_DRV_FAULT_GPIO       GPIO_MOTOR_DIAG
#define BLDC_PWM_UH_GPIO          GPIO_MOTOR_UH
#define BLDC_PWM_UL_GPIO          GPIO_MOTOR_UL
#define BLDC_PWM_VH_GPIO          GPIO_MOTOR_VH
#define BLDC_PWM_VL_GPIO          GPIO_MOTOR_VL
#define BLDC_PWM_WH_GPIO          GPIO_MOTOR_WH
#define BLDC_PWM_WL_GPIO          GPIO_MOTOR_WL


#define BLDC_MCPWM_GEN_INDEX_HIGH 0
#define BLDC_MCPWM_GEN_INDEX_LOW  1

#define PI 3.14
#define FACTOR 75
#define SPEED 20000

static const char *TAG = "example";

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t operators[3];
mcpwm_cmpr_handle_t comparators[3];
mcpwm_fault_handle_t over_cur_fault = NULL;
mcpwm_gen_handle_t generators[3][2] = {};
TaskHandle_t s_processor_handle;

IRAM_ATTR static bool md_update(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{    
    xTaskResumeFromISR(user_ctx);
    return true;
}

//MAIN foc loop

IRAM_ATTR void vMotorProcessor(void *params) {
    for (;;) {
        vTaskSuspend( NULL );
        static int step = 0;

        int valA = 250 + (int) (sinf((float) step/SPEED*2.0*PI)*FACTOR);
        int valB = 250 + (int) (sinf((float) step/SPEED*2.0*PI+2.0*PI/3)*FACTOR);
        int valC = 250 + (int) (sinf((float) step/SPEED*2.0*PI+4.0*PI/3)*FACTOR);
        //ESP_LOGI(TAG, "%d %d %d", valA, valB, valC);
        mcpwm_comparator_set_compare_value(comparators[0], valA);
        mcpwm_comparator_set_compare_value(comparators[1], valB);
        mcpwm_comparator_set_compare_value(comparators[2], valC);

        step = (step+1) % SPEED;
    }
}

void motor_init(void)
{

    ESP_LOGI(TAG, "Create MCPWM timer");
    
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = BLDC_MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = BLDC_MCPWM_PERIOD,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    ESP_LOGI(TAG, "Create MCPWM operator");
    
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators[i]));
    }

    ESP_LOGI(TAG, "Connect operators to the same timer");
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators[i], timer));
    }

    ESP_LOGI(TAG, "Create comparators");
    //Update values when counter is at max
    mcpwm_comparator_config_t compare_config = {
        .flags.update_cmp_on_tep = true,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_new_comparator(operators[i], &compare_config, &comparators[i]));
        // set compare value to 0, we will adjust the speed in a period timer callback
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators[i], 250));
    }

    ESP_LOGI(TAG, "Create over current fault detector");
   
    mcpwm_gpio_fault_config_t gpio_fault_config = {
        .gpio_num = BLDC_DRV_FAULT_GPIO,
        .group_id = 0,
        .flags.active_level = 1, // low level means fault, refer to DRV8302 datasheet
        .flags.pull_up = true,   // internally pull up
    };
    ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gpio_fault_config, &over_cur_fault));

    ESP_LOGI(TAG, "Set brake mode on the fault event");
    mcpwm_brake_config_t brake_config = {
        .brake_mode = MCPWM_OPER_BRAKE_MODE_CBC,
        .fault = over_cur_fault,
        .flags.cbc_recover_on_tez = true,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(operators[i], &brake_config));
    }

    ESP_LOGI(TAG, "Create PWM generators");
    
    mcpwm_generator_config_t gen_config = {};
    const int gen_gpios[3][2] = {
        {BLDC_PWM_UH_GPIO, BLDC_PWM_UL_GPIO},
        {BLDC_PWM_VH_GPIO, BLDC_PWM_VL_GPIO},
        {BLDC_PWM_WH_GPIO, BLDC_PWM_WL_GPIO},
    };
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            gen_config.gen_gpio_num = gen_gpios[i][j];
            ESP_ERROR_CHECK(mcpwm_new_generator(operators[i], &gen_config, &generators[i][j]));
           
        }
         ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(generators[i][0],
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH), MCPWM_GEN_TIMER_EVENT_ACTION_END()));
            ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(generators[i][0],
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW), MCPWM_GEN_COMPARE_EVENT_ACTION_END()));
    }

    ESP_LOGI(TAG, "Setup deadtime");
    mcpwm_dead_time_config_t dt_config = {
        .posedge_delay_ticks = 5,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], &dt_config));
    }
    dt_config = (mcpwm_dead_time_config_t) {
        .negedge_delay_ticks = 5,
        .flags.invert_output = true,
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(generators[i][BLDC_MCPWM_GEN_INDEX_HIGH], generators[i][BLDC_MCPWM_GEN_INDEX_LOW], &dt_config));
    }

    //Generate interrupts events on timer zero
    mcpwm_timer_event_callbacks_t callbacks;
    callbacks.on_full = NULL;
    callbacks.on_empty = md_update;
    callbacks.on_stop = NULL;
    xTaskCreate(vMotorProcessor, "FOC", 8192, NULL, 1, &s_processor_handle);
    mcpwm_timer_register_event_callbacks(timer, &callbacks, s_processor_handle);

    ESP_LOGI(TAG, "Start the MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}