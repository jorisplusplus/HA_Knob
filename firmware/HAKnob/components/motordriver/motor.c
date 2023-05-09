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
#include "driver/i2c.h"

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
#define BLDC_MCPWM_PERIOD              2000      // 50us, 20KHz
#define BLDC_SPIN_DIRECTION_CCW        false    // define the spin direction
#define BLDC_SPEED_UPDATE_PERIOD_US    333   

#define BLDC_DRV_FAULT_GPIO       GPIO_MOTOR_DIAG
#define BLDC_PWM_UH_GPIO          GPIO_MOTOR_UH
#define BLDC_PWM_UL_GPIO          GPIO_MOTOR_UL
#define BLDC_PWM_VH_GPIO          GPIO_MOTOR_VH
#define BLDC_PWM_VL_GPIO          GPIO_MOTOR_VL
#define BLDC_PWM_WH_GPIO          GPIO_MOTOR_WH
#define BLDC_PWM_WL_GPIO          GPIO_MOTOR_WL


#define I2C_MASTER_SCL_IO           GPIO_MM_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_MM_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          1000000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define LCD_HOST    SPI2_HOST
#define MM_ADDRESS 0b0000110


#define BLDC_MCPWM_GEN_INDEX_HIGH 0
#define BLDC_MCPWM_GEN_INDEX_LOW  1

#define PI M_PI
#define PI2    6.28318530718
#define _3PI_2 4.71238898038
#define FACTOR 75
#define SPEED 2857
#define _SQRT3_2 0.86602540378f
#define VSUPPLY 5.0

static const char *TAG = "example";

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t operators[3];
mcpwm_cmpr_handle_t comparators[3];
mcpwm_fault_handle_t over_cur_fault = NULL;
mcpwm_gen_handle_t generators[3][2] = {};
TaskHandle_t s_processor_handle;

float offset = 0.0f;

volatile int process_en = 0;

IRAM_ATTR static bool md_update(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx)
{    
    //xTaskResumeFromISR(user_ctx);
    //vMotorProcessor(NULL);
    static int decimator = 0;
    decimator++;
    if (decimator == 10) {
    process_en = 1;
    decimator = 0;
    }
    return true;
}

//MAIN foc loop

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static float IRAM_ATTR read_angle()
{
        uint8_t txdata;
        uint8_t data[2];
        txdata = 0x03;
        esp_err_t res = i2c_master_write_read_device(I2C_MASTER_NUM, MM_ADDRESS, &txdata, 1, data, 1, 10);
        txdata = 0x04;
        res = i2c_master_write_read_device(I2C_MASTER_NUM, MM_ADDRESS, &txdata, 1, &data[1], 1, 10);
        uint16_t val = (data[0] << 6) | data[1] >> 2;
        float angle = ((float) val)/16384*PI2;
        //ESP_LOGI(TAG, "%d %d  ANGLE: %f", res, val, angle);
        return angle;       
}


/**
 * @brief Determines the electrical angle of the motor assumes CCW and 7 pole pairs
 * 
 * @param raw Input raw angle in between 0 and 2PI
 * @return IRAM_ATTR 
 */
static float IRAM_ATTR electrical_angle(float raw) {
        float angle = (-7*raw) - offset;
        angle = fmod(angle, PI2);
        if (angle < 0) angle += (PI2);
        return angle;
}

static void IRAM_ATTR set_voltages(float Ua, float Ub, float Uc) {
        int valA = Ua/VSUPPLY*BLDC_MCPWM_PERIOD;
        valA = valA < 0 ? 0 : valA > BLDC_MCPWM_PERIOD ? BLDC_MCPWM_PERIOD : valA;
        int valB = Ub/VSUPPLY*BLDC_MCPWM_PERIOD;
        valB = valB < 0 ? 0 : valB > BLDC_MCPWM_PERIOD ? BLDC_MCPWM_PERIOD : valB;
        int valC = Uc/VSUPPLY*BLDC_MCPWM_PERIOD;
        valC = valC < 0 ? 0 : valC > BLDC_MCPWM_PERIOD ? BLDC_MCPWM_PERIOD : valC;
        //ESP_LOGI(TAG, "%d %d %d", valA, valB, valC);
        mcpwm_comparator_set_compare_value(comparators[0], valA);
        mcpwm_comparator_set_compare_value(comparators[1], valB);
        mcpwm_comparator_set_compare_value(comparators[2], valC);
}

#define POS_P (5.0f)

#define VEL_P (0.08f)
#define VEL_I (0.001f)
#define VEL_D (0.0001f)
#define VMAX (2*PI2)
#define ULIMIT (1.4f)

// PID controller function with integral limit
static float IRAM_ATTR pid_controller(float setpoint, float process_variable, float dt)
{
    // PID controller variables
    static float error = 0.0f;
    static float integral = 0.0f;
    static float derivative = 0.0f;
    static float prev_error = 0.0f;
    static float integral_limit = 10.0f;

    // Calculate the error term
    error = setpoint - process_variable;
    
    // Calculate the integral term
    integral += error * dt;
    
    // Apply integral limit
    if (integral > integral_limit)
    {
        integral = integral_limit;
    }
    else if (integral < -integral_limit)
    {
        integral = -integral_limit;
    }
    
    // Calculate the derivative term
    derivative = (error - prev_error) / dt;
    prev_error = error;
    
    // Calculate the control output
    float output = VEL_P * error + VEL_I * integral + VEL_D * derivative;
    
    return output;
}

float IRAM_ATTR _normalizeAngle(float angle){
  float a = fmod(angle, PI2);
  return a >= 0 ? a : (a + PI2);
}

void IRAM_ATTR setPhaseVoltage(float Uq, float Ud, float angle_el) {
      // Sinusoidal PWM modulation
      // Inverse Park + Clarke transformation

      // angle normalization in between 0 and 2pi
      // only necessary if using _sin and _cos - approximation functions
      angle_el = _normalizeAngle(angle_el);
      float _ca = cos(angle_el);
      float _sa = sin(angle_el);
      // Inverse park transform
      float Ualpha =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
      float Ubeta =  _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;

      // center = modulation_centered ? (driver->voltage_limit)/2 : Uq;
      float center = 5.0/2;
      // Clarke transform
      float Ua = Ualpha + center;
      float Ub = -0.5f * Ualpha  + _SQRT3_2 * Ubeta + center;
      float Uc = -0.5f * Ualpha - _SQRT3_2 * Ubeta + center;

      set_voltages(Ua, Ub, Uc);
}

void IRAM_ATTR vMotorProcessor(void *params) {
    float Uq = 1.5f;
    float voltage_power_supply = 5.0;
    int64_t last_update = esp_timer_get_time();
    int64_t last_update_PID = esp_timer_get_time();
    float last_angle = read_angle();
    float vfilt = 0;
    float desired_angle = PI;
    int decimator = 0;
    for (;;) {
        while(process_en == 0) {}
        process_en = 0;
        int64_t current_time = esp_timer_get_time();
        float angle = read_angle();
        int64_t readtime = esp_timer_get_time() - current_time;
        
        int64_t dt = current_time - last_update;
         decimator++;
        if (decimator == 5) {
            float dangle = angle - last_angle;
            if (dangle > PI) dangle -= PI2;
            if (dangle < -PI) dangle += PI2;
            float v = dangle/((float)dt/1000000);
            vfilt = vfilt*0.95f + 0.05f*v;    //Single Pole FIR low pass filter see https://fiiir.com/ decay = 0.75


       
            //Step 1: Compute target Velocity
            if (angle < PI/2) {
                desired_angle = PI/4;
            } else if (angle < PI) {
                desired_angle = PI*3/4;
            } else if (angle < 1.5*PI) {
                desired_angle = PI*5/4;
            } else {
                desired_angle = PI*7/4;
            }


            float error_angle = desired_angle - angle;
            float vtarget = -error_angle*POS_P;
            vtarget = fmin(vtarget, VMAX);
            //vtarget = 10.0f;

            //Step 2: Compute Voltage
            float timestep = ((float) current_time-last_update_PID)/1000000;
            Uq = pid_controller(vtarget, vfilt, timestep);
            if (Uq < -ULIMIT) Uq = -ULIMIT;
            if (Uq > ULIMIT) Uq = ULIMIT;
            last_update_PID = current_time;
            decimator = 0;
            ESP_LOGI(TAG, "A:%f %f, U: %f, Vt: %f, Vf: %f, v: %f %d", angle, dangle, Uq, vtarget, vfilt, v, (int) dt);

        }
        

        setPhaseVoltage(Uq, 0.0, electrical_angle(angle));
        //set_voltages(0, 0, 0);
        last_update = current_time;
        last_angle = angle;
    }
}

void motor_init(void)
{
    i2c_master_init();
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

    ESP_LOGI(TAG, "Start the MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "A");
    setPhaseVoltage(2.0, 0, _3PI_2);
    vTaskDelay(100);
    offset = 0;
    offset = electrical_angle(read_angle(NULL));
    ESP_LOGI(TAG, "OFFSET: %f", offset);
    set_voltages(0, 0, 0);
    vTaskDelay(100);
    mcpwm_timer_disable(timer);

    //Generate interrupts events on timer zero
    mcpwm_timer_event_callbacks_t callbacks;
    callbacks.on_full = NULL;
    callbacks.on_empty = md_update;
    callbacks.on_stop = NULL;
    xTaskCreatePinnedToCore(vMotorProcessor, "FOC", 16000, NULL, 100, &s_processor_handle, 1);
    mcpwm_timer_register_event_callbacks(timer, &callbacks, s_processor_handle);
    ESP_LOGI(TAG, "Start the MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}