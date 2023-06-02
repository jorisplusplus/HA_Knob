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
#include "driver/spi_master.h"
#include "motorindent.h"

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

#define PI     3.14159265359f
#define PI2    6.28318530718f
#define _3PI_2 4.71238898038f
#define FACTOR 75
#define SPEED 2857
#define _SQRT3_2 0.86602540378f
#define VSUPPLY 5.0f

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
    if (decimator == 4) {
        process_en = 1;
        decimator = 0;
    }
    return true;
}

//MAIN foc loop

static spi_device_handle_t spi_handle;

static esp_err_t spi_master_init(void) {
    
    spi_bus_config_t bus_config = {
        .mosi_io_num = -1,    // MOSI pin
        .miso_io_num = GPIO_MM_SDA,    // MISO pin
        .sclk_io_num = GPIO_MM_SCL,    // CLK pin
        .quadwp_io_num = -1,           // Not used
        .quadhd_io_num = -1,           // Not used
        .max_transfer_sz = 4,          // Use default maximum transfer size
    };

    spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_DISABLED);

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,             // No command bits
        .address_bits = 0,             // No address bits
        .dummy_bits = 0,               // No dummy bits
        .mode = 3,                     // SPI mode 0
        .duty_cycle_pos = 128,         // 50% duty cycle
        .cs_ena_pretrans = 0,          // No CS toggling before transactions
        .cs_ena_posttrans = 0,         // No CS toggling after transactions
        .clock_speed_hz = 10000000,     // Clock speed of 1MHz
        .input_delay_ns = 0,           // No input delay
        .spics_io_num = GPIO_NUM_7,    // CS pin
        .flags = 0,                    // No flags
        .queue_size = 1,               // Use 1 transaction at a time
        .pre_cb = NULL,                // No pre-transfer callback
        .post_cb = NULL,               // No post-transfer callback
    };

    spi_bus_add_device(SPI3_HOST, &dev_config, &spi_handle);

    spi_transaction_t transaction = {0};
    transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    transaction.length = 3 * 8;

    spi_device_transmit(spi_handle, &transaction);
    ESP_LOGI(TAG, "%d %d %d", transaction.rx_data[0], transaction.rx_data[1], transaction.rx_data[2]);
    return ESP_OK;

}

static float IRAM_ATTR read_angle()
{
        spi_transaction_t transaction = {0};
        transaction.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
        transaction.length = 3 * 8;

        spi_device_polling_transmit(spi_handle, &transaction);
        //ESP_LOGI(TAG, "%d %d %d", transaction.rx_data[0], transaction.rx_data[1], transaction.rx_data[2]);
        uint16_t val = (transaction.rx_data[0] << 6) | transaction.rx_data[1] >> 2;
        float angle = ((float) val)/16384.0f*PI2;
        //ESP_LOGI(TAG, "%d  ANGLE: %f", val, angle);
        return angle;       
}

#define AVG_NUM (20)
static float IRAM_ATTR read_angle_avg() {
    static float angles[AVG_NUM];
    float sumX = 0.0f;
    float sumY = 0.0f;

    for (int i = 0; i < (AVG_NUM - 1); i++)
    {
        angles[i] = angles[i + 1];
    }
    angles[AVG_NUM - 1] = read_angle();

    for (int i = 0; i < AVG_NUM; i++) {
        sumX += cosf(angles[i]);
        sumY += sinf(angles[i]);
    }
    float avgX = sumX/AVG_NUM;
    float avgY = sumY/AVG_NUM;

    float average = atan2f(avgY, avgX);
    if (average < 0.0f) {
        average += PI2;
    }

    return average;
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

#define POS_P (50.0f)

#define VEL_P (0.065f)
#define VEL_I (0.04f)
#define VEL_D (0.00007f)
#define VMAX (4*PI2)
#define ULIMIT (2.0f)

// PID controller function with integral limit
static float IRAM_ATTR pid_controller(float setpoint, float process_variable, float dt)
{
    // PID controller variables
    static float error = 0.0f;
    static float integral = 0.0f;
    static float derivative = 0.0f;
    static float prev_error = 0.0f;
    static float integral_limit = 5.0f;

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
      float _ca = cosf(angle_el);
      float _sa = sinf(angle_el);
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
    float last_angle = read_angle();
    int64_t last_update = esp_timer_get_time();
    int64_t last_update_PID = esp_timer_get_time();
    float vfilt = 0;
    int decimator = 0;
    int motor_enable = 0;
    ESP_LOGI(TAG, "Motor controller started");
    for (;;) {
        while(process_en == 0) {}
        process_en = 0;
        int64_t current_time = esp_timer_get_time();
        float angle = read_angle_avg();
        int64_t readtime = esp_timer_get_time() - current_time;
        
        
        decimator++;
        if (decimator == 10) {      
            float dangle = angle - last_angle;
            dangle = -dangle;
            if (dangle > PI) dangle -= PI2;
            if (dangle < -PI) dangle += PI2;
            int64_t dt = current_time - last_update_PID;
            float v = dangle/((float)dt/1000000);
            vfilt = v;    //Single Pole FIR low pass filter see https://fiiir.com/ decay = 0.75 
            //Step 1: Compute target Velocity
            motor_indent_t *indent = motor_indent_find(angle * 360 / PI2);

            float vtarget;

            if (indent != NULL) {
                float error_angle = indent->angle*PI2/360 - angle;
                vtarget = -error_angle*POS_P*indent->force;
                vtarget = fmin(vtarget, VMAX);
                motor_enable = 1;
            } else {
                motor_enable = 0;
                vtarget = 0;
            }

            //Step 2: Compute Voltage
            float timestep = ((float) current_time-last_update_PID)/1000000;
            //vtarget = 6.0f;
            Uq = pid_controller(vtarget, vfilt, timestep);
            if (Uq < -ULIMIT) Uq = -ULIMIT;
            if (Uq > ULIMIT) Uq = ULIMIT;
            last_update_PID = current_time;
            decimator = 0;
            //printf("%f \t%f \t%f \t%f \t%f \t%f \t%f \t%d \t%d\n", angle, last_angle, dangle, Uq, vtarget, vfilt, v, (int) dt, (int) readtime);
            last_angle = angle;
            //ESP_LOGI(TAG, "A:\t%f \t%f \t%f, U: \t%f, Vt: \t%f, Vf: \t%f, v: \t%f \t%d \t%d", angle, last_angle, dangle, Uq, vtarget, vfilt, v, (int) dt, (int) readtime);

        }

        setPhaseVoltage(motor_enable ? Uq : 0.0f, 0.0, electrical_angle(read_angle()));
        //setPhaseVoltage(0.5f, 0.0, electrical_angle(angle));
        //set_voltages(0, 0, 0);
        last_update = current_time;
    }
}

void motor_init(void)
{
    spi_master_init();
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
    
    for (int i = 0; i < 30; i ++) {
        float angle = read_angle_avg();
        offset = 0.0f;
        offset = electrical_angle(angle);
        ESP_LOGI(TAG, "OFFSET: %f %f", offset, angle);
        vTaskDelay(10);
    }
    
    set_voltages(0, 0, 0);
    vTaskDelay(100);
    mcpwm_timer_disable(timer);

    //Generate interrupts events on timer zero
    mcpwm_timer_event_callbacks_t callbacks;
    callbacks.on_full = NULL;
    callbacks.on_empty = md_update;
    callbacks.on_stop = NULL;
    for (int i = 15; i < 360; i+= 30) {
        motor_indent_register(i, 15, 15, 0.8f);
    }
    xTaskCreatePinnedToCore(vMotorProcessor, "FOC", 16000, NULL, 100, &s_processor_handle, 1);
    mcpwm_timer_register_event_callbacks(timer, &callbacks, s_processor_handle);
    ESP_LOGI(TAG, "Start the MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}