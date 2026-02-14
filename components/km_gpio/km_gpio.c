/******************************************************************************
 * @file    km_gpio.c
 * @brief   Implementación de la librería.
 *****************************************************************************/

#include "km_gpio.h"
#include <stdio.h>   // solo si es necesario para debug interno

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas

/******************************* MACROS PRIVADAS ********************************/
// Constantes internas, flags de debug
// #define LIBRERIA_DEBUG 1

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)

/* ---------- USB (UART0 to ORIN) ---------- */
const gpio_config_t pin_usb_uart_rx = {
    .pin_bit_mask = 1ULL << PIN_USB_UART_RX,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_usb_uart_tx = {
    .pin_bit_mask = 1ULL << PIN_USB_UART_TX,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

/* ---------- ADC INPUTS (Sensors) ---------- */
/* ADC1 - input only */
const gpio_config_t pin_pressure_1 = {
    .pin_bit_mask = 1ULL << PIN_PRESSURE_1,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_pressure_2 = {
    .pin_bit_mask = 1ULL << PIN_PRESSURE_2,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_pressure_3 = {
    .pin_bit_mask = 1ULL << PIN_PRESSURE_3,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_pedal_acc = {
    .pin_bit_mask = 1ULL << PIN_PEDAL_ACC,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_pedal_brake = {
    .pin_bit_mask = 1ULL << PIN_PEDAL_BRAKE,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_hydraulic_1 = {
    .pin_bit_mask = 1ULL << PIN_HYDRAULIC_1,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

/* ADC2 - allowed (WiFi not used) */
const gpio_config_t pin_hydraulic_2 = {
    .pin_bit_mask = 1ULL << PIN_HYDRAULIC_2,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

/* ---------- DAC OUTPUTS ---------- */
const gpio_config_t pin_cmd_acc = {
    .pin_bit_mask = 1ULL << PIN_CMD_ACC,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_cmd_brake = {
    .pin_bit_mask = 1ULL << PIN_CMD_BRAKE,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

/* ---------- STEERING MOTOR ---------- */
const gpio_config_t pin_steer_pwm = {
    .pin_bit_mask = 1ULL << PIN_STEER_PWM,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_steer_dir = {
    .pin_bit_mask = 1ULL << PIN_STEER_DIR,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

/* ---------- HALL SENSORS ---------- */
const gpio_config_t pin_motor_hall_1 = {
    .pin_bit_mask = 1ULL << PIN_MOTOR_HALL_1,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_POSEDGE
};

const gpio_config_t pin_motor_hall_2 = {
    .pin_bit_mask = 1ULL << PIN_MOTOR_HALL_2,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_POSEDGE
};

const gpio_config_t pin_motor_hall_3 = {
    .pin_bit_mask = 1ULL << PIN_MOTOR_HALL_3,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_POSEDGE
};

/* ---------- I2C (AS5600) ---------- */
const gpio_config_t pin_i2c_scl = {
    .pin_bit_mask = 1ULL << PIN_I2C_SCL,
    .mode = GPIO_MODE_INPUT_OUTPUT_OD,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

const gpio_config_t pin_i2c_sda = {
    .pin_bit_mask = 1ULL << PIN_I2C_SDA,
    .mode = GPIO_MODE_INPUT_OUTPUT_OD,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

/* ---------- STATUS LED ---------- */
const gpio_config_t pin_status_led = {
    .pin_bit_mask = 1ULL << PIN_STATUS_LED,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};


/******************************* DECLARACION FUNCIONES PRIVADAS ***************/


/******************************* FUNCIONES PÚBLICAS ***************************/

/* ---------- Initialization ---------- */
esp_err_t KM_GPIO_Init(void)
{
    esp_err_t ret;

    // Enable DAC channels
    ret = dac_output_enable(DAC_CHAN_0); // CMD_ACC
    if (ret != ESP_OK) return ret;
    ret = dac_output_enable(DAC_CHAN_1); // CMD_BRAKE
    if (ret != ESP_OK) return ret;

    // Setup PWM (LEDC)
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&pwm_timer);
    if (ret != ESP_OK) return ret;

    ledc_channel_config_t pwm_channel = {
        .gpio_num = (gpio_num_t)pin_steer_pwm.pin_bit_mask,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ret = ledc_channel_config(&pwm_channel);
    if (ret != ESP_OK) return ret;

    // Initialize I2C
    ret = KM_GPIO_I2CInit();
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

/* ---------- Digital GPIO ---------- */
uint8_t KM_GPIO_ReadPin(const gpio_config_t *pin)
{
    return gpio_get_level((gpio_num_t)(pin->pin_bit_mask));
}

esp_err_t KM_GPIO_WritePinHigh(const gpio_config_t *pin)
{
    return gpio_set_level((gpio_num_t)(pin->pin_bit_mask), 1);
}

esp_err_t KM_GPIO_WritePinLow(const gpio_config_t *pin)
{
    return gpio_set_level((gpio_num_t)(pin->pin_bit_mask), 0);
}

/* ---------- ADC ---------- */
uint16_t KM_GPIO_ReadADC(const gpio_config_t *pin)
{
    gpio_num_t gpio = (gpio_num_t)(pin->pin_bit_mask);
    int raw_out_adc2 = 0;

    switch (gpio)
    {
        case GPIO_NUM_36: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_0); // pressure 1
        case GPIO_NUM_39: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_3); // pressure 2
        case GPIO_NUM_34: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_6); // pressure 3
        case GPIO_NUM_35: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_7); // pedal acc
        case GPIO_NUM_32: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_4); // pedal brake
        case GPIO_NUM_33: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_5); // hydraulic 1
        case GPIO_NUM_13:    // hydraulic 2
            if (adc2_get_raw(ADC2_CHANNEL_4, ADC_WIDTH_BIT_12, &raw_out_adc2) == ESP_OK)
                return raw_out_adc2;
            return 0;
            
        default: return 0;
    }
}

/* ---------- DAC ---------- */
esp_err_t KM_GPIO_WriteDAC(const gpio_config_t *pin, uint8_t value)
{
    gpio_num_t gpio = (gpio_num_t)(pin->pin_bit_mask);

    if (gpio == GPIO_NUM_25) return dac_output_voltage(DAC_CHAN_0, value);
    if (gpio == GPIO_NUM_26) return dac_output_voltage(DAC_CHAN_1, value);

    return ESP_ERR_INVALID_ARG;
}

/* ---------- PWM ---------- */
esp_err_t KM_GPIO_WritePWM(const gpio_config_t *pin, uint8_t duty)
{
    gpio_num_t gpio = (gpio_num_t)(pin->pin_bit_mask);

    if (gpio == GPIO_NUM_27) // Steering PWM
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
        return ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    }

    return ESP_ERR_INVALID_ARG;
}

/* ---------- I2C ---------- */
esp_err_t KM_GPIO_I2CInit(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) return ret;

    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}


/******************************* FUNCIONES PRIVADAS ***************************/
/**
 * @brief   Función interna no visible desde fuera
 */
//  void funcion_privada(void);

/******************************* FIN DE ARCHIVO ********************************/
