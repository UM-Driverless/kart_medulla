/******************************************************************************
 * @file    km_gpio.c
 * @brief   Implementación de la librería.
 *****************************************************************************/

#include "km_gpio.h"
#include "esp_log.h"

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas

/******************************* MACROS PRIVADAS ********************************/
// Constantes internas, flags de debug
// #define LIBRERIA_DEBUG 1

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)


/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
const uart_config_t uart0_config;
const uart_config_t uart2_config;

/******************************* FUNCIONES PÚBLICAS ***************************/

/* ---------- Initialization ---------- */
esp_err_t KM_GPIO_Init(void)
{
    esp_err_t ret;
    printf("GPIO_Init: PWM=%d DIR=%d H1=%d H2=%d H3=%d HY1=%d HY2=%d\n",
           PIN_STEER_PWM, PIN_STEER_DIR, PIN_MOTOR_HALL_1,
           PIN_MOTOR_HALL_2, PIN_MOTOR_HALL_3, PIN_HYDRAULIC_1, PIN_HYDRAULIC_2);

    /* ======================== ADC ======================== */
    // Configurar ADC1/ADC2 pins como entrada
    gpio_config_t adc_pin_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    // Lista de ADC1
    const gpio_num_t adc1_pins[] = {
        PIN_PEDAL_ACC, PIN_PEDAL_BRAKE, PIN_HYDRAULIC_1,
        PIN_PRESSURE_1, PIN_PRESSURE_2, PIN_PRESSURE_3
    };
    for (int i = 0; i < sizeof(adc1_pins)/sizeof(adc1_pins[0]); i++) {
        adc_pin_cfg.pin_bit_mask = 1ULL << adc1_pins[i];
        ret = gpio_config(&adc_pin_cfg);
        if (ret != ESP_OK) return ret;
    }

    // ADC2 pins (si se usan)
    adc_pin_cfg.pin_bit_mask = 1ULL << PIN_HYDRAULIC_2;
    ret = gpio_config(&adc_pin_cfg);
    if (ret != ESP_OK) return ret;

    /* ---------- DAC ---------- */
#ifdef CONFIG_IDF_TARGET_ESP32
    ret = dac_output_enable(DAC_CHAN_0); // CMD_ACC
    if (ret != ESP_OK) return ret;
    ret = dac_output_enable(DAC_CHAN_1); // CMD_BRAKE
    if (ret != ESP_OK) return ret;
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S3
    // SPI config for MCP4922 (DAC externo)
    static spi_device_handle_t mcp4922_handle = NULL;
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 1
    };
    ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI_HOST, &devcfg, &mcp4922_handle);
    if (ret != ESP_OK) return ret;
#endif

    /* ---------- PWM (LEDC) ---------- */
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
        .gpio_num = (gpio_num_t)PIN_STEER_PWM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ret = ledc_channel_config(&pwm_channel);
    if (ret != ESP_OK) return ret;

    /* ======================== DIR PIN ======================== */
    gpio_config_t dir_cfg = {
        .pin_bit_mask = 1ULL << PIN_STEER_DIR,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&dir_cfg);
    if (ret != ESP_OK) return ret;

    /* HALL SENSORS disabled - GPIO17/16 conflict with UART2 */
    #if 0
    gpio_config_t hall_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    const gpio_num_t hall_pins[] = { PIN_MOTOR_HALL_1, PIN_MOTOR_HALL_2, PIN_MOTOR_HALL_3 };
    for (int i = 0; i < sizeof(hall_pins)/sizeof(hall_pins[0]); i++) {
        hall_cfg.pin_bit_mask = 1ULL << hall_pins[i];
        ret = gpio_config(&hall_cfg);
        if (ret != ESP_OK) return ret;
    }
    #endif

    /* ======================== I2C ======================== */
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    ret = i2c_param_config(I2C_NUM_0, &i2c_cfg);
    if (ret != ESP_OK) return ret;
    ret = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) return ret;

    /* ---------- UART0 (Orin) ---------- */
    // uart0_config = {
    //     .baud_rate = 115200,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity    = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    // };
    // ret = uart_param_config(UART_NUM_0, &uart0_config);
    // if (ret != ESP_OK) return ret;
    // ret = uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
    // if (ret != ESP_OK) return ret;

    // /* ---------- UART2 (debug) ---------- */
    // uart2_config = {
    //     .baud_rate = 115200,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity    = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    // };
    // ret = uart_param_config(UART_NUM_2, &uart2_config);
    // if (ret != ESP_OK) return ret;
    // ret = uart_driver_install(UART_NUM_2, 1024, 0, 0, NULL, 0);
    // if (ret != ESP_OK) return ret;

    return ESP_OK;
}

/* ---------- Digital GPIO ---------- */
esp_err_t KM_GPIO_WriteDigital(gpio_num_t pin, uint8_t level)
{
    return gpio_set_level(pin, level ? 1 : 0);
}

uint8_t KM_GPIO_ReadDigital(gpio_num_t pin)
{
    return gpio_get_level(pin);
}

/* ---------- ADC ---------- */
uint16_t KM_GPIO_ReadADC(gpio_num_t pin)
{
    gpio_num_t gpio = (gpio_num_t)(pin);
    int raw_out_adc2 = 0;

    switch (gpio)
    {
        case GPIO_NUM_36: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_0); // pressure 1
        case GPIO_NUM_39: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_3); // pressure 2
        case GPIO_NUM_34: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_6); // pressure 3
        case GPIO_NUM_35: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_7); // pedal acc
        case GPIO_NUM_32: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_4); // pedal brake
        case GPIO_NUM_27:    // hydraulic 1 (ADC2_CH7)
            if (adc2_get_raw(ADC2_CHANNEL_7, ADC_WIDTH_BIT_12, &raw_out_adc2) == ESP_OK)
                return raw_out_adc2;
            return 0;
        case GPIO_NUM_14:    // hydraulic 2 (ADC2_CH6)
            if (adc2_get_raw(ADC2_CHANNEL_6, ADC_WIDTH_BIT_12, &raw_out_adc2) == ESP_OK)
                return raw_out_adc2;
            return 0;
            
        default: 
            return 0;
    }
}

/* ---------- DAC ---------- */
esp_err_t KM_GPIO_WriteDAC(gpio_num_t pin, uint8_t value)
{
    gpio_num_t gpio = (gpio_num_t)pin;

    if (gpio == PIN_CMD_ACC) return dac_output_voltage(DAC_CHAN_0, value);
    if (gpio == PIN_CMD_BRAKE) return dac_output_voltage(DAC_CHAN_1, value);

    return ESP_ERR_INVALID_ARG;
}

/* ---------- PWM ---------- */
esp_err_t KM_GPIO_WritePWM(gpio_num_t pin, uint32_t duty)
{
    gpio_num_t gpio = (gpio_num_t)pin;

    if (gpio == PIN_STEER_PWM) // Steering PWM
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
        return ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    }

    return ESP_ERR_INVALID_ARG;
}


/******************************* FUNCIONES PRIVADAS ***************************/
/**
 * @brief   Función interna no visible desde fuera
 */
//  void funcion_privada(void);

/******************************* FIN DE ARCHIVO ********************************/
