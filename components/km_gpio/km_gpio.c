/******************************************************************************
 * @file    km_gpio.c
 * @brief   Implementation of the KM_GPIO hardware abstraction library.
 *****************************************************************************/

#include "km_gpio.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/******************************* INCLUDES INTERNOS ****************************/
// Headers internos opcionales, dependencias privadas

/******************************* MACROS PRIVADAS ********************************/
// Constantes internas, flags de debug
// #define LIBRERIA_DEBUG 1

/******************************* VARIABLES PRIVADAS ***************************/
// Variables globales internas (static)

static const char *GPIO_TAG = "km_gpio";

#ifdef CONFIG_IDF_TARGET_ESP32S3
static const char *SPI_TAG = "mcp4922";

/* File scope, NOT function scope. This handle used to be declared `static`
 * inside KM_GPIO_Init(), where KM_GPIO_WriteDAC() could not see it — so even a
 * fully implemented write had nothing to transmit through. */
static spi_device_handle_t mcp4922_handle = NULL;

/* Counters the diagnostics report. Reads/writes are not atomic; they are
 * indicative only and must not be used for control decisions. */
static uint32_t mcp4922_writes_ok   = 0;
static uint32_t mcp4922_writes_fail = 0;
static uint16_t mcp4922_last_cmd    = 0;

/* MCP4922 16-bit command word (datasheet section 5.0), MSB first:
 *   bit 15    channel: 0 = A, 1 = B
 *   bit 14    BUF:  0 = VREF unbuffered
 *   bit 13    /GA:  1 = gain 1x  (0 would be 2x and clip against the 5 V rail)
 *   bit 12    /SHDN: 1 = output active
 *   bits 11-0 the 12-bit code */
#define MCP4922_CH_A    0x0000u
#define MCP4922_CH_B    0x8000u
#define MCP4922_BUF_OFF 0x0000u
#define MCP4922_GAIN_1X 0x2000u
#define MCP4922_ACTIVE  0x1000u
#endif


/******************************* DECLARACION FUNCIONES PRIVADAS ***************/
static esp_adc_cal_characteristics_t adc1_chars;   // filled by KM_GPIO_Init

const uart_config_t uart0_config;
const uart_config_t uart2_config;

/******************************* FUNCIONES PÚBLICAS ***************************/

/* ---------- Initialization ---------- */
esp_err_t KM_GPIO_Init(void)
{
    esp_err_t ret;
    printf("GPIO_Init: PWM=%d DIR=%d H2=%d HY1=%d HY2=%d\n",
           PIN_STEER_PWM, PIN_STEER_DIR,
           PIN_MOTOR_HALL_2, PIN_HYDRAULIC_1, PIN_HYDRAULIC_2);

    /* ======================== ADC ======================== */
    // Configurar ADC1/ADC2 pins como entrada
    gpio_config_t adc_pin_cfg = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    // Lista de ADC1
    // PRESSURE_3 is absent on the S3 on purpose: that pad (GPIO 1 / CN5.2) carries
    // the steering sensor's PWM output and belongs to MCPWM capture, which cannot
    // share it with the ADC. See PIN_STEER_PWM_IN in km_gpio.h.
    const gpio_num_t adc1_pins[] = {
        PIN_PEDAL_ACC, PIN_PEDAL_BRAKE, PIN_HYDRAULIC_1,
        PIN_PRESSURE_1, PIN_PRESSURE_2,
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
        PIN_PRESSURE_3,
#endif
    };
    
    adc1_config_width(ADC_WIDTH_BIT_12);

    /* ADC calibration. Raw counts are not volts: the mapping has per-chip offset
     * and gain error, and the "full scale" voltage at 11 dB is neither the 3.3 V
     * rail nor a number worth guessing. Every ESP32 ships calibration data in
     * eFuse, and this turns a raw count into millivolts using it — so downstream
     * code never has to invent a counts-per-volt constant. Characterised once
     * here; KM_GPIO_ReadADC_mV() uses it. */
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                             0 /* no Vref override; eFuse or default is used */,
                             &adc1_chars);
    
    for (int i = 0; i < sizeof(adc1_pins)/sizeof(adc1_pins[0]); i++) {
        adc_pin_cfg.pin_bit_mask = 1ULL << adc1_pins[i];
        ret = gpio_config(&adc_pin_cfg);
        if (ret != ESP_OK) return ret;
        
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        if (adc1_pins[i] == GPIO_NUM_1) adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
        else if (adc1_pins[i] == GPIO_NUM_2) adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
        else if (adc1_pins[i] == GPIO_NUM_4) adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
        else if (adc1_pins[i] == GPIO_NUM_5) adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
        else if (adc1_pins[i] == GPIO_NUM_6) adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11); // PRESSURE_1
        else if (adc1_pins[i] == GPIO_NUM_7) adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
        else if (adc1_pins[i] == GPIO_NUM_10) adc1_config_channel_atten(ADC1_CHANNEL_9, ADC_ATTEN_DB_11);
#else
        if (adc1_pins[i] == GPIO_NUM_36) adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // PRESSURE_1
        else if (adc1_pins[i] == GPIO_NUM_39) adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
        else if (adc1_pins[i] == GPIO_NUM_34) adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
        else if (adc1_pins[i] == GPIO_NUM_35) adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
        else if (adc1_pins[i] == GPIO_NUM_32) adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
#endif
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
    /* ---------- SPI2 → MCP4922 external DAC ---------- */
    /* miso_io_num stays -1 on purpose: the MCP4922 has no data output, so there
     * is nothing on the bus to read back. See the note in km_gpio.h. */
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2
    };
    /* Mode 0 (CPOL=0, CPHA=0) — the MCP4922 accepts 0,0 or 1,1. 1 MHz is far
     * inside the chip's 20 MHz ceiling and keeps the edges kind to a hand-wired
     * board. CS is driven by the driver and must stay low for all 16 bits. */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,
        .mode = 0,
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 1
    };
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_LOGI(SPI_TAG, "spi_bus_initialize(SPI2, mosi=%d sclk=%d) -> %s",
             (int)SPI_MOSI_PIN, (int)SPI_SCLK_PIN, esp_err_to_name(ret));
    if (ret != ESP_OK) return ret;

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &mcp4922_handle);
    ESP_LOGI(SPI_TAG, "spi_bus_add_device(cs=%d, 1 MHz, mode 0) -> %s, handle=%p",
             (int)SPI_CS_PIN, esp_err_to_name(ret), (void *)mcp4922_handle);
    if (ret != ESP_OK) return ret;

    if (mcp4922_handle == NULL) {
        /* Belt and braces: a NULL handle with ESP_OK would mean every later
         * write silently does nothing, which is the failure this whole branch
         * exists to rule out. */
        ESP_LOGE(SPI_TAG, "handle is NULL despite ESP_OK — refusing to continue");
        return ESP_ERR_INVALID_STATE;
    }
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

#ifdef PIN_SELECT_THROTTLE
    /* ================= THROTTLE SOURCE MUX (MAX4660 U14) ================= */
    /* LOW  = COM->NC = the pedal passes through to the kart (safe default).
     * HIGH = COM->NO = the MCP4922's VOUTA drives the throttle instead.
     *
     * R32's 10 k pulldown already holds this LOW before firmware runs, so
     * configuring it here does not change the boot state — it replaces a weak
     * pulldown with a driven level so KM_GPIO_SetThrottleSource() has a pin it
     * can actually assert. control_task() re-decides it every cycle.
     *
     * Until 2026-08-01 nothing drove this pin at all, so the mux never left the
     * pedal and the DAC's output could not reach CN10.1 however well the SPI
     * write worked. Manual-mode safety was handled by zeroing the DAC instead,
     * which is why the gap went unnoticed. */
    gpio_config_t thr_mux_cfg = {
        .pin_bit_mask = 1ULL << PIN_SELECT_THROTTLE,
        /* INPUT_OUTPUT (spi-test-50 diagnostic): keeps the input buffer on so
         * gpio_get_level() reports the level the pin is really driving. */
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   // keep the pedal if the pin floats
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&thr_mux_cfg);
    if (ret != ESP_OK) return ret;
    ret = gpio_set_level(PIN_SELECT_THROTTLE, 0);   // 0 = pedal
    if (ret != ESP_OK) return ret;
#endif

#ifdef PIN_SDC_NOT_EMERGENCY
    /* ==================== SHUTDOWN CIRCUIT — SAFETY ==================== */
    /* Gate of Q3 (IRLZ44N) through R22 100 R. HIGH = Q3 conducts = shutdown
     * chain closed = no emergency. R23's 100 k pulldown already holds it LOW
     * before firmware runs, so driving it LOW here changes nothing about the
     * boot state — it only replaces a weak pulldown with a driven level, so
     * that KM_GPIO_SetEmergency() has a pin it can actually assert.
     *
     * control_task() in main.c re-decides the level every cycle and is the only
     * caller that ever drives it HIGH, so the chain closes only while the Orin
     * reports AS_READY/AS_DRIVING with every interlock passing. Boot therefore
     * still starts in emergency and stays there until that is true.
     *
     * INPUT_OUTPUT, not OUTPUT: the input buffer has to stay enabled for
     * gpio_get_level() to return the level the pin is really driving. A plain
     * OUTPUT pin reads back 0 whatever it is doing, which would make the SDC
     * field in the pneumatic frame a fabricated constant rather than a
     * measurement — and that field is the only way to observe this pin while
     * the gate is not wired to anything. */
    gpio_config_t sdc_cfg = {
        .pin_bit_mask = 1ULL << PIN_SDC_NOT_EMERGENCY,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   // keep the fail-safe if the pin floats
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&sdc_cfg);
    if (ret != ESP_OK) return ret;
    ret = gpio_set_level(PIN_SDC_NOT_EMERGENCY, 0);   // 0 = emergency asserted
    if (ret != ESP_OK) return ret;
#endif

    /* HALL SENSORS — only HALL2 (GPIO33) available; HALL1/3 pins used by UART2 */

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

#ifdef PIN_CMD_COMPRESSOR
    /* ---------- Compressor PWM (LEDC, own timer: different freq to steering) ---------- */
    ledc_timer_config_t comp_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = COMPRESSOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ret = ledc_timer_config(&comp_timer);
    if (ret != ESP_OK) return ret;

    ledc_channel_config_t comp_channel = {
        .gpio_num = (gpio_num_t)PIN_CMD_COMPRESSOR,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0   // compressor off until control_task ramps it up
    };
    ret = ledc_channel_config(&comp_channel);
    if (ret != ESP_OK) return ret;
#endif

    return ESP_OK;
}

/* ---------- Digital GPIO ---------- */
/** @copydoc KM_GPIO_WriteDigital */
esp_err_t KM_GPIO_WriteDigital(gpio_num_t pin, uint8_t level)
{
    return gpio_set_level(pin, level ? 1 : 0);
}

/** @copydoc KM_GPIO_ReadDigital */
uint8_t KM_GPIO_ReadDigital(gpio_num_t pin)
{
    return gpio_get_level(pin);
}

/** @copydoc KM_GPIO_SetEmergency */
esp_err_t KM_GPIO_SetEmergency(uint8_t assert_emergency)
{
#ifdef PIN_SDC_NOT_EMERGENCY
    /* SDC_NOT_EMERGENCY is active-low for emergency: LOW opens the shutdown
     * chain, which is what fires the EBS. */
    return gpio_set_level(PIN_SDC_NOT_EMERGENCY, assert_emergency ? 0 : 1);
#else
    (void)assert_emergency;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

/* ---------- ADC ---------- */
/** @copydoc KM_GPIO_ReadADC */
uint16_t KM_GPIO_ReadADC(gpio_num_t pin)
{
    gpio_num_t gpio = (gpio_num_t)(pin);
    int raw_out_adc2 = 0;

    switch (gpio)
    {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
        // No GPIO_NUM_1 case: that pad is the steering sensor's PWM input, read by
        // MCPWM capture (km_sdir_pwm.h). It falls through to the default and reads 0.
        case GPIO_NUM_2: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_1); // hydraulic 2
        case GPIO_NUM_4: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_3); // pedal acc
        case GPIO_NUM_5: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_4); // pedal brake
        case GPIO_NUM_6: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_5); // pressure 1
        case GPIO_NUM_7: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_6); // pressure 2
        case GPIO_NUM_10: return (uint16_t)adc1_get_raw(ADC1_CHANNEL_9); // hydraulic 1
#else
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
#endif
            
        default: 
            return 0;
    }
}

/** @copydoc KM_GPIO_ReadADC_mV */
uint32_t KM_GPIO_ReadADC_mV(gpio_num_t pin)
{
    /* Converts through the chip's own eFuse calibration rather than through an
     * assumed counts-per-volt. That assumption is what produced the tank-pressure
     * mess of 2026-07-26: the dashboard guessed a 3.3 V full scale, the datasheet
     * says the 11 dB range is 0-2900 mV, and neither is the per-chip truth. With
     * millivolts on the wire the consumer only needs the divider ratio, which is a
     * property of the board and is knowable: PRESSURE_1/2 sit behind three equal
     * 10 k resistors (R11/R12/R13, net PRESSURE_n__0_10V -> PRESSURE_n__0_3V3), so
     * bar = 3 * V_pin for the 1 V/bar Festo SDE5.
     *
     * NOTE the divider maps the sensor's 0-10 V onto 0-3.33 V, but the ADC's usable
     * range stops near 2900 mV, so readings saturate around 8.7 bar. Anything at or
     * above that is out of range, not a pressure. */
    return esp_adc_cal_raw_to_voltage(KM_GPIO_ReadADC(pin), &adc1_chars);
}


/* ---------- DAC ---------- */
#ifdef CONFIG_IDF_TARGET_ESP32S3
/**
 * @brief   Writes one 12-bit code to one MCP4922 channel over SPI2.
 *
 * @param   channel_bit  MCP4922_CH_A or MCP4922_CH_B.
 * @param   value        8-bit level from the actuator layer.
 *
 * @details The caller's 8 bits are widened to the chip's 12 by replicating the
 *          top nibble into the bottom one, so 0x00 maps to 0x000 and 0xFF maps
 *          to 0xFFF. A plain `value << 4` would top out at 0xFF0 and quietly
 *          cost the last 0.4% of travel.
 *
 *          Uses polling_transmit rather than the queued path: a 16-bit word at
 *          1 MHz is ~16 us, so an interrupt round-trip would cost more than the
 *          transfer, and the control loop wants this to be over when it returns.
 */
static esp_err_t mcp4922_write(uint16_t channel_bit, uint8_t value)
{
    if (mcp4922_handle == NULL) {
        ESP_LOGE(SPI_TAG, "write with NULL handle — KM_GPIO_Init() did not run or failed");
        mcp4922_writes_fail++;
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t code = ((uint16_t)value << 4) | (value >> 4);
    uint16_t cmd  = channel_bit | MCP4922_BUF_OFF | MCP4922_GAIN_1X | MCP4922_ACTIVE | (code & 0x0FFFu);

    uint8_t tx[2] = { (uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF) };
    spi_transaction_t t = {
        .length    = 16,        // bits, not bytes
        .tx_buffer = tx,
        .rx_buffer = NULL
    };

    esp_err_t ret = spi_device_polling_transmit(mcp4922_handle, &t);
    mcp4922_last_cmd = cmd;

    if (ret != ESP_OK) {
        mcp4922_writes_fail++;
        ESP_LOGE(SPI_TAG, "transmit failed: %s (cmd=0x%04X)", esp_err_to_name(ret), cmd);
        return ret;
    }

    mcp4922_writes_ok++;
    /* First few writes verbosely, then one line per 500 so a running kart does
     * not drown the console. ESP_LOGD is compiled out at the default level. */
    if (mcp4922_writes_ok <= 5 || (mcp4922_writes_ok % 500) == 0) {
        ESP_LOGI(SPI_TAG, "write #%lu ch%c val=%3u code=%4u cmd=0x%04X expect ~%.2f V",
                 (unsigned long)mcp4922_writes_ok,
                 (channel_bit == MCP4922_CH_A) ? 'A' : 'B',
                 value, (unsigned)(code & 0x0FFF), cmd,
                 5.0 * (double)(code & 0x0FFF) / 4095.0);
    }
    return ESP_OK;
}

/** @copydoc KM_GPIO_McpStats */
void KM_GPIO_McpStats(uint32_t *ok, uint32_t *fail, uint16_t *last_cmd)
{
    if (ok)       *ok       = mcp4922_writes_ok;
    if (fail)     *fail     = mcp4922_writes_fail;
    if (last_cmd) *last_cmd = mcp4922_last_cmd;
}

/** @copydoc KM_GPIO_McpSelfTest */
esp_err_t KM_GPIO_McpSelfTest(void)
{
    /* Safe to run at boot ONLY because SELECT_THROTTLE (GPIO 15) has a 10 kOhm
     * pulldown and nothing drives it: the MAX4660 sits on the pedal
     * pass-through, so channel A's output stops at U14 pin 8 and never reaches
     * CN10.1. If firmware ever starts driving GPIO 15, this must move behind an
     * explicit operator request. Channel B is NOT swept — brake goes through
     * U1A to CN10.2 unmuxed, so a sweep there would command real brake. */
    static const uint8_t steps[] = { 0, 64, 128, 192, 255, 0 };

    ESP_LOGW(SPI_TAG, "self-test: sweeping channel A. Meter U13 pin 14 (VOUTA) to GND.");
    ESP_LOGW(SPI_TAG, "self-test: this does NOT reach the kart — the mux is on pedal pass-through.");

    for (size_t i = 0; i < sizeof(steps); i++) {
        esp_err_t ret = mcp4922_write(MCP4922_CH_A, steps[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(SPI_TAG, "self-test aborted at step %u: %s", (unsigned)i, esp_err_to_name(ret));
            return ret;
        }
        ESP_LOGW(SPI_TAG, "self-test: step %u/%u — expect ~%.2f V, holding 2 s",
                 (unsigned)(i + 1), (unsigned)sizeof(steps), 5.0 * (double)steps[i] / 255.0);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGW(SPI_TAG, "self-test done: %lu ok, %lu failed. If the ESP32 side reports all-ok "
                      "but the meter never moved, the fault is downstream of the MCU: check "
                      "U13 VDD (pin 1) and VREFA/VREFB (pins 13/11) sit at 5 V.",
             (unsigned long)mcp4922_writes_ok, (unsigned long)mcp4922_writes_fail);
    return ESP_OK;
}
#endif /* CONFIG_IDF_TARGET_ESP32S3 */

/** @copydoc KM_GPIO_SetThrottleSource */
esp_err_t KM_GPIO_SetThrottleSource(bool use_dac)
{
#ifdef PIN_SELECT_THROTTLE
    return gpio_set_level(PIN_SELECT_THROTTLE, use_dac ? 1 : 0);
#else
    (void)use_dac;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}


/** @copydoc KM_GPIO_WriteDAC */
esp_err_t KM_GPIO_WriteDAC(gpio_num_t pin, uint8_t value)
{
    gpio_num_t gpio = (gpio_num_t)pin;

    if (gpio == PIN_CMD_ACC) {
#if defined(CONFIG_IDF_TARGET_ESP32)
        return dac_output_voltage(DAC_CHAN_0, value);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
        return mcp4922_write(MCP4922_CH_A, value);
#else
        return ESP_ERR_NOT_SUPPORTED;   // no DAC and no MCP4922 on this target
#endif
    }
    if (gpio == PIN_CMD_BRAKE) {
#if defined(CONFIG_IDF_TARGET_ESP32)
        return dac_output_voltage(DAC_CHAN_1, value);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
        return mcp4922_write(MCP4922_CH_B, value);
#else
        return ESP_ERR_NOT_SUPPORTED;
#endif
    }

    ESP_LOGE(GPIO_TAG, "WriteDAC: unknown channel %d (expected %d=ACC or %d=BRAKE)",
             (int)gpio, (int)PIN_CMD_ACC, (int)PIN_CMD_BRAKE);
    return ESP_ERR_INVALID_ARG;
}

/* ---------- PWM ---------- */
/** @copydoc KM_GPIO_WritePWM */
esp_err_t KM_GPIO_WritePWM(gpio_num_t pin, uint32_t duty)
{
    gpio_num_t gpio = (gpio_num_t)pin;

    if (gpio == PIN_STEER_PWM) // Steering PWM
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
        return ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    }

#ifdef PIN_CMD_COMPRESSOR
    if (gpio == PIN_CMD_COMPRESSOR) // EBS compressor PWM
    {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty);
        return ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    }
#endif

    return ESP_ERR_INVALID_ARG;
}


/******************************* FUNCIONES PRIVADAS ***************************/
/**
 * @brief   Función interna no visible desde fuera
 */
//  void funcion_privada(void);

/******************************* FIN DE ARCHIVO ********************************/
