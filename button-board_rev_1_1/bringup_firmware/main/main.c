#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_app_trace.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_dmx.h"

#define LED_SR_DATA (GPIO_NUM_4)
#define LED_SR_CLOCK (GPIO_NUM_5)
#define LED_SR_LATCH (GPIO_NUM_6)
#define LED_SR_ALL_ON (0x003F3F3F)
#define LED_SR_ALL_OFF (0x00000000)

#define I2C_SCL_1 (GPIO_NUM_1)
#define I2C_SDA_1 (GPIO_NUM_2)
#define I2C_SCL_2 (GPIO_NUM_9)
#define I2C_SDA_2 (GPIO_NUM_10)
#define I2C_EXPECT_ACK (true)
#define AT42QT_I2C_ADDR (0x1C)

#define CAP_TOUCH_CHANGE_0 (GPIO_NUM_7)
#define CAP_TOUCH_CHANGE_1 (GPIO_NUM_8)

#define DMX_TX_PIN (GPIO_NUM_21)
#define DMX_RX_PIN (GPIO_NUM_17)
#define DMX_DE_nRE_PIN (GPIO_NUM_18)

#define TAG "main"

typedef struct {
    uint8_t master_dim;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t w;
    uint8_t strobe;
    uint8_t mode;   // 0-250 Auto mode, 251-255 sound mode
    uint8_t speed;  // 0-250 control speed for mode
} par_dmx_packet_t;

static volatile bool    cap_touch_change_flag;
static volatile uint8_t cap_touch_change_mask;

static void IRAM_ATTR cap_touch_change_isr(void *arg) {
    uint32_t pin = (uint32_t)arg;
    configASSERT(pin == CAP_TOUCH_CHANGE_0 || pin == CAP_TOUCH_CHANGE_1);

    cap_touch_change_flag = true;
    cap_touch_change_mask |= pin;
}

static uint32_t key_status_to_led_mask(uint32_t key_status) {
    uint32_t mask = 0;

    // First 12 bits are cap touch 0, mapped differently than cap touch 1
    for (uint8_t i = 0; i < 12; i++) {
        if (key_status & (1 << i)) {
            mask |= (1 << (13 - i - 2 * (i / 6)));
        }
    }

    // Pull off top 6 bits and shift down to normal byte
    uint8_t cap_touch_1 = (key_status >> 12) & 0xFF;
    for (uint8_t i = 0; i < 8; i++) {
        if (cap_touch_1 & (1 << i)) {
            mask |= (1 << (21 - i));
        }
    }

    return mask;
}

static void led_sr_send(uint32_t mask) {
    gpio_set_level(LED_SR_CLOCK, 0);
    for (uint8_t i = 0; i < 24; i++) {
        gpio_set_level(LED_SR_DATA, (1 << (24 - i - 1) & mask));
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(LED_SR_CLOCK, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(LED_SR_CLOCK, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(LED_SR_LATCH, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(LED_SR_LATCH, 0);
}

static uint16_t cap_touch_0_read_status_bytes() {
    i2c_cmd_handle_t i2c_cmd_handle = {0};
    uint8_t          key_status_0   = 0x00;
    uint8_t          key_status_1   = 0x00;
    uint8_t          detect_status  = 0x00;

    // Write internal register addr of key status byte 2 with write bit
    // Build command within cmd handle - nothing is transmitted at this point
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                          I2C_EXPECT_ACK);                        // addr+wr
    i2c_master_write_byte(i2c_cmd_handle, 0x03, I2C_EXPECT_ACK);  // mem_addr in slave
    i2c_master_stop(i2c_cmd_handle);

    // Transmit constructed cmd sequence
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));

    // Tear down constructed cmd handle for that specific cmd
    i2c_cmd_link_delete(i2c_cmd_handle);

    // Read internal register of key status 0 with read bit
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_READ, I2C_EXPECT_ACK);  // addr+re
    i2c_master_read_byte(i2c_cmd_handle, &key_status_0, I2C_MASTER_LAST_NACK);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    // Write internal register addr of key status byte 1 with write bit
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                          I2C_EXPECT_ACK);                        // addr+wr
    i2c_master_write_byte(i2c_cmd_handle, 0x04, I2C_EXPECT_ACK);  // mem_addr in slave
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    // Read internal register of key status 1 with read bit
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_READ, I2C_EXPECT_ACK);  // addr+re
    i2c_master_read_byte(i2c_cmd_handle, &key_status_1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    // Write internal register addr of detection status with write bit
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                          I2C_EXPECT_ACK);                        // addr+wr
    i2c_master_write_byte(i2c_cmd_handle, 0x02, I2C_EXPECT_ACK);  // mem_addr in slave
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    // Read internal register of detection status with read bit
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_READ, I2C_EXPECT_ACK);  // addr+re
    i2c_master_read_byte(i2c_cmd_handle, &detect_status, I2C_MASTER_LAST_NACK);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    // ESP_LOGI(TAG,
    //          "cap touch 0 (0xK11|..|K0): 0x%03X, Detect status: 0x%02X",
    //          (key_status_1 << 8) | key_status_0,
    //          detect_status);
    return (key_status_1 << 8) | key_status_0;
}

static uint8_t cap_touch_1_read_status_bytes() {
    i2c_cmd_handle_t i2c_cmd_handle = {0};
    uint8_t          key_status_0   = 0x00;
    uint8_t          key_status_1   = 0x00;
    uint8_t          detect_status  = 0x00;

    // key status byte 1
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                          I2C_EXPECT_ACK);                        // addr+wr
    i2c_master_write_byte(i2c_cmd_handle, 0x03, I2C_EXPECT_ACK);  // mem_addr in slave
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_READ, I2C_EXPECT_ACK);  // addr+re
    i2c_master_read_byte(i2c_cmd_handle, &key_status_0, I2C_MASTER_LAST_NACK);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    // key status byte 2
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                          I2C_EXPECT_ACK);                        // addr+wr
    i2c_master_write_byte(i2c_cmd_handle, 0x04, I2C_EXPECT_ACK);  // mem_addr in slave
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_READ, I2C_EXPECT_ACK);  // addr+re
    i2c_master_read_byte(i2c_cmd_handle, &key_status_1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    // detection status byte
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_WRITE,
                          I2C_EXPECT_ACK);                        // addr+wr
    i2c_master_write_byte(i2c_cmd_handle, 0x02, I2C_EXPECT_ACK);  // mem_addr in slave
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);
    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (AT42QT_I2C_ADDR << 1) | I2C_MASTER_READ, I2C_EXPECT_ACK);  // addr+re
    i2c_master_read_byte(i2c_cmd_handle, &detect_status, I2C_MASTER_LAST_NACK);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_1, i2c_cmd_handle, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(i2c_cmd_handle);

    uint8_t key_status = ((key_status_0 & 0xC0) >> 6) | (key_status_1 << 2);
    // ESP_LOGI(TAG, "cap touch 1 (0xK11|..|K0): 0x%02X, Detect status: 0x%02X", key_status, detect_status);
    return key_status;
}

static void system_init(void) {
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
}

static void system_start(void) {
}

static void app_init() {
    cap_touch_change_flag = false;
    cap_touch_change_mask = 0x00;

    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << LED_SR_DATA) | (1ULL << LED_SR_CLOCK) | (1ULL << LED_SR_LATCH),
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = false,
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&output_config));

    gpio_config_t input_interrupt_config = {
        .pin_bit_mask = (1ULL << CAP_TOUCH_CHANGE_0) | (1ULL << CAP_TOUCH_CHANGE_1),
        .intr_type    = GPIO_INTR_NEGEDGE,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = false,  // pulled up externally
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&input_interrupt_config));
    gpio_install_isr_service(0x0);
    gpio_isr_handler_add(CAP_TOUCH_CHANGE_0, cap_touch_change_isr, (void *)CAP_TOUCH_CHANGE_0);
    gpio_isr_handler_add(CAP_TOUCH_CHANGE_1, cap_touch_change_isr, (void *)CAP_TOUCH_CHANGE_1);

    i2c_config_t i2c_config = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_1,
        .sda_pullup_en    = GPIO_PULLUP_DISABLE,
        .scl_io_num       = I2C_SCL_1,
        .scl_pullup_en    = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000,
        .clk_flags        = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    i2c_config.sda_io_num    = I2C_SDA_2;
    i2c_config.scl_io_num    = I2C_SCL_2;
    i2c_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_1, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0));

    const int         personality_count = 1;
    dmx_personality_t personalities[]   = {{1, "Default Personality"}};

    const dmx_port_t dmx_num = DMX_NUM_1;
    dmx_config_t     config  = DMX_CONFIG_DEFAULT;
    dmx_driver_install(dmx_num, &config, personalities, personality_count);
    dmx_set_pin(dmx_num, DMX_TX_PIN, DMX_RX_PIN, DMX_DE_nRE_PIN);
}

static void app_start(void) {
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting cap-touch-button-board firmware");

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    system_init();
    system_start();

    app_init();
    app_start();

    // LED init flashes
    uint32_t mask    = 0x000000;
    uint8_t  bit_pos = 0;
    uint8_t  i       = 0;
    while (i < 3) {
        mask = (1 << bit_pos);
        led_sr_send(mask);

        bit_pos++;
        if (bit_pos == 6) {
            bit_pos = 8;
        } else if (bit_pos == 14) {
            bit_pos = 16;
        } else if (bit_pos >= 22) {
            bit_pos = 0;
            i++;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    mask = LED_SR_ALL_ON;
    led_sr_send(mask);
    vTaskDelay(pdMS_TO_TICKS(1000));
    mask = LED_SR_ALL_OFF;
    led_sr_send(mask);
    mask = 1;

    cap_touch_0_read_status_bytes();
    cap_touch_1_read_status_bytes();

    uint32_t         key_status                 = 0x0000;
    par_dmx_packet_t current_packet             = {0};
    uint8_t          start_byte                 = 0x00;
    uint8_t          par_mask                   = 0x00;
    uint8_t          last_par_mask              = 0x00;
    uint8_t          key_status_bottom_row      = 0x00;
    uint8_t          last_key_status_bottom_row = 0x00;
    while (1) {
        if (cap_touch_change_flag) {
            cap_touch_change_flag = false;
            key_status            = 0;

            if (cap_touch_change_mask | CAP_TOUCH_CHANGE_0) {
                cap_touch_change_mask &= ~CAP_TOUCH_CHANGE_0;
                key_status |= cap_touch_0_read_status_bytes();
            }

            if (cap_touch_change_mask | CAP_TOUCH_CHANGE_1) {
                cap_touch_change_mask &= ~CAP_TOUCH_CHANGE_1;
                uint8_t temp = cap_touch_1_read_status_bytes();
                key_status |= (temp << 12);
            }

            // Hack around shorted led <-> cap touch pad triggering constantly. If this is the only change after we're
            // triggered, ignore and continue
            if (key_status == (1 << 2) || key_status == ((1 << 2) | (1 << 1))) {
                continue;
            }

            mask = key_status_to_led_mask(key_status);
            led_sr_send(mask);
        }

        key_status_bottom_row = (key_status >> 6) & 0xFF;
        if (key_status_bottom_row == 0) {
            par_mask = last_par_mask;
        } else {
            ESP_LOGI(TAG, "this: 0x%02X - last: 0x%02X", key_status_bottom_row, last_key_status_bottom_row);
            last_key_status_bottom_row &= key_status_bottom_row;
            key_status_bottom_row ^= last_key_status_bottom_row;
            if (key_status_bottom_row == 0) {
                par_mask = last_par_mask;
            } else {
                switch (key_status_bottom_row) {
                    case 1 << 0:
                        par_mask = 0xFF;
                        break;
                    case 1 << 1:
                        par_mask = (0xFF / 5) * 4;
                        break;
                    case 1 << 2:
                        par_mask = (0xFF / 5) * 3;
                        break;
                    case 1 << 3:
                        par_mask = (0xFF / 5) * 2;
                        break;
                    case 1 << 4:
                        par_mask = 0xFF / 5;
                        break;
                    case 1 << 5:
                        par_mask = 0x00;
                        break;
                    default:
                        // Means two bits have been enabled since the last loop
                        ESP_LOGI(TAG, "Unhandled bottom row val: 0x%02X", key_status_bottom_row);
                        break;
                }
            }
        }

        last_key_status_bottom_row = key_status_bottom_row;

        current_packet.master_dim = par_mask;
        current_packet.r          = 0xFF;
        current_packet.g          = 0xFF;
        current_packet.b          = 0xFF;
        current_packet.w          = 0xFF;
        current_packet.strobe     = 0;
        current_packet.mode       = 0;
        current_packet.speed      = 0;

        dmx_write(DMX_NUM_1, &start_byte, 1);
        dmx_write_offset(DMX_NUM_1, 0x01, &current_packet, sizeof(par_dmx_packet_t));
        dmx_send(DMX_NUM_1);

        last_par_mask = par_mask;

        /*
        // Test DMX output sending RGB sequence
        switch (color) {
            case 0:
                r     = 0xFF;
                b     = 0x00;
                g     = 0x00;
                color = 1;
                break;
            case 1:
                r     = 0x00;
                b     = 0xFF;
                g     = 0x00;
                color = 2;
                break;
            case 2:
                r     = 0x00;
                b     = 0x00;
                g     = 0xFF;
                color = 0;
                break;
            default:
                configASSERT(0);
        }

        for (uint16_t i = 0; i < 510; i += 3) {
            dmx_buffer[i]     = r;
            dmx_buffer[i + 1] = g;
            dmx_buffer[i + 2] = b;
        }

        dmx_write(DMX_NUM_1, &start_addr, 1);
        dmx_write_offset(DMX_NUM_1, 0x1, dmx_buffer, sizeof(dmx_buffer));
        dmx_send(DMX_NUM_1);
        vTaskDelay(pdMS_TO_TICKS(50));
        */
    }
}
