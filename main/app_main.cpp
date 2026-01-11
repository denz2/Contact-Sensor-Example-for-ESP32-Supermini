/* Double Contact Sensor Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_vfs_eventfd.h"

#include <esp_matter.h>
#include <esp_matter_core.h>
#include <esp_matter_endpoint.h>
#include <esp_matter_cluster.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>
#include <platform/CHIPDeviceLayer.h>

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#include <esp_openthread.h>
#include <esp_openthread_types.h>
#include <app_openthread_config.h>
#endif

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace chip::app::Clusters;

static const char *TAG = "CONTACT_SENSOR";

/* GPIOs */
#define CONTACT_LOW_PIN   GPIO_NUM_4
#define CONTACT_FULL_PIN  GPIO_NUM_5
#define RESET_BUTTON_PIN  GPIO_NUM_9

/* Commissioning LED (ESP32-H2 onboard blue LED) */
#define COMM_LED_GPIO GPIO_NUM_13

/* Endpoint IDs */
static uint16_t low_ep_id  = 0;
static uint16_t full_ep_id = 0;

/* Commissioning LED state */
static bool commissioning_active = true;

/* --------------------------------------------------------------------------
 * LED HELPERS
 * -------------------------------------------------------------------------- */
static void led_init()
{
    gpio_reset_pin(COMM_LED_GPIO);
    gpio_set_direction(COMM_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(COMM_LED_GPIO, 0);   // LED off
}

static void led_set(bool on)
{
    gpio_set_level(COMM_LED_GPIO, on ? 1 : 0);
}

static void commissioning_led_task(void *arg)
{
    bool state = false;

    while (true) {
        if (commissioning_active) {
            led_set(state);
            state = !state;
            vTaskDelay(pdMS_TO_TICKS(200));   // fast blink
        } else {
            led_set(false);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/* --------------------------------------------------------------------------
 * HELPERS
 * -------------------------------------------------------------------------- */
static void open_commissioning_window_if_necessary() {
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
        chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg) {
            chip::Server::GetInstance().GetCommissioningWindowManager()
                .OpenBasicCommissioningWindow();
        }, 0);
    }
}

/* --------------------------------------------------------------------------
 * MATTER CALLBACKS
 * -------------------------------------------------------------------------- */
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type,
                                         uint16_t endpoint_id,
                                         uint32_t cluster_id,
                                         uint32_t attribute_id,
                                         esp_matter_attr_val_t *val,
                                         void *priv_data) {
    return ESP_OK;
}

static void app_event_cb(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg) {
    switch (event->Type) {

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        commissioning_active = false;
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        commissioning_active = true;
        ESP_LOGI(TAG, "Commissioning failed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        commissioning_active = true;
        ESP_LOGI(TAG, "Fabric removed");
        open_commissioning_window_if_necessary();
        break;

    default:
        break;
    }
}

/* --------------------------------------------------------------------------
 * HARDWARE POLLING TASK (LOW + FULL debounce)
 * -------------------------------------------------------------------------- */
static void hardware_task(void *arg)
{
    bool last_low  = (gpio_get_level(CONTACT_LOW_PIN)  == 0);
    bool last_full = (gpio_get_level(CONTACT_FULL_PIN) == 0);
    TickType_t reset_start = 0;

    const TickType_t debounce_ticks = pdMS_TO_TICKS(50);

    while (true) {

        /* Factory reset */
        if (gpio_get_level(RESET_BUTTON_PIN) == 0) {
            if (reset_start == 0) {
                reset_start = xTaskGetTickCount();
            } else if ((xTaskGetTickCount() - reset_start) > pdMS_TO_TICKS(5000)) {
                ESP_LOGW(TAG, "Factory reset triggered");
                nvs_flash_erase();
                esp_restart();
            }
        } else {
            reset_start = 0;
        }

        /* LOW contact with debounce */
        bool raw_low = (gpio_get_level(CONTACT_LOW_PIN) == 0);
        if (raw_low != last_low) {
            vTaskDelay(debounce_ticks);
            bool confirm_low = (gpio_get_level(CONTACT_LOW_PIN) == 0);

            if (confirm_low == raw_low) {
                last_low = confirm_low;

                chip::DeviceLayer::SystemLayer().ScheduleLambda([confirm_low]() {
                    esp_matter_attr_val_t v = esp_matter_bool(confirm_low);
                    attribute::update(low_ep_id,
                                      BooleanState::Id,
                                      BooleanState::Attributes::StateValue::Id,
                                      &v);
                    attribute::report(low_ep_id,
                                      BooleanState::Id,
                                      BooleanState::Attributes::StateValue::Id,
                                      &v);
                });
            }
        }

        /* FULL contact with debounce */
        bool raw_full = (gpio_get_level(CONTACT_FULL_PIN) == 0);
        if (raw_full != last_full) {
            vTaskDelay(debounce_ticks);
            bool confirm_full = (gpio_get_level(CONTACT_FULL_PIN) == 0);

            if (confirm_full == raw_full) {
                last_full = confirm_full;

                chip::DeviceLayer::SystemLayer().ScheduleLambda([confirm_full]() {
                    esp_matter_attr_val_t v = esp_matter_bool(confirm_full);
                    attribute::update(full_ep_id,
                                      BooleanState::Id,
                                      BooleanState::Attributes::StateValue::Id,
                                      &v);
                    attribute::report(full_ep_id,
                                      BooleanState::Id,
                                      BooleanState::Attributes::StateValue::Id,
                                      &v);
                });
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* --------------------------------------------------------------------------
 * MAIN APPLICATION
 * -------------------------------------------------------------------------- */
extern "C" void app_main()
{
    /* NVS init */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    esp_vfs_eventfd_config_t eventfd_config = ESP_VFS_EVENTD_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    esp_openthread_platform_config_t ot_config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&ot_config);
#endif

    /* GPIO config */
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << CONTACT_LOW_PIN) |
                      (1ULL << CONTACT_FULL_PIN) |
                      (1ULL << RESET_BUTTON_PIN);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io);

    /* LED setup */
    led_init();
    xTaskCreate(commissioning_led_task, "commission_led", 2048, NULL, 5, NULL);

    /* Matter node */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, NULL);

    /* LOW endpoint */
    contact_sensor::config_t low_cfg;
    endpoint_t *low_ep = contact_sensor::create(node, &low_cfg, ENDPOINT_FLAG_NONE, NULL);
    low_ep_id = endpoint::get_id(low_ep);

    /* FULL endpoint */
    contact_sensor::config_t full_cfg;
    endpoint_t *full_ep = contact_sensor::create(node, &full_cfg, ENDPOINT_FLAG_NONE, NULL);
    full_ep_id = endpoint::get_id(full_ep);

    /* Start Matter + tasks */
    esp_matter::start(app_event_cb);
    xTaskCreate(hardware_task, "hw_task", 4096, NULL, 5, NULL);
}



