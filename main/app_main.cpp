/* --------------------------------------------------------------------------
 * Contact sensor with two endpoints
 * -------------------------------------------------------------------------- */
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
#define RESET_BUTTON_PIN  GPIO_NUM_9

/* Endpoint IDs */
static uint16_t low_ep_id = 0;

/* --------------------------------------------------------------------------
 * HELPERS
 * -------------------------------------------------------------------------- */
static void open_commissioning_window_if_necessary() {
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
        chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg) {
            chip::Server::GetInstance().GetCommissioningWindowManager().OpenBasicCommissioningWindow();
        }, 0);
    }
}

/* --------------------------------------------------------------------------
 * MATTER CALLBACKS
 * -------------------------------------------------------------------------- */
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data) {
    return ESP_OK;
}

static void app_event_cb(const chip::DeviceLayer::ChipDeviceEvent *event, intptr_t arg) {
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;
    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed");
        break;
    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        ESP_LOGI(TAG, "Fabric removed");
        open_commissioning_window_if_necessary();
        break;
    default:
        break;
    }
}

/* --------------------------------------------------------------------------
 * HARDWARE POLLING TASK
 * -------------------------------------------------------------------------- */
static void hardware_task(void *arg) {
    bool last_low = (gpio_get_level(CONTACT_LOW_PIN) == 0);
    TickType_t reset_start = 0;

    while (true) {
        if (gpio_get_level(RESET_BUTTON_PIN) == 0) {
            if (reset_start == 0) reset_start = xTaskGetTickCount();
            else if ((xTaskGetTickCount() - reset_start) > pdMS_TO_TICKS(5000)) {
                nvs_flash_erase();
                esp_restart();
            }
        } else reset_start = 0;

        bool cur_low = (gpio_get_level(CONTACT_LOW_PIN) == 0);
        if (cur_low != last_low) {
            last_low = cur_low;
            chip::DeviceLayer::SystemLayer().ScheduleLambda([cur_low]() {
                esp_matter_attr_val_t v = esp_matter_bool(cur_low);
                attribute::update(low_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v);
                attribute::report(low_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v);
            });
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* --------------------------------------------------------------------------
 * MAIN APPLICATION
 * -------------------------------------------------------------------------- */
extern "C" void app_main() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* FIX: Correct VFS Macro for 2026 */
    esp_vfs_eventfd_config_t eventfd_config = ESP_VFS_EVENTD_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));

    /* FIX: Use standard ESP-IDF initialization values for Thread */
    esp_openthread_platform_config_t ot_config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&ot_config);
#endif

    gpio_config_t io = {.pin_bit_mask = (1ULL << CONTACT_LOW_PIN) | (1ULL << RESET_BUTTON_PIN),
                        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE};
    gpio_config(&io);

    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, NULL);

    contact_sensor::config_t low_cfg;
    endpoint_t *low_ep = contact_sensor::create(node, &low_cfg, ENDPOINT_FLAG_NONE, NULL);
    low_ep_id = endpoint::get_id(low_ep);

    esp_matter::start(app_event_cb);
    xTaskCreate(hardware_task, "hw_task", 4096, NULL, 5, NULL);
}
