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
#include "esp_sleep.h"
#include "esp_pm.h"
#include <platform/ConnectivityManager.h>
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
#include <openthread/thread.h>
#include <openthread/link.h>
#endif

#include <platform/CHIPDeviceConfig.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0 

using namespace esp_matter;
using namespace esp_matter::endpoint;
using namespace esp_matter::cluster;
using namespace chip::app::Clusters;

static const char *TAG = "CONTACT_SENSOR";

/* GPIOs */
#define CONTACT_LOW_PIN   GPIO_NUM_4
#define CONTACT_FULL_PIN  GPIO_NUM_5
#define RESET_BUTTON_PIN  GPIO_NUM_9
#define COMM_LED_GPIO     GPIO_NUM_13

/* Endpoint IDs */
static uint16_t low_ep_id  = 0;
static uint16_t full_ep_id = 0;
static bool commissioning_active = false;

/* --- GLOBAL HANDLES --- */
static TaskHandle_t hardware_task_handle = NULL;
static TaskHandle_t led_task_handle = NULL;

/* --- ISR HANDLER (Must be ABOVE hardware_task) --- */
static void IRAM_ATTR contact_sensor_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(hardware_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}


// This variable survives Deep Sleep on the ESP32-H2
static RTC_DATA_ATTR float smoothed_bat_mv = 0.0f; 


/* ========================================================================
 * LED FUNCTIONS
 * ======================================================================== */
static void led_init() {
    gpio_reset_pin(COMM_LED_GPIO);
    gpio_set_direction(COMM_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(COMM_LED_GPIO, 0);
}

static void led_set(bool on) {
    gpio_set_level(COMM_LED_GPIO, on ? 1 : 0);
}

static void commissioning_led_task(void *arg) {
    bool state = false;
    while (commissioning_active) { // Only run while pairing
        led_set(state);
        state = !state;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    led_set(false); // Ensure OFF
    led_task_handle = NULL;
    vTaskDelete(NULL); // Kill the task to save RAM and Power
}

/* ========================================================================
 * HELPER FUNCTIONS
 * ======================================================================== */
static void open_commissioning_window_if_necessary() {
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
        chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg) {
            chip::Server::GetInstance().GetCommissioningWindowManager()
                .OpenBasicCommissioningWindow();
        }, 0);
    }
}

/* ========================================================================
 * MATTER CALLBACKS
 * ======================================================================== */
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

/* ========================================================================
 * BATTERY TASK - Optimized for long battery life
 * ======================================================================== */


static void battery_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(15000)); // Matter stability delay
    
    while (true) {
        adc_oneshot_unit_handle_t adc_handle;
        adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
        adc_oneshot_new_unit(&init_config, &adc_handle);
        
        adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN_DB_6, .bitwidth = ADC_BITWIDTH_DEFAULT };
        adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL, &config);
        
        adc_cali_handle_t cali_handle = NULL;
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_6, .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
        
        int32_t raw_acc = 0;
        int count = 0;
        for (int i = 0; i < 15; i++) {
            int val;
            if (adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &val) == ESP_OK) {
                if (i >= 5) { raw_acc += val; count++; }
            }
            vTaskDelay(pdMS_TO_TICKS(150));
        }
        
        int pin_mv;
        adc_cali_raw_to_voltage(cali_handle, (raw_acc / count), &pin_mv);
        
        // 1. FINAL CALIBRATED MULTIPLIER (1730)
        uint32_t current_reading_mv = (pin_mv * 1730) / 1000;

        // 2. EMA FILTER (Saves to RTC Memory)
        if (smoothed_bat_mv < 2000.0f || smoothed_bat_mv > 4500.0f) {
            smoothed_bat_mv = (float)current_reading_mv;
        } else {
            smoothed_bat_mv = (current_reading_mv * 0.1f) + (smoothed_bat_mv * 0.9f);
        }
        
        uint32_t battery_mv = (uint32_t)smoothed_bat_mv;
        
        // 3. FINAL PERCENTAGE LOGIC (3150mV = 100%, 2600mV = 0%)
        uint8_t percentage;
        if (battery_mv >= 3150) {
            percentage = 200; 
        } else if (battery_mv <= 2600) {
            percentage = 0;
        } else {
            // Range is 550mV (3150 - 2600)
            percentage = (uint8_t)((battery_mv - 2600) * 200 / 550);
        }
        
        adc_cali_delete_scheme_curve_fitting(cali_handle);
        adc_oneshot_del_unit(adc_handle);
        
        chip::DeviceLayer::SystemLayer().ScheduleLambda([battery_mv, percentage]() {
            esp_matter_attr_val_t v_v = esp_matter_nullable_uint32(battery_mv);
            esp_matter_attr_val_t v_p = esp_matter_nullable_uint8(percentage);
            attribute::report(low_ep_id, PowerSource::Id, PowerSource::Attributes::BatVoltage::Id, &v_v);
            attribute::report(low_ep_id, PowerSource::Id, PowerSource::Attributes::BatPercentRemaining::Id, &v_p);
            ESP_LOGI("BATT", "Reported: %lu mV (%d%%)", battery_mv, percentage / 2);
        });
        
        vTaskDelay(pdMS_TO_TICKS(86400000)); // Daily check
    }
}

/* ========================================================================
 * HARDWARE TASK - Contact Sensors + Factory Reset
 * ======================================================================== */
static void hardware_task(void *arg) {
    hardware_task_handle = xTaskGetCurrentTaskHandle();

    // 1. Setup GPIO Interrupts
    gpio_install_isr_service(0);
    gpio_set_intr_type(CONTACT_LOW_PIN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(CONTACT_FULL_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(CONTACT_LOW_PIN, contact_sensor_isr_handler, NULL);
    gpio_isr_handler_add(CONTACT_FULL_PIN, contact_sensor_isr_handler, NULL);

    // 2. Boot Delay (Prevents Watchdog issues during startup)
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    // 3. Initial Sync: Report current state to Matter on boot
    bool initial_low = (gpio_get_level(CONTACT_LOW_PIN) == 0);
    bool initial_full = (gpio_get_level(CONTACT_FULL_PIN) == 0);
    
    chip::DeviceLayer::SystemLayer().ScheduleLambda([initial_low, initial_full]() {
        esp_matter_attr_val_t v1 = esp_matter_bool(initial_low);
        attribute::report(low_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v1);
        
        esp_matter_attr_val_t v2 = esp_matter_bool(initial_full);
        attribute::report(full_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v2);
        
        ESP_LOGI("BOOT", "Sync: Low=%d Full=%d", initial_low, initial_full);
    });

        while (true) {
        // 4. Sleep until the ISR sends a notification (3.6uA mode)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

        // 5. Short 50ms Blue Flash on movement
        led_set(true);                  
        vTaskDelay(pdMS_TO_TICKS(50)); // Reduced to 50ms to save battery
        led_set(false);                 

        // 6. Short debounce delay
        vTaskDelay(pdMS_TO_TICKS(30));

        // 7. Report the change to the Matter Hub
        bool cur_low = (gpio_get_level(CONTACT_LOW_PIN) == 0);
        bool cur_full = (gpio_get_level(CONTACT_FULL_PIN) == 0);

        chip::DeviceLayer::SystemLayer().ScheduleLambda([cur_low, cur_full]() {
            esp_matter_attr_val_t v_l = esp_matter_bool(cur_low);
            attribute::report(low_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v_l);
            
            esp_matter_attr_val_t v_f = esp_matter_bool(cur_full);
            attribute::report(full_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v_f);
        });
    }

}

/* ========================================================================
 * SLEEP & POWER MANAGEMENT
 * ======================================================================== */
void configure_sed_parameters() {
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    // ICD handles polling automatically via sdkconfig
    // Just set the device type
    chip::DeviceLayer::ConnectivityMgr().SetThreadDeviceType(
        chip::DeviceLayer::ConnectivityManager::kThreadDeviceType_SleepyEndDevice
    );
    
    ESP_LOGI(TAG, "ICD Mode Enabled: Slow Poll=30s, LIT support active");
#endif
}

void init_power_management() {
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 96,
        .min_freq_mhz = 32, // Increased from 8 to 32 for stability
        .light_sleep_enable = true
    };
    // Use ESP_LOG instead of CHECK to prevent rebooting on error
    esp_err_t err = esp_pm_configure(&pm_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config PM (%d)", err);
    }
}

void setup_gpio_wakeup() {
    // 1. Configure the pins for wakeup when pulled LOW (magnet present)
    // On H2, we use gpio_wakeup_enable for both Light and Deep sleep
    ESP_ERROR_CHECK(gpio_wakeup_enable(CONTACT_LOW_PIN, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable(CONTACT_FULL_PIN, GPIO_INTR_LOW_LEVEL));

    // 2. Enable the global GPIO wakeup source
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

    ESP_LOGI("SLEEP", "H2 Wakeup configured for Pins 4 & 5");
}



/* ========================================================================
 * MAIN APPLICATION
 * ======================================================================== */
extern "C" void app_main() {
    // 1. Initialize NVS and partitions
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    nvs_flash_init_partition("fctry");

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    esp_vfs_eventfd_config_t eventfd_config = ESP_VFS_EVENTD_CONFIG_DEFAULT();
    esp_vfs_eventfd_register(&eventfd_config);
    esp_openthread_platform_config_t ot_config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config  = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&ot_config);
#endif

    // 2. GPIO Setup (Pull-ups DISABLED for 470k resistors)
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << CONTACT_LOW_PIN) | (1ULL << CONTACT_FULL_PIN) | (1ULL << RESET_BUTTON_PIN);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io);

    led_init();
    setup_gpio_wakeup();

    // 3. Create Node and Endpoints
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, NULL);

    contact_sensor::config_t sensor_config; 
    endpoint_t *low_ep = contact_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, NULL);
    low_ep_id = endpoint::get_id(low_ep);

    endpoint_t *full_ep = contact_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, NULL);
    full_ep_id = endpoint::get_id(full_ep);

  // 4. Power Source Cluster
    cluster_t *ps_cluster = cluster::create(low_ep, PowerSource::Id, CLUSTER_FLAG_SERVER);
    if (ps_cluster) {
        cluster::global::attribute::create_feature_map(ps_cluster, 0x02);
        cluster::power_source::attribute::create_bat_charge_level(ps_cluster, 1);
        
        // Fix: Provide (Initial Value, Min, Max) as expected by your SDK version
        // Voltage: Initial 3200mV, Min 0mV, Max 65535mV
        cluster::power_source::attribute::create_bat_voltage(
            ps_cluster, 
            nullable<uint32_t>(3200), 
            nullable<uint32_t>(0), 
            nullable<uint32_t>(65535)
        );
        
        // Percent: Initial 200 (100%), Min 0, Max 200
        cluster::power_source::attribute::create_bat_percent_remaining(
            ps_cluster, 
            nullable<uint8_t>(200), 
            nullable<uint8_t>(0), 
            nullable<uint8_t>(200)
        );
        
        ESP_LOGI("APP", "Power Source Cluster created successfully");
    }

    // 5. Start Matter Stack
    esp_matter::start(app_event_cb);

    // Give the data model a moment to stabilise internally
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 6. Commissioning and Power Logic
    bool is_commissioned = (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0);
    
    if (is_commissioned) {
        commissioning_active = false;
        chip::DeviceLayer::SystemLayer().ScheduleLambda([]() {
            configure_sed_parameters(); 
            init_power_management();
        });
    } else {
        commissioning_active = true;
        xTaskCreate(commissioning_led_task, "comm_led", 2048, NULL, 5, &led_task_handle);
    }

    // 7. Start Hardware and Battery Tasks (Increased stacks for H2/Matter)
    xTaskCreate(hardware_task, "hw", 4096, NULL, 5, &hardware_task_handle);
    xTaskCreate(battery_task, "bat", 5120, NULL, 2, NULL);
}

