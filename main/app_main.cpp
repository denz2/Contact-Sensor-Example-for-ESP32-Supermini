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
TaskHandle_t led_task_handle = NULL;

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

uint8_t calculate_percentage(uint32_t voltage_mv) {
    // CR123A: 3000mV (full) to 2500mV (empty)
    const uint32_t max_mv = 3000;
    const uint32_t min_mv = 2500;
    
    if (voltage_mv > max_mv) return 200;  // 100% in Matter (0.5% steps)
    if (voltage_mv < min_mv) return 0;    // 0%
    
    return (uint8_t)(((voltage_mv - min_mv) * 200) / (max_mv - min_mv));
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
 * HARDWARE TASK - Contact Sensors + Factory Reset
 * ======================================================================== */
// 1. Define a handle for the task
static TaskHandle_t hardware_task_handle = NULL;

// 2. Simple ISR (Interrupt Service Routine)
static void IRAM_ATTR contact_sensor_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(hardware_task_handle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

static void hardware_task(void *arg) {
    hardware_task_handle = xTaskGetCurrentTaskHandle();

    // 1. Initial Setup
    gpio_install_isr_service(0);
    gpio_set_intr_type(CONTACT_LOW_PIN, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(CONTACT_FULL_PIN, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(CONTACT_LOW_PIN, contact_sensor_isr_handler, NULL);
    gpio_isr_handler_add(CONTACT_FULL_PIN, contact_sensor_isr_handler, NULL);

    // 2. Wait for stack to be stable before first report
    vTaskDelay(pdMS_TO_TICKS(10000));
    
      // Initial Sync
    bool initial_low = (gpio_get_level(CONTACT_LOW_PIN) == 0);
    bool initial_full = (gpio_get_level(CONTACT_FULL_PIN) == 0);
    
    chip::DeviceLayer::SystemLayer().ScheduleLambda([initial_low, initial_full]() {
        // Create the value first
        esp_matter_attr_val_t v1 = esp_matter_bool(initial_low);
        attribute::report(low_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v1);
        
        esp_matter_attr_val_t v2 = esp_matter_bool(initial_full);
        attribute::report(full_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v2);
    });

    while (true) {
        // 3. BLOCK (This is where the CPU sleeps at 3.6uA)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

        // Debounce & Flash
        gpio_set_level((gpio_num_t)COMM_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); 
        gpio_set_level((gpio_num_t)COMM_LED_GPIO, 0);

        // Define BOTH variables here so they are "in scope" for the log below
        bool cur_low = (gpio_get_level(CONTACT_LOW_PIN) == 0);
        bool cur_full = (gpio_get_level(CONTACT_FULL_PIN) == 0);

        chip::DeviceLayer::SystemLayer().ScheduleLambda([cur_low, cur_full]() {
            esp_matter_attr_val_t v_l = esp_matter_bool(cur_low);
            attribute::report(low_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v_l);
            
            esp_matter_attr_val_t v_f = esp_matter_bool(cur_full);
            attribute::report(full_ep_id, BooleanState::Id, BooleanState::Attributes::StateValue::Id, &v_f);
        });
        
        // Use cur_low and cur_full (NOT raw_low/raw_full)
        ESP_LOGI("HW", "Device Woke Up & Reported: Low=%d Full=%d", (int)cur_low, (int)cur_full);
    }
}

/* ========================================================================
 * BATTERY TASK - Voltage + Percentage Monitoring
 * ======================================================================== */
static void battery_task(void *pvParameters) {
    // Wait for initial stability
    vTaskDelay(pdMS_TO_TICKS(10000));
    
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config, &adc_handle);
    adc_oneshot_chan_cfg_t config = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &config);

    while (true) {
        int val = 0, raw_adc = 0;
        // NO DELAY LOOP: Keeps CPU awake for only ~1ms
        for (int i = 0; i < 10; i++) {
            adc_oneshot_read(adc_handle, ADC_CHANNEL_0, &val);
            raw_adc += val;
        }
        raw_adc /= 10;

        // Math for CR123A (3.0V nominal) with 100k/100k divider
        uint32_t battery_mv = (raw_adc * 2450 / 4095) * 2; 
        uint8_t percentage = (battery_mv >= 3100) ? 200 : (battery_mv <= 2600) ? 0 : (uint8_t)((battery_mv - 2600) * 200 / 500);

        chip::DeviceLayer::SystemLayer().ScheduleLambda([battery_mv, percentage]() {
            esp_matter_attr_val_t v_v = esp_matter_nullable_uint32(battery_mv);
            attribute::update(low_ep_id, PowerSource::Id, PowerSource::Attributes::BatVoltage::Id, &v_v);
            esp_matter_attr_val_t v_p = esp_matter_nullable_uint8(percentage);
            attribute::update(low_ep_id, PowerSource::Id, PowerSource::Attributes::BatPercentRemaining::Id, &v_p);
        });
      vTaskDelay(pdMS_TO_TICKS(43200000)); // Check every 12 hours
    }
}
/* ========================================================================
 * REFRESH ATTRIBUTES ON ENDPOINTS AT START
 * ======================================================================== */
static void sync_sensor_state_at_boot() {
    // Read physical state: 0 = Closed (Low), 1 = Open (High)
    bool is_low_closed = (gpio_get_level((gpio_num_t)CONTACT_LOW_PIN) == 0);
    bool is_full_closed = (gpio_get_level((gpio_num_t)CONTACT_FULL_PIN) == 0);

    // Update Matter attributes immediately
    esp_matter_attr_val_t val = esp_matter_bool(is_low_closed);
    esp_matter::attribute::update(low_ep_id, chip::app::Clusters::BooleanState::Id, 
                                  chip::app::Clusters::BooleanState::Attributes::StateValue::Id, &val);

    val = esp_matter_bool(is_full_closed);
    esp_matter::attribute::update(full_ep_id, chip::app::Clusters::BooleanState::Id, 
                                  chip::app::Clusters::BooleanState::Attributes::StateValue::Id, &val);
    
    ESP_LOGI("BOOT", "Initial Sync: Low=%s, Full=%s", 
             is_low_closed ? "Closed" : "Open", is_full_closed ? "Closed" : "Open");
}
		
            
       

/* ========================================================================
 * SLEEP & POWER MANAGEMENT
 * ======================================================================== */
void configure_sed_parameters() {
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    // Note: Device type is set via sdkconfig (CONFIG_OPENTHREAD_MTD=y)
    // Polling interval matches sdkconfig setting
    uint32_t idleIntervalMS = 15000;  // 15 seconds (max without LIT support)
    chip::DeviceLayer::ConnectivityMgr().SetPollingInterval(
        chip::System::Clock::Milliseconds32(idleIntervalMS)
    );
    ESP_LOGI(TAG, "Thread polling interval set to %lu ms", idleIntervalMS);
#endif
}

void init_power_management() {
    // Check if we are commissioned
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0) {
        esp_pm_config_t pm_config = {
            .max_freq_mhz = 96,
            .min_freq_mhz = 8, // To lower power usuage
            .light_sleep_enable = true // MUST stay true for auto-sleep
        };
        ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
        ESP_LOGI(TAG, "Power Management: Automatic Light Sleep Enabled");
    } else {
        // While commissioning, we keep sleep disabled to ensure BLE/Thread pairing is fast
        ESP_LOGI(TAG, "Power Management: Performance Mode for Commissioning");
    }
}

void setup_gpio_wakeup() {
    gpio_wakeup_enable(CONTACT_LOW_PIN, GPIO_INTR_LOW_LEVEL);
    gpio_wakeup_enable(CONTACT_FULL_PIN, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    ESP_LOGI("SLEEP", "GPIO wakeup configured for pins %d and %d", 
             CONTACT_LOW_PIN, CONTACT_FULL_PIN);
}

/* ========================================================================
 * MAIN APPLICATION
 * ======================================================================== */
extern "C" void app_main() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

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

    // GPIO Setup for External Resistors
    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << CONTACT_LOW_PIN) | (1ULL << CONTACT_FULL_PIN) | (1ULL << RESET_BUTTON_PIN);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE; // Using external 100k
    gpio_config(&io);

    led_init();
    setup_gpio_wakeup();
	
	
	    // 1. CREATE DATA MODEL
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, NULL);

    // Create a default config for the endpoints (DO NOT PASS NULL)
    contact_sensor::config_t sensor_config; 

    // Endpoint 1: Low Sensor
    endpoint_t *low_ep = contact_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, NULL);
    low_ep_id = endpoint::get_id(low_ep);

    // Endpoint 2: Full Sensor
    endpoint_t *full_ep = contact_sensor::create(node, &sensor_config, ENDPOINT_FLAG_NONE, NULL);
    full_ep_id = endpoint::get_id(full_ep);

    
    // Create Power Cluster
    cluster_t *ps_cluster = cluster::create(low_ep, PowerSource::Id, CLUSTER_FLAG_SERVER);
    cluster::global::attribute::create_feature_map(ps_cluster, 0x02);
    cluster::power_source::attribute::create_bat_voltage(ps_cluster, nullable<uint32_t>(3000), 0, 65535);
    cluster::power_source::attribute::create_bat_percent_remaining(ps_cluster, nullable<uint8_t>(200), 0, 200);

      // Initial Start
    esp_matter::start(app_event_cb);

 
  

    if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0) {
        commissioning_active = false;
        chip::DeviceLayer::SystemLayer().ScheduleLambda([]() {
            configure_sed_parameters();
            init_power_management();
        });
    } else {
        commissioning_active = true;
        xTaskCreate(commissioning_led_task, "comm_led", 2048, NULL, 5, &led_task_handle);
    }

    // Always create these. 
    xTaskCreate(hardware_task, "hw", 4096, NULL, 5, &hardware_task_handle);
    xTaskCreate(battery_task, "bat", 4096, NULL, 2, NULL); 
}




