/*******************************************************************************
 *
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 *
 * Copyright (c) 2018-2021 Manuel Bleichenbacher
 *
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * High-level C API for ttn-esp32.
 *******************************************************************************/

#include "ttn.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "hal/hal_esp32.h"
#include "lmic/lmic.h"
#include "ttn_logging.h"
#include "ttn_provisioning.h"
#include "ttn_nvs.h"
#include "ttn_rtc.h"

#define TAG "ttn"
#define TTN_CLOCK_ERROR_PERCENT 10
#ifndef TTN_JOIN_EVENT_TIMEOUT_MS
#define TTN_JOIN_EVENT_TIMEOUT_MS 120000U
#endif
#ifndef TTN_TRANSMIT_EVENT_TIMEOUT_MS
#define TTN_TRANSMIT_EVENT_TIMEOUT_MS 120000U
#endif
#ifndef TTN_IDLE_WAIT_TIMEOUT_MS
#define TTN_IDLE_WAIT_TIMEOUT_MS 120000U
#endif

int retransmit_counter = 0;

#define DEFAULT_MAX_TX_POWER -1000
extern TaskHandle_t LED_SEQUENCE;

/**
 * @brief Reason the user code is waiting
 */
typedef enum
{
    TTN_WAITING_NONE,
    TTN_WAITING_FOR_JOIN,
    TTN_WAITING_FOR_TRANSMISSION
} ttn_waiting_reason_t;

// extern ttn_waiting_reason_t waiting_reason;

/**
 * @brief Event type
 */
typedef enum
{
    TTN_EVENT_NONE,
    TTN_EVNT_JOIN_COMPLETED,
    TTN_EVENT_JOIN_FAILED,
    TTN_EVENT_MESSAGE_RECEIVED,
    TTN_EVENT_TRANSMISSION_COMPLETED,
    TTN_EVENT_TRANSMISSION_FAILED
} ttn_event_t;

/**
 * @brief Event message sent from LMIC task to waiting client task
 */
typedef struct
{
    ttn_event_t event;
    uint8_t port;
    const uint8_t *message;
    size_t message_size;
} ttn_lmic_event_t;

static bool is_started;
bool has_joined;
static QueueHandle_t lmic_event_queue;
static ttn_message_cb message_callback;
ttn_waiting_reason_t waiting_reason;
static ttn_rf_settings_t last_rf_settings[4];
static ttn_rx_tx_window_t current_rx_tx_window;
static int subband = 2;
static ttn_data_rate_t join_data_rate = TTN_DR_JOIN_DEFAULT;
static int max_tx_power = DEFAULT_MAX_TX_POWER;

static void start(void);
static void stop(void);
static bool join_core(void);
static void config_rf_params(void);
static void event_callback(void *user_data, ev_t event);
static void message_received_callback(void *user_data, uint8_t port, const uint8_t *message, size_t message_size);
static void message_transmitted_callback(void *user_data, int success);
static void save_rf_settings(ttn_rf_settings_t *rf_settings);
static void clear_rf_settings(ttn_rf_settings_t *rf_settings);
static bool restored_session_is_valid(void);
static void clear_transient_tx_state_for_sleep_resume(const char *reason);
static void reset_waiting_state_after_timeout(const char *reason);

static bool restored_session_is_valid(void)
{
    // A restored LMIC image is only usable as a resumed joined session if it
    // actually carries a session DevAddr and is not still mid-join.
    return (LMIC.devaddr != 0) && ((LMIC.opmode & OP_JOINING) == 0);
}

static void clear_transient_tx_state_for_sleep_resume(const char *reason)
{
    const u2_t transient_mask = OP_TXDATA | OP_POLL | OP_TXRXPEND | OP_RNDTX | OP_NEXTCHNL;
    const u2_t old_opmode = LMIC.opmode;
    const u1_t old_tx_cnt = LMIC.txCnt;
    const u1_t old_up_repeat_count = LMIC.upRepeatCount;
    const u1_t old_pend_tx_len = LMIC.pendTxLen;

    if ((old_opmode & transient_mask) == 0 &&
        old_tx_cnt == 0 &&
        old_up_repeat_count == 0 &&
        old_pend_tx_len == 0)
    {
        return;
    }

    LMIC.opmode &= ~transient_mask;
    LMIC.txCnt = 0;
    LMIC.upRepeatCount = 0;
    LMIC.pendTxConf = 0;
    LMIC.pendTxPort = 0;
    LMIC.pendTxLen = 0;
    LMIC.dataBeg = 0;
    LMIC.dataLen = 0;

    ESP_LOGW(TAG,
             "%s: cleared transient LMIC TX state (opmode 0x%x -> 0x%x, txCnt=%u, upRepeat=%u, pendTxLen=%u)",
             reason,
             (unsigned)old_opmode,
             (unsigned)LMIC.opmode,
             (unsigned)old_tx_cnt,
             (unsigned)old_up_repeat_count,
             (unsigned)old_pend_tx_len);
}

static void reset_waiting_state_after_timeout(const char *reason)
{
    u2_t old_opmode;

    hal_esp32_enter_critical_section();
    waiting_reason = TTN_WAITING_NONE;
    hal_esp32_leave_critical_section();

    clear_transient_tx_state_for_sleep_resume(reason);

    hal_esp32_enter_critical_section();
    old_opmode = LMIC.opmode;
    LMIC.opmode = OP_NONE;
    hal_esp32_leave_critical_section();

    if (old_opmode != OP_NONE) {
        ESP_LOGW(TAG, "%s: forced LMIC opmode 0x%x -> OP_NONE after timeout",
                 reason,
                 (unsigned)old_opmode);
    }

    if (lmic_event_queue != NULL) {
        xQueueReset(lmic_event_queue);
    }
}

void ttn_init(void)
{
#if defined(TTN_IS_DISABLED)
    ESP_LOGE(TAG, "TTN is disabled. Configure a frequency plan using 'make menuconfig'");
    ASSERT(0);
#endif

    message_callback = NULL;
    hal_esp32_init_critical_section();
}

void ttn_configure_pins(spi_host_device_t spi_host, uint8_t nss, uint8_t rxtx, uint8_t rst, uint8_t dio0, uint8_t dio1)
{
    hal_esp32_configure_pins(spi_host, nss, rxtx, rst, dio0, dio1);

#if LMIC_ENABLE_event_logging
    ttn_log_init();
#endif
}

void ttn_set_subband(int band)
{
    // ESP_LOGW(TAG, "BAND %d", subband);

    subband = band;
    // ESP_LOGW(TAG, "BAND %d", subband);
}

void start(void)
{
    if (is_started)
        return;

    LMIC_registerEventCb(event_callback, NULL);
    LMIC_registerRxMessageCb(message_received_callback, NULL);

    os_init_ex(NULL);
    hal_esp32_enter_critical_section();
    LMIC_reset();

    LMIC_setClockError(MAX_CLOCK_ERROR * TTN_CLOCK_ERROR_PERCENT / 100);
    ESP_LOGI(TAG,
             "LoRa RX clock error allowance requested: %u%% (LMIC_ENABLE_arbitrary_clock_error=%d)",
             (unsigned)TTN_CLOCK_ERROR_PERCENT,
             (int)LMIC_ENABLE_arbitrary_clock_error);
    waiting_reason = TTN_WAITING_NONE;
    // lora_state_tracker = waiting_reason;

    hal_esp32_leave_critical_section();

    lmic_event_queue = xQueueCreate(4, sizeof(ttn_lmic_event_t));
    ESP_LOGI(TAG, "133:\n");
    ASSERT(lmic_event_queue != NULL);
    hal_esp32_start_lmic_task();
    is_started = true;
}

void stop(void)
{
    if (!is_started)
        return;
    
    hal_esp32_enter_critical_section();
    LMIC_shutdown();
    hal_esp32_stop_lmic_task();
    waiting_reason = TTN_WAITING_NONE;
    // lora_state_tracker = waiting_reason;

    hal_esp32_leave_critical_section();
}

void ttn_shutdown(void)
{
    stop();
}

bool ttn_provision(const char *dev_eui, const char *app_eui, const char *app_key)
{
    if (!ttn_provisioning_decode_keys(dev_eui, app_eui, app_key))
        return false;

    return ttn_provisioning_save_keys();
}

bool ttn_provision_transiently(const char *dev_eui, const char *app_eui, const char *app_key)
{
    return ttn_provisioning_decode_keys(dev_eui, app_eui, app_key);
}

bool ttn_provision_with_mac(const char *app_eui, const char *app_key)
{
    if (!ttn_provisioning_from_mac(app_eui, app_key))
        return false;

    return ttn_provisioning_save_keys();
}

void ttn_start_provisioning_task(void)
{
#if defined(TTN_HAS_AT_COMMANDS)
    ttn_provisioning_start_task();
#else
    ESP_LOGE(TAG, "AT commands are disabled. Change the configuration using 'make menuconfig'");
    ASSERT(0);
    esp_restart();
#endif
}

void ttn_wait_for_provisioning(void)
{
#if defined(TTN_HAS_AT_COMMANDS)
    if (ttn_is_provisioned())
    {
        ESP_LOGI(TAG, "Device is already provisioned");
        return;
    }

    while (!ttn_provisioning_have_keys())
        vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Device successfully provisioned");
#else
    ESP_LOGE(TAG, "AT commands are disabled. Change the configuration using 'make menuconfig'");
    ASSERT(0);
    esp_restart();
#endif
}

bool ttn_join_with_keys(const char *dev_eui, const char *app_eui, const char *app_key)
{
    if (!ttn_provisioning_decode_keys(dev_eui, app_eui, app_key))
        return false;

    return join_core();
}

bool ttn_join(void)
{
    if (!ttn_provisioning_have_keys())
    {
        if (!ttn_provisioning_restore_keys(false))
            return false;
    }

 
    // Register callback for received messages
    return join_core();
}

bool ttn_resume_after_deep_sleep(void)
{
    if (!ttn_provisioning_have_keys())
    {
        ESP_LOGW(TAG, "Dev ttn_provisioning_have_keys ed");
        if (!ttn_provisioning_restore_keys(false))
        {
            ESP_LOGW(TAG, "Dev ttn_provisioning_restore_keys ed");
            return false;
        }
    }

    if (!ttn_provisioning_have_keys())
    {
        ESP_LOGW(TAG, "DevEUI, AppEUI/JoinEUI and/or AppKey have not been provided");
        return false;
    }

    start();

    if (!ttn_rtc_restore())
        return false;

    clear_transient_tx_state_for_sleep_resume("ttn_resume_after_deep_sleep");

    if (!restored_session_is_valid())
    {
        ESP_LOGW(TAG, "RTC LMIC restore had no valid joined session; forcing fresh join");
        hal_esp32_enter_critical_section();
        LMIC_reset();
        waiting_reason = TTN_WAITING_NONE;
        hal_esp32_leave_critical_section();
        has_joined = false;
        return false;
    }

    has_joined = true;
    return true;
}

bool ttn_resume_after_power_off(int off_duration)
{
    if (!ttn_provisioning_have_keys())
    {
        if (!ttn_provisioning_restore_keys(false))
            return false;
    }

    if (!ttn_provisioning_have_keys())
    {
        ESP_LOGW(TAG, "DevEUI, AppEUI/JoinEUI and/or AppKey have not been provided");
        return false;
    }

    start();

    if (!ttn_nvs_restore(off_duration))
        return false;

    clear_transient_tx_state_for_sleep_resume("ttn_resume_after_power_off");

    if (!restored_session_is_valid())
    {
        ESP_LOGW(TAG, "NVS LMIC restore had no valid joined session; forcing fresh join");
        hal_esp32_enter_critical_section();
        LMIC_reset();
        waiting_reason = TTN_WAITING_NONE;
        hal_esp32_leave_critical_section();
        has_joined = false;
        return false;
    }

    has_joined = true;
    return true;
}

// Called immediately before sending join request message
void config_rf_params(void)
{
#if defined(CFG_us915) || defined(CFG_au915)
    if (subband != 0)
        LMIC_selectSubBand(subband - 1);
#endif

    if (join_data_rate != TTN_DR_JOIN_DEFAULT || max_tx_power != DEFAULT_MAX_TX_POWER)
    {
        dr_t dr = join_data_rate == TTN_DR_JOIN_DEFAULT ? LMIC.datarate : (dr_t)join_data_rate;
        s1_t txpow = max_tx_power == DEFAULT_MAX_TX_POWER ? LMIC.adrTxPow : max_tx_power;
        LMIC_setDrTxpow(dr, txpow);
    }
}

bool join_core(void)
{
    if (!ttn_provisioning_have_keys())
    {
        ESP_LOGW(TAG, "DevEUI, AppEUI/JoinEUI and/or AppKey have not been provided");
        return false;
    }

    start();

    has_joined = false;
    comms_counter = 0;
    retransmit_counter = 0;
    hal_esp32_enter_critical_section();

    xQueueReset(lmic_event_queue);

    waiting_reason = TTN_WAITING_FOR_JOIN;
    // lora_state_tracker = waiting_reason;


    config_rf_params();
    LMIC_startJoining();


    hal_esp32_wake_up();

    hal_esp32_leave_critical_section();

    ttn_lmic_event_t event;

    if (xQueueReceive(lmic_event_queue, &event, pdMS_TO_TICKS(TTN_JOIN_EVENT_TIMEOUT_MS)) != pdTRUE)
    {
        ESP_LOGW(TAG, "LoRaWAN join timed out waiting for LMIC event after %u ms",
                 (unsigned)TTN_JOIN_EVENT_TIMEOUT_MS);
        reset_waiting_state_after_timeout("join event timeout");
        hal_esp32_enter_critical_section();
        LMIC_reset();
        waiting_reason = TTN_WAITING_NONE;
        hal_esp32_leave_critical_section();
        has_joined = false;
        return false;
    }

    has_joined = event.event == TTN_EVNT_JOIN_COMPLETED;
    if (!has_joined) {
        hal_esp32_enter_critical_section();
        LMIC_reset();
        waiting_reason = TTN_WAITING_NONE;
        hal_esp32_leave_critical_section();
    }

    return has_joined;
}


// volatile int button_pressed_lora = -1;

// static void IRAM_ATTR button_isr_handler(void* arg) 
// {
//     // This function will be called when the button is pressed.
//     // You can put your code here.
//     // ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
//     // ESP_LOGI(TAG, "I2C de-initialized successfully");
//     // state = 0;
//     // vTaskDelete(NULL);
//     // vTaskDelay(10);
//     button_pressed_lora = (int)arg;

//     ets_printf("Button pressed! %d\n", (int)arg);

// }

/* 1.  map the event numbers to readable names once, near the top     */
static __attribute__((unused)) const char *lmic_ev_name(uint8_t e)
{
    /* add entries you care about; unknown ones fall through */
    switch (e) {
        case EV_SCAN_TIMEOUT:   return "SCAN_TIMEOUT";
        case EV_BEACON_FOUND:   return "BEACON_FOUND";
        case EV_JOINING:        return "JOINING";
        case EV_JOINED:         return "JOINED";
        case EV_LINK_DEAD:      return "LINK_DEAD";
        case EV_LINK_ALIVE:     return "LINK_ALIVE";
        case EV_TXCOMPLETE:     return "TXCOMPLETE";
        default:                return "UNKNOWN";
    }
}
/* ------------------------------------------------------------------ */

ttn_response_code_t ttn_transmit_message(const uint8_t *payload, size_t length, ttn_port_t port, bool confirm)
{
    // gpio_config_t io_conf;
    // //interrupt on both edges
    // io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
    // //bit mask of the pins
    // io_conf.pin_bit_mask = service_noimpact;
    // //set as input mode
    // io_conf.mode = GPIO_MODE_INPUT;
    // //enable pull-up mode
    // io_conf.pull_up_en = 1;
    // gpio_config(&io_conf);

    // //install gpio isr service
    // gpio_install_isr_service(0);
    // //hook isr handler for specific gpio pin
    // gpio_isr_handler_add(19, button_isr_handler, (void*) 19);
    // gpio_isr_handler_add(6, button_isr_handler, (void*) 6);

    hal_esp32_enter_critical_section();
    const ttn_waiting_reason_t busy_reason = waiting_reason;
    const u2_t busy_opmode = LMIC.opmode;
    if (busy_reason != TTN_WAITING_NONE || (busy_opmode & OP_TXRXPEND) != 0)
    {
        hal_esp32_leave_critical_section();
        ESP_LOGW(TAG,
                 "LoRaWAN transmit requested while busy (waiting_reason=%d, opmode=0x%x); clearing stale busy state",
                 busy_reason,
                 (unsigned)busy_opmode);
        reset_waiting_state_after_timeout("pre-transmit busy");
        return TTN_ERROR_TRANSMISSION_FAILED;
    }

    waiting_reason = TTN_WAITING_FOR_TRANSMISSION;
    // lora_state_tracker = waiting_reason;

    xQueueReset(lmic_event_queue);

    LMIC.client.txMessageCb = message_transmitted_callback;
    LMIC.client.txMessageUserData = NULL;
    LMIC_setTxData2(port, (xref2u1_t)payload, length, confirm);
    hal_esp32_wake_up();
    ESP_LOGI(TAG, "381:\n");

    hal_esp32_leave_critical_section();
    ESP_LOGI(TAG, "382:\n");
    const TickType_t tx_timeout_ticks = pdMS_TO_TICKS(TTN_TRANSMIT_EVENT_TIMEOUT_MS);
    const TickType_t tx_started_ticks = xTaskGetTickCount();
    while (true)
    {
        // printf("button_pressed_lora %d", button_pressed_lora);
        // if(button_pressed_lora == 6)
        // {

        //     // ttn_prepare_for_deep_sleep();
        //     turning_off();
        //     printf("Turn off called, transmit failed\n");
        //     return TTN_ERROR_TRANSMISSION_FAILED;
        //     // vTaskDelay(pdTICKS_TO_MS(3000));
        // }
        ttn_lmic_event_t result;
        ESP_LOGI(TAG, "397:\n");

        TickType_t elapsed_ticks = xTaskGetTickCount() - tx_started_ticks;
        if (elapsed_ticks >= tx_timeout_ticks)
        {
            ESP_LOGW(TAG, "LoRaWAN transmit timed out waiting for LMIC event after %u ms",
                     (unsigned)TTN_TRANSMIT_EVENT_TIMEOUT_MS);
            reset_waiting_state_after_timeout("transmit event timeout");
            return TTN_ERROR_TRANSMISSION_FAILED;
        }

        TickType_t remaining_ticks = tx_timeout_ticks - elapsed_ticks;
        if (remaining_ticks == 0)
            remaining_ticks = 1;

        if (xQueueReceive(lmic_event_queue, &result, remaining_ticks) != pdTRUE)
        {
            ESP_LOGW(TAG, "LoRaWAN transmit timed out waiting for LMIC event after %u ms",
                     (unsigned)TTN_TRANSMIT_EVENT_TIMEOUT_MS);
            reset_waiting_state_after_timeout("transmit event timeout");
            return TTN_ERROR_TRANSMISSION_FAILED;
        }
    /* now ‘result’ is valid – print it in a human-readable form       */
        ESP_LOGI(TAG, "399:\n");
        switch (result.event)
        {
        case TTN_EVENT_MESSAGE_RECEIVED:
            if (message_callback != NULL)
                message_callback(result.message, result.message_size, result.port);
            break;

        case TTN_EVENT_TRANSMISSION_COMPLETED:
            return TTN_SUCCESSFUL_TRANSMISSION;

        case TTN_EVENT_TRANSMISSION_FAILED:
            return TTN_ERROR_TRANSMISSION_FAILED;

        default:
            ASSERT(0);
        }
    }
}

void ttn_on_message(ttn_message_cb callback)
{
    message_callback = callback;
}

bool ttn_is_provisioned(void)
{
    if (ttn_provisioning_have_keys())
        return true;

    ttn_provisioning_restore_keys(true);

    return ttn_provisioning_have_keys();
}

void ttn_prepare_for_deep_sleep(void)
{
    if (!is_started)
    {
        ESP_LOGI(TAG, "LoRaWAN sleep prep skipped: stack not started; preserving RTC session");
        return;
    }

    clear_transient_tx_state_for_sleep_resume("ttn_prepare_for_deep_sleep");
    if (restored_session_is_valid())
    {
        ttn_rtc_save();
    }
    else
    {
        ESP_LOGW(TAG,
                 "LoRaWAN sleep prep skipped RTC save: no joined session (devaddr=0x%08lx opmode=0x%x)",
                 (unsigned long)LMIC.devaddr,
                 (unsigned)LMIC.opmode);
    }
    stop();
}

void ttn_prepare_for_power_off(void)
{
    clear_transient_tx_state_for_sleep_resume("ttn_prepare_for_power_off");
    ttn_nvs_save();
    stop();
}

void ttn_wait_for_idle(void)
{
    const TickType_t idle_timeout_ticks = pdMS_TO_TICKS(TTN_IDLE_WAIT_TIMEOUT_MS);
    const TickType_t started_ticks = xTaskGetTickCount();

    while (true)
    {
        TickType_t ticks_to_wait = ttn_busy_duration();
        if (ticks_to_wait == 0) {
            return;
        }

        const TickType_t elapsed_ticks = xTaskGetTickCount() - started_ticks;
        if (elapsed_ticks >= idle_timeout_ticks) {
            ESP_LOGW(TAG,
                     "LoRaWAN idle wait timed out after %u ms; clearing stale busy state",
                     (unsigned)TTN_IDLE_WAIT_TIMEOUT_MS);
            reset_waiting_state_after_timeout("idle wait timeout");
            return;
        }

        const TickType_t remaining_ticks = idle_timeout_ticks - elapsed_ticks;
        if (ticks_to_wait > remaining_ticks) {
            ticks_to_wait = remaining_ticks;
        }
        if (ticks_to_wait == 0) {
            ticks_to_wait = 1;
        }

        vTaskDelay(ticks_to_wait);
    }
}

TickType_t ttn_busy_duration(void)
{
    TickType_t duration = hal_esp32_get_timer_duration();
    if (duration != 0)
        return duration; // busy or timer scheduled

    if ((LMIC.opmode & (OP_JOINING | OP_TXDATA | OP_POLL | OP_TXRXPEND)) != 0)
        return pdMS_TO_TICKS(100); // pending action

    return 0; // idle
}


void ttn_set_rssi_cal(int8_t rssi_cal)
{
    hal_esp32_set_rssi_cal(rssi_cal);
}

bool ttn_adr_enabled(void)
{
    return LMIC.adrEnabled != 0;
}

void ttn_set_adr_enabled(bool enabled)
{
    hal_esp32_enter_critical_section();
    LMIC_setAdrMode(enabled);
    hal_esp32_leave_critical_section();
}

void ttn_set_data_rate(ttn_data_rate_t data_rate)
{
    join_data_rate = data_rate;

    if (has_joined)
    {
        hal_esp32_enter_critical_section();
        LMIC_setDrTxpow(data_rate, LMIC.adrTxPow);
        hal_esp32_leave_critical_section();
    }
}

void ttn_set_max_tx_pow(int tx_pow)
{
    max_tx_power = tx_pow;

    if (has_joined)
    {
        hal_esp32_enter_critical_section();
        LMIC_setDrTxpow(LMIC.datarate, tx_pow);
        hal_esp32_leave_critical_section();
    }
}

void ttn_set_confirm_retry_limit(uint8_t attempts)
{
    hal_esp32_enter_critical_section();
    LMIC_set_confirm_retry_limit((u1_t)attempts);
    hal_esp32_leave_critical_section();
}

uint8_t ttn_get_confirm_retry_limit(void)
{
    return (uint8_t)LMIC_get_confirm_retry_limit();
}

ttn_rf_settings_t ttn_get_rf_settings(ttn_rx_tx_window_t window)
{
    int index = ((int)window) & 0x03;
    return last_rf_settings[index];
}

ttn_rf_settings_t ttn_tx_settings(void)
{
    return last_rf_settings[TTN_WINDOW_TX];
}

ttn_rf_settings_t ttn_rx1_settings(void)
{
    return last_rf_settings[TTN_WINDOW_RX1];
}

ttn_rf_settings_t ttn_rx2_settings(void)
{
    return last_rf_settings[TTN_WINDOW_RX2];
}

ttn_rx_tx_window_t ttn_rx_tx_window(void)
{
    return current_rx_tx_window;
}

int ttn_rssi(void)
{
    return LMIC.rssi;
}

// --- Callbacks ---

#if LMIC_ENABLE_event_logging
static __attribute__((unused)) const char *event_names[] = {LMIC_EVENT_NAME_TABLE__INIT};
#endif

// Called by LMIC when an LMIC event (join, join failed, reset etc.) occurs
void event_callback(void *user_data, ev_t event)
{
    // update monitoring information
    switch (event)
    {
    case EV_TXSTART:
        retransmit_counter = (LMIC.pendTxConf && LMIC.txCnt > 0) ? (LMIC.txCnt - 1) : 0;
        current_rx_tx_window = TTN_WINDOW_TX;
        save_rf_settings(&last_rf_settings[TTN_WINDOW_TX]);
        clear_rf_settings(&last_rf_settings[TTN_WINDOW_RX1]);
        clear_rf_settings(&last_rf_settings[TTN_WINDOW_RX2]);

        break;

    case EV_RXSTART:
        if (current_rx_tx_window != TTN_WINDOW_RX1)
        {
            current_rx_tx_window = TTN_WINDOW_RX1;
            save_rf_settings(&last_rf_settings[TTN_WINDOW_RX1]);
        }
        else
        {
            current_rx_tx_window = TTN_WINDOW_RX2;
            save_rf_settings(&last_rf_settings[TTN_WINDOW_RX2]);
        }
        break;

    default:
        current_rx_tx_window = TTN_WINDOW_IDLE;
        break;
    };

#if LMIC_ENABLE_event_logging
    ttn_log_event(event, event_names[event], 0);
#elif CONFIG_LOG_DEFAULT_LEVEL >= 3

     
#endif
        ttn_event_t ttn_event = TTN_EVENT_NONE;
    ESP_LOGI(TAG, "ttn_event %d, waiting_reason %d, LMIC.opmode %u \n", ttn_event, waiting_reason, LMIC.opmode);
    if (waiting_reason == TTN_WAITING_FOR_JOIN)
    {
        if (event == EV_JOINED)
        {
            ttn_event = TTN_EVNT_JOIN_COMPLETED;
            comms_counter = 0;
            retransmit_counter = 0;
        }
        else if (event == EV_JOIN_FAILED || event == EV_REJOIN_FAILED || event == EV_RESET)
        {
            ttn_event = TTN_EVENT_JOIN_FAILED;
        }
    }

    if (ttn_event == TTN_EVENT_NONE)
        return;

    ttn_lmic_event_t result = {.event = ttn_event};
    waiting_reason = TTN_WAITING_NONE;
    // lora_state_tracker = waiting_reason;
    ESP_LOGI(TAG, "714:\n");
    xQueueSend(lmic_event_queue, &result, pdMS_TO_TICKS(100));
}

// Called by LMIC when a message has been received
void message_received_callback(void *user_data, uint8_t port, const uint8_t *message, size_t message_size)
{
    // ESP_LOGI(TAG, "Downlink Recevied on port %d", port);
    // ESP_LOGI(TAG,"Downlink of %d bytes received on port %d:", message_size, port);

    for(int i = 0; i < sizeof(message_size); ++i)
    {
        // printf("%d\n ", message[i]); // Print message
    }
    if (LMIC_complianceRxMessage(port, message, message_size) == LMIC_COMPLIANCE_RX_ACTION_PROCESS) 
    {
        ttn_lmic_event_t result = {
            .event = TTN_EVENT_MESSAGE_RECEIVED, .port = port, .message = message, .message_size = message_size};
        xQueueSend(lmic_event_queue, &result, pdMS_TO_TICKS(100));
        printf("\033[38;5;202mLMIC Compliance message process\033[0m\n");
    } else {    //Required or any application layer downlink code will not be processed.
        ttn_lmic_event_t result = {
        .event = TTN_EVENT_MESSAGE_RECEIVED, .port = port, .message = message, .message_size = message_size};
        ESP_LOGI(TAG, "737:\n");
        xQueueSend(lmic_event_queue, &result, pdMS_TO_TICKS(100));
        // printf("\033[38;5;202mLMIC Compliance message not process\033[0m\n");
    }
}

// Called by LMIC when a message has been transmitted (or the transmission failed)
void message_transmitted_callback(void *user_data, int success)
{
    waiting_reason = TTN_WAITING_NONE;
    // lora_state_tracker = waiting_reason;
    if (success) {
        retransmit_counter = 0;
    }

    ttn_lmic_event_t result = {.event = success ? TTN_EVENT_TRANSMISSION_COMPLETED : TTN_EVENT_TRANSMISSION_FAILED};
    ESP_LOGI(TAG, "750:\n");
    xQueueSend(lmic_event_queue, &result, pdMS_TO_TICKS(100));
}

// --- Helpers

void save_rf_settings(ttn_rf_settings_t *rf_settings)
{
    rf_settings->spreading_factor = (ttn_spreading_factor_t)(getSf(LMIC.rps) + 1);
    rf_settings->bandwidth = (ttn_bandwidth_t)(getBw(LMIC.rps) + 1);
    rf_settings->frequency = LMIC.freq;
}

void clear_rf_settings(ttn_rf_settings_t *rf_settings)
{
    memset(rf_settings, 0, sizeof(*rf_settings));
}


