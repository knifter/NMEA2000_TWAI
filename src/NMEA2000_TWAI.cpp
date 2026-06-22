#include "NMEA2000_TWAI.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#define TAG "NMEA2000_TWAI"


tNMEA2000_TWAI::tNMEA2000_TWAI(gpio_num_t txPin, gpio_num_t rxPin, tNMEA2000_TWAI::CAN_speed_t speed)
    : _txPin(txPin), _rxPin(rxPin), _speed(speed) {}

bool tNMEA2000_TWAI::CANOpen() 
{
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(_txPin, _rxPin, TWAI_MODE_NORMAL);
    g.rx_queue_len = TWAI_RX_QUEUE_LEN;
    g.tx_queue_len = TWAI_TX_QUEUE_LEN;
#ifdef CONFIG_TWAI_ISR_IN_IRAM
    g.intr_flags = ESP_INTR_FLAG_IRAM;
#endif
    twai_timing_config_t t;
    switch (_speed) 
    {
        case CAN_SPEED_25KBPS:   t = TWAI_TIMING_CONFIG_25KBITS();   break;
        case CAN_SPEED_50KBPS:   t = TWAI_TIMING_CONFIG_50KBITS();   break;
        case CAN_SPEED_100KBPS:  t = TWAI_TIMING_CONFIG_100KBITS();  break;
        case CAN_SPEED_125KBPS:  t = TWAI_TIMING_CONFIG_125KBITS();  break;
        case CAN_SPEED_500KBPS:  t = TWAI_TIMING_CONFIG_500KBITS();  break;
        case CAN_SPEED_1000KBPS: t = TWAI_TIMING_CONFIG_1MBITS();    break;
        case CAN_SPEED_250KBPS:
        default:                 t = TWAI_TIMING_CONFIG_250KBITS();  break;
    };
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK)
        return false;
    if (twai_start() != ESP_OK)
        return false;

    _running = true;
    return true;
};

bool tNMEA2000_TWAI::CANSendFrame(unsigned long id, unsigned char len,
                                  const unsigned char *buf, bool wait_sent)
{
    // Transmit immediately. If the transceiver is still in standby, bring it up
    // first via the wake callback; a false return means "not ready yet" — refuse
    // the frame so the NMEA2000 library buffers it and retries on the next
    // ParseMessages(). _txAwake keeps the wake/settle to the first frame of a
    // burst; the rest go straight onto the controller. The transceiver is
    // returned to standby later by txStandby(), driven from the main loop.
    if (len > 8)
        return false;
    if (!_txAwake)
    {
        if (_txWakeCb != nullptr && !_txWakeCb())
            return false;           // not ready — library buffers + retries
        _txAwake = true;
    };
    twai_message_t msg = {};
    msg.extd             = 1;       // NMEA 2000 uses 29-bit extended frames
    msg.identifier       = id;
    msg.data_length_code = len;
    memcpy(msg.data, buf, len);
    return twai_transmit(&msg, 0) == ESP_OK;   // HW queue full — library retries
}

void tNMEA2000_TWAI::setTxStandbyCallbacks(tTxWakeCallback wake, tTxIdleCallback idle)
{
    _txWakeCb = wake;
    _txIdleCb = idle;
}

bool tNMEA2000_TWAI::txStandby()
{
    // Non-blocking. Drop the transceiver to standby only once everything queued
    // has drained — both the NMEA2000 library send buffer AND the controller's
    // HW TX queue — so a frame is never cut off mid-flight. Returns true if the
    // transceiver is (now) in standby, false while frames are still in flight.
    // The driver never busy-waits; the caller decides whether to loop.
    if (!_txAwake)
        return true;                // already in standby
    // If the controller can't transmit (bus-off / recovering), nothing will ever
    // drain — waiting for it would livelock the caller. Give up draining, drop to
    // standby, and let handleBusError() recover. The still-queued frames can't go
    // out anyway; the library keeps retrying them once the bus is back.
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK || st.state != TWAI_STATE_RUNNING)
    {
        if (_txIdleCb != nullptr)
            _txIdleCb();
        _txAwake = false;
        return true;
    };
    SendFrames();                   // push any library backlog to HW (non-blocking)
    if (!librarySendBufferEmpty())
        return false;               // library still has frames queued
    if (!twaiTxQueueEmpty())
        return false;               // HW queue still draining onto the bus
    if (_txIdleCb != nullptr)
        _txIdleCb();
    _txAwake = false;
    return true;
}

bool tNMEA2000_TWAI::CANGetFrame(unsigned long &id, unsigned char &len,
                                 unsigned char *buf) 
{
    twai_message_t msg = {};
    if (twai_receive(&msg, 0) != ESP_OK) return false;  // non-blocking
    id  = msg.identifier;
    len = msg.data_length_code;
    memcpy(buf, msg.data, len);
    return true;
}

bool tNMEA2000_TWAI::librarySendBufferEmpty() const
{
    return CANSendFrameBufferRead == CANSendFrameBufferWrite;
}

bool tNMEA2000_TWAI::twaiTxQueueEmpty()
{
    if (!_running)
        return true;
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK)
        return true;
    return s.msgs_to_tx == 0;
}

void tNMEA2000_TWAI::twaiSleep(uint32_t timeout) 
{
    if (!_running)
        return;
    twai_status_info_t s;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout);
    while ((int32_t)(xTaskGetTickCount() - deadline) < 0) 
    {
        if (twai_get_status_info(&s) == ESP_OK && s.msgs_to_tx == 0) 
            break;
        vTaskDelay(pdMS_TO_TICKS(1));
    };
    twai_stop();
    _running = false;
}

void tNMEA2000_TWAI::twaiWake() 
{
    if (_running) 
        return;
    twai_start();
    vTaskDelay(pdMS_TO_TICKS(2));
    _running = true;
}

bool tNMEA2000_TWAI::handleBusError() 
{
    twai_status_info_t s;
    if (twai_get_status_info(&s) != ESP_OK)
        return false;
    if (s.state == TWAI_STATE_BUS_OFF)
    {
        ESP_LOGE(TAG, "Bus-Off error detected. re-init...");
        twai_stop();
        twai_driver_uninstall();
        _running = false;
        vTaskDelay(pdMS_TO_TICKS(500));
        return CANOpen();
    };
    return false;
}
