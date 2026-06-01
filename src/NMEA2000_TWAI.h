#pragma once

// Minimal NMEA 2000 CAN driver using the ESP-IDF TWAI peripheral directly.
// Replaces ttlappalainen/NMEA2000_esp32 which does not compile on ESP32-C3
// because it includes soc/dport_reg.h (Xtensa/ESP32 only).

#include <NMEA2000.h>
#include "driver/twai.h"

#ifndef TWAI_RX_QUEUE_LEN
#define TWAI_RX_QUEUE_LEN 20
#endif
#ifndef TWAI_TX_QUEUE_LEN
#define TWAI_TX_QUEUE_LEN 20
#endif


class tNMEA2000_TWAI : public tNMEA2000 {
public:
    typedef enum {
        CAN_SPEED_25KBPS,
        CAN_SPEED_50KBPS,
        CAN_SPEED_100KBPS,
        CAN_SPEED_125KBPS,
        CAN_SPEED_250KBPS,
        CAN_SPEED_500KBPS,
        CAN_SPEED_1000KBPS,
    } CAN_speed_t;

    tNMEA2000_TWAI(gpio_num_t txPin, gpio_num_t rxPin, tNMEA2000_TWAI::CAN_speed_t speed = CAN_SPEED_250KBPS);

    // Stop the TWAI controller between sends so the APB clock domain can
    // be gated during light sleep. twai_stop() discards in-flight frames,
    // so wait for the driver's TX queue to drain first. msgs_to_tx hitting
    // zero means the controller is idle regardless of whether the last
    // frame succeeded or failed — both are safe states for stopping.
    void twaiSleep(uint32_t timeout = 100);

    // Restart TWAI before sending. CAN requires listening to 11 consecutive
    // recessive bits (bus integration) before transmitting — ~44 µs at 250 kbps.
    // A small delay after start allows integration to complete.
    void twaiWake();

    // Non-blocking: true once the driver's TX queue has drained. Poll this
    // before putting the transceiver into standby so a frame is never cut off
    // mid-flight (the msgs_to_tx==0 check twaiSleep() blocks on internally).
    bool twaiTxQueueEmpty();

    // Poll from the main loop to recover from bus-off. When the controller
    // has entered TWAI_STATE_BUS_OFF, tears down and reinstalls the driver
    // after a 1 s cool-down. Returns true if recovery was attempted.
    bool handleBusError();

protected:
    bool CANOpen() override;
    bool CANSendFrame(unsigned long id, unsigned char len,
                      const unsigned char *buf, bool wait_sent = true) override;
    bool CANGetFrame(unsigned long &id, unsigned char &len,
                     unsigned char *buf) override;

private:
    gpio_num_t   _txPin;
    gpio_num_t   _rxPin;
    CAN_speed_t  _speed;
    bool         _running = false;
};
