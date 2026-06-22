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

    // --- Transmit gate -------------------------------------------------------
    // Lets the application keep the CAN transceiver in standby and power it up
    // only around a real transmit burst, without ever stopping the TWAI
    // controller (so RX keeps working in standby). CANSendFrame() transmits
    // directly: the first frame of a burst brings the transceiver up via the
    // wake callback, the rest go straight onto the controller. Frames that
    // cannot be queued (transceiver not ready, HW queue full) are refused so the
    // NMEA2000 library buffers and retries them. txStandby(), called from
    // the main loop, returns the transceiver to standby once everything has
    // drained.
    typedef bool (*tTxWakeCallback)();   // bring transceiver up; true = ready to send
    typedef void (*tTxIdleCallback)();   // queues drained; safe to standby
    void setTxStandbyCallbacks(tTxWakeCallback wake, tTxIdleCallback idle);

    // Non-blocking. Returns the transceiver to standby once the NMEA2000 library
    // send buffer AND the controller's HW TX queue have both drained, so a frame
    // is never cut off. Returns true if the transceiver is (now) in standby,
    // false while frames are still in flight. The driver never busy-waits — the
    // caller decides whether to loop on it before the next transmit cycle.
    // If the controller is not in the RUNNING state (bus-off / recovering) it
    // drops straight to standby and returns true, since nothing can drain — this
    // keeps a caller's drain loop from livelocking; recovery is left to
    // handleBusError().
    bool txStandby();

    // Poll from the main loop to recover from bus-off. When the controller has
    // entered TWAI_STATE_BUS_OFF, kicks off TWAI's native bus recovery (the
    // controller waits for 128 * 11 recessive bits, then parks in STOPPED) and
    // restarts it on a later poll once recovery completes. Non-blocking — the
    // driver is never torn down and the main loop is never stalled. Returns true
    // when a recovery step (initiate or restart) was taken this call.
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
    bool         _recovering = false;    // bus-off recovery in progress

    bool            _txAwake   = false;  // transceiver currently out of standby
    tTxWakeCallback _txWakeCb  = nullptr;
    tTxIdleCallback _txIdleCb  = nullptr;

    // Single point of coupling to the base class's send-frame ring buffer.
    // CANSendFrameBufferRead/Write are protected members of tNMEA2000 (the
    // library's own drivers read them the same way). Funnelled through here so
    // the dependency is obvious and lives in one place if the library renames
    // them.
    bool librarySendBufferEmpty() const;
};
