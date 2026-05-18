# NMEA2000_TWAI

NMEA 2000 CAN driver for ESP32 that talks to the **TWAI** (Two-Wire Automotive
Interface) peripheral directly through ESP-IDF, with no dependency on the
Arduino `CAN` library or any Xtensa-specific headers.

It is a drop-in replacement for [`ttlappalainen/NMEA2000_esp32`][orig], which
does not compile on RISC-V ESP32 variants (ESP32-C3, C6, H2) because it
includes `soc/dport_reg.h` — a header that only exists for the original
Xtensa ESP32.

[orig]: https://github.com/ttlappalainen/NMEA2000_esp32

## Features

- Works on every ESP32 family with a TWAI controller: classic ESP32, S2, S3,
  C3, C6, H2.
- Uses only the public ESP-IDF TWAI API (`driver/twai.h`).
- Sleep/wake helpers (`twaiSleep()` / `twaiWake()`) that drain the TX queue
  and stop the controller so the APB clock can be gated during light sleep —
  measured at ~1 mA on an ESP32-C3 SuperMini.
- Bus-off recovery via `handleBusError()`, polled from the main loop.
- Configurable TX/RX queue depth (`TWAI_TX_QUEUE_LEN`, `TWAI_RX_QUEUE_LEN`).

## Compatibility

| Item              | Supported                                          |
|-------------------|----------------------------------------------------|
| Chips             | ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2 |
| Frameworks        | Arduino-ESP32, ESP-IDF                             |
| ESP-IDF range     | **4.2 – 5.4** (tested), compiles with deprecation warnings on **5.5+** |
| Arduino-ESP32     | 2.0.x and 3.x                                      |

### A note on the ESP-IDF TWAI API

This driver uses the **legacy single-controller TWAI API** (`driver/twai.h`,
functions without the `_v2` suffix):

- **Introduced**: ESP-IDF v4.2.
- **`_v2` variants added**: ESP-IDF v5.2 (for multi-controller support).
- **Formally deprecated**: ESP-IDF v5.5 in favour of the new `esp_twai.h`
  driver (component `esp_driver_twai`), which has an event-driven architecture
  and TWAI-FD support.
- **Removal**: not yet scheduled; the legacy header still compiles on the
  v6.x master branch but emits deprecation warnings.

If/when Espressif removes the legacy header, this library will need to be
ported to `esp_twai_onchip.h`. The new and legacy drivers cannot be linked
into the same firmware image.

## Installation

### PlatformIO

Either drop this folder into your project's `lib/` directory, or add to
`platformio.ini`:

```ini
lib_deps =
    https://github.com/ttlappalainen/NMEA2000
    https://github.com/knifter/NMEA2000_TWAI
```

## Usage

```cpp
#include <NMEA2000_TWAI.h>

// CAN TX = GPIO 5, CAN RX = GPIO 4, 250 kbps (NMEA 2000 default)
tNMEA2000_TWAI NMEA2000(GPIO_NUM_5, GPIO_NUM_4);

void setup() {
    NMEA2000.SetProductInformation("00000001", 100, "NemaWorm", "1.0", "1.0");
    NMEA2000.SetDeviceInformation(1, 132, 25, 2046);
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);
    NMEA2000.Open();
}

void loop() {
    NMEA2000.ParseMessages();
    NMEA2000.handleBusError();   // recover from bus-off if needed
}
```

### Low-power / sleep

```cpp
NMEA2000.twaiSleep();          // drains TX queue, then twai_stop()
esp_light_sleep_start();
NMEA2000.twaiWake();           // twai_start() + 2 ms bus-integration delay
```

### Tuning queue depth

```cpp
// In platformio.ini build_flags, or before including the header:
build_flags =
    -DTWAI_TX_QUEUE_LEN=40
    -DTWAI_RX_QUEUE_LEN=40
```

Defaults are 20 frames each direction.

## License
This software is written by [Tijs van Roon](https://github.com/knifter). It is free to use under the [MIT License](https://opensource.org/licenses/MIT).
