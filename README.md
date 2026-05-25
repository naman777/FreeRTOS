# ESP32 FreeRTOS Multi-Task Sensor Monitor

A real-time embedded firmware project for the ESP32 that demonstrates multi-task scheduling, inter-task communication, and peripheral drivers using **FreeRTOS** and the **ESP-IDF** framework. The entire project runs on the [Wokwi](https://wokwi.com) ESP32 simulator — no physical hardware required, but all I²C transactions, register reads, and driver logic are identical to what runs on real silicon.

---

## Features

- **3 concurrent FreeRTOS tasks** with distinct priorities and responsibilities
- **BMP280 temperature sensor** — real I²C driver with chip-ID validation, NVM calibration read, and Bosch's integer compensation formula
- **FreeRTOS Queue Set** — `control_task` wakes instantly on either a sensor reading *or* a button press with no polling loop
- **Mutex-protected shared state** — demonstrable race condition that can be reproduced by commenting two lines
- **GPIO interrupt (ISR)** — falling-edge button press handled in interrupt context, signalled via binary semaphore
- **UART logging** — periodic formatted output from a dedicated logger task
- **Fully buildable** with ESP-IDF v5.3+ and simulatable with the Wokwi VS Code extension

---

## Hardware / Simulation

| Component | Connection |
|-----------|-----------|
| ESP32 DevKit V1 | — |
| BMP280 (temperature sensor) | SDA → GPIO21, SCL → GPIO22 (I²C 100 kHz) |
| Push button | GPIO4 (falling-edge interrupt, 10 kΩ pull-up) |
| Green LED | GPIO2 (220 Ω series resistor) |

The `diagram.json` file describes the complete circuit for the Wokwi simulator.

---

## Architecture

### Task overview

```
┌─────────────────────────────────────────────────────────────┐
│                        ESP32 Firmware                        │
│                                                             │
│  sensor_task (pri 5)                                        │
│  ├─ Reads BMP280 via I²C every 500 ms                       │
│  ├─ Updates latest_reading (mutex-protected)                │
│  └─ Pushes sensor_reading_t onto sensorQueue                │
│                                                             │
│  control_task (pri 6)  ◄── Queue Set                        │
│  ├─ Blocks on sensorQueue OR buttonSemaphore simultaneously  │
│  ├─ Applies alert threshold → drives LED                    │
│  └─ Handles button override immediately via ISR semaphore   │
│                                                             │
│  logger_task (pri 4)                                        │
│  ├─ Wakes every 1 s                                         │
│  ├─ Reads latest_reading (mutex-protected)                  │
│  └─ Writes "t=… temp=…" line over UART                     │
└─────────────────────────────────────────────────────────────┘
```

### Synchronisation objects

```
sensor_task  ──[sensor_reading_t]──► sensorQueue    ──┐
                                                       ├──► controlQueueSet ──► control_task
button ISR   ──[xSemaphoreGiveFromISR]──► buttonSem ──┘

sensor_task  ◄──[dataMutex]──► logger_task   (protects latest_reading)
```

| Object | Type | Purpose |
|--------|------|---------|
| `sensorQueue` | `QueueHandle_t` (depth 10) | Passes readings from `sensor_task` to `control_task` |
| `dataMutex` | Mutex semaphore | Guards `latest_reading` shared between `sensor_task` and `logger_task` |
| `buttonSemaphore` | Binary semaphore | Carries the ISR signal to `control_task` |
| `controlQueueSet` | Queue Set (capacity 11) | Fans `sensorQueue` + `buttonSemaphore` into a single blocking point |

### Why a Queue Set?

Without a queue set, `control_task` would call `xQueueReceive(sensorQueue, portMAX_DELAY)` and be unable to react to a button press until the *next* sensor reading arrived — up to 500 ms of latency.

With `xQueueSelectFromSet(controlQueueSet, portMAX_DELAY)`, the task blocks on **both** handles simultaneously and wakes the instant *either* fires:

```c
QueueSetMemberHandle_t active = xQueueSelectFromSet(controlQueueSet, portMAX_DELAY);

if (active == sensorQueue)       { /* threshold logic → LED */ }
else if (active == buttonSemaphore) { /* manual override → LED off */ }
```

---

## BMP280 Driver (`main/bmp280.c`)

A full I²C device driver, not a stub. Every transaction targets a real register:

| Step | Register | Description |
|------|----------|-------------|
| 1 | `0xD0` (read) | Chip-ID check — must return `0x60`, otherwise init fails |
| 2 | `0x88–0x8D` (read 6 bytes) | Factory calibration NVM: `dig_T1` (u16), `dig_T2`, `dig_T3` (i16) |
| 3 | `0xF4` ← `0x23` (write) | `ctrl_meas`: temp oversampling ×1, normal/continuous mode |
| 4 | `0xFA–0xFC` (read 3 bytes) | Raw 20-bit ADC temperature value |
| 5 | — (computation) | Bosch integer compensation formula (datasheet §8.2) |

The compensation formula (from the BMP280 datasheet, verbatim):

```c
var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * dig_T2) >> 11;
var2 = (((((adc_T >> 4) - (int32_t)dig_T1) *
           ((adc_T >> 4) - (int32_t)dig_T1)) >> 12) * dig_T3) >> 14;
t_fine = var1 + var2;
T = (float)((t_fine * 5 + 128) >> 8) / 100.0f;  // degrees Celsius
```

Error handling covers NACKs, I²C timeouts, and the not-ready sentinel value (`0x80000`).

---

## ISR Design

The button ISR is intentionally minimal — no logging, no heap allocation, no blocking calls:

```c
static void IRAM_ATTR button_isr_handler(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &higher_prio_woken);
    portYIELD_FROM_ISR_ARG(higher_prio_woken);
}
```

`xSemaphoreGiveFromISR` is the ISR-safe variant. `portYIELD_FROM_ISR_ARG` allows the scheduler to immediately context-switch to `control_task` if it has higher priority than whatever was running — this is what gives the button sub-millisecond response time.

---

## Race Condition Demo

The firmware includes a deliberately introducible race condition to demonstrate what happens when shared state is accessed without synchronisation.

**To reproduce:**

In [`main/main.c`](main/main.c), find the `/* MUTEX GUARDS */` comments and comment out the `xSemaphoreTake` / `xSemaphoreGive` pairs in both `sensor_task` and `logger_task`. Rebuild and run.

Under load you'll see `logger_task` occasionally print a timestamp from reading *N* paired with the temperature from reading *N+1* — a classic TOCTOU (time-of-check/time-of-use) race on a non-atomic struct copy.

**Re-enable both mutex blocks** and the corruption disappears.

---

## Project Structure

```
freeRtos/
├── main/
│   ├── main.c          # App entry point, tasks, ISR, RTOS setup
│   ├── bmp280.c        # Full BMP280 I²C driver
│   ├── bmp280.h        # Driver API and register definitions
│   └── CMakeLists.txt  # Component registration
├── CMakeLists.txt      # Root ESP-IDF project file
├── sdkconfig.defaults  # Pre-configured: ESP32 target, FreeRTOS 1 kHz, debug logging
├── diagram.json        # Wokwi circuit (ESP32 + BMP280 + button + LED)
├── wokwi.toml          # Wokwi simulator config (points to build artifacts)
└── .vscode/
    └── tasks.json      # VS Code build/clean/set-target tasks
```

---

## Build & Run

### Prerequisites

- [VS Code](https://code.visualstudio.com/)
- [Espressif IDF Extension](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension) — run **ESP-IDF: Configure ESP-IDF Extension** (Express mode, v5.3) to install the full toolchain
- [Wokwi Simulator Extension](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode) — free personal license at [wokwi.com/license](https://wokwi.com/license)

### Build

```bash
# In an ESP-IDF terminal (Ctrl+Shift+P → "ESP-IDF: Open ESP-IDF Terminal")
idf.py set-target esp32
idf.py build
```

Or use the VS Code task: **Ctrl+Shift+B** → **ESP-IDF: Build**.

### Simulate

1. Open `diagram.json` in VS Code
2. Press `F1` → **Wokwi: Start Simulator**

### Expected serial output

```
I (330) BMP280: chip ID 0x60 OK
I (340) BMP280: calib: T1=27504 T2=26435 T3=-1000
I (350) BMP280: BMP280 initialised, normal mode
I (360) FIRMWARE: boot: sensor=500ms threshold=30.0°C queue_set=yes
t=501234 temp=27.53
t=1502891 temp=27.54
I (1860) FIRMWARE: ALERT: 31.20 °C > threshold 30.0 °C
I (1862) FIRMWARE: button ISR: alert cleared by manual override
```

### Testing the simulation

| Action | What to do | Expected behaviour |
|--------|-----------|-------------------|
| Periodic logging | Watch the serial monitor | `t=… temp=…` line every second |
| Temperature alert | Click BMP280 → raise temperature above 30 °C | Green LED turns on, `ALERT` log appears |
| Button override | Click the red button while LED is on | LED turns off **immediately** (queue set latency) |
| Race condition | Comment out `MUTEX GUARDS` blocks → rebuild | Mismatched timestamp/temperature pairs in logger output |

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Hardware target | ESP32 (Xtensa LX6 dual-core) |
| RTOS | FreeRTOS (via ESP-IDF) |
| Framework | ESP-IDF v5.3 |
| Language | C (C17) |
| Build system | CMake + Ninja |
| Compiler | xtensa-esp-elf-gcc 13.2.0 |
| Simulation | Wokwi ESP32 simulator |

---

## License

MIT
