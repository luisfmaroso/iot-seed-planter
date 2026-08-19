# IoT Seed Planter

A rough simulation of a **seeder ECU** (Electronic Control Unit) with IoT monitoring, built on an ESP32 with the Arduino framework and the [Blynk](https://blynk.io/) cloud platform.

Final project for the *Internet of Things* course — PPGCA / UPF, 2024.

<img width="1278" alt="Blynk dashboard for the seed planter" src="https://github.com/user-attachments/assets/5e49af4f-4f53-481d-a9b9-f456c7c3ce3a" />

## What it does

Agricultural seeders drop seeds at a target rate while the machine moves. Two things can go wrong: a **fail** (no seed dropped in the expected window) or a **double** (two seeds dropped instead of one). This project simulates that pipeline and reports the quality of the seeding in real time.

- A **stepper motor** stands in for the seed metering disc, its speed derived from the requested seed rate.
- An **ultrasonic sensor** watches the seed tube and counts seeds passing by.
- The firmware compares detections against the expected seeding interval and classifies each window as a success, a fail, or a double.
- Temperature/humidity and a simulated battery voltage are sampled alongside, as an ECU would monitor its environment and power rail.
- Everything is pushed to a **Blynk dashboard**, which also lets you enable/disable the system and change the seed rate remotely.

## Hardware

| Component | Purpose | ESP32 pins |
|---|---|---|
| ESP32 DevKit | Controller | — |
| 28BYJ-48 stepper (or similar, 200 steps/rev) | Seed metering disc | 27, 14, 12, 13 |
| HC-SR04 ultrasonic sensor | Seed detection | Trigger 25, Echo 26 |
| DHT22 | Temperature / humidity | 33 |
| Potentiometer | Battery voltage simulator (0–16 V) | 32 |
| Onboard LED | System enabled indicator | 2 |

## Blynk virtual pins

| Pin | Direction | Meaning |
|---|---|---|
| V0 | In | System enable switch |
| V1 | Out | Fails + doubles counter |
| V2 | Out | Uptime (seconds) |
| V3 | In | Seed rate (seeds/m) |
| V4 | Out | Temperature (°C) |
| V5 | Out | Simulated battery voltage |

## Building

The project uses [PlatformIO](https://platformio.org/). Dependencies are declared in `platformio.ini` and fetched automatically.

1. Fill in your credentials in [`inc/custom.h`](inc/custom.h) — Wi-Fi SSID/password and the Blynk template ID, template name, and auth token.
2. Build and upload:

```bash
pio run --target upload
```

3. Open the serial monitor at 9600 baud to watch the fail/success/double counters:

```bash
pio device monitor
```

## Notes

This is a course project, not production firmware . The seed detection is a simple distance threshold on a single sensor, and the "vehicle speed" is implied by the stepper rate rather than measured. It was built to demonstrate an end-to-end IoT loop: sensing on the edge, control logic on the MCU, and remote monitoring in the cloud.
