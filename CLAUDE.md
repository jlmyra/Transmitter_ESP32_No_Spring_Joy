# Project Instructions for Claude Code

## Project Overview

ESP32-based RC transmitter (no-spring throttle variant) for the MultiWii drone flight controller.

- **Board:** ESP32 MH-ET LIVE MiniKit
- **Framework:** Arduino (ESP32 Core 3.0.7+)
- **Protocol:** ESP-NOW (bi-directional)
- **License:** GPL

## Relationship to Other Projects

This is one of two transmitter variants. See parent folder for comparison.

**Flight Controller:**
`../../NEW_FPV_DRONE_Code-MultiWII_ESP32/MultiWii_Drone_FLT_CNTL_ESP32-ESPNOW/`

**Standard Transmitter:**
`../NEW_FPV_Transmitter_Code_ESP32_ESPNOW/`

## Data Structure Sync

When modifying `struct_message` or `struct_ack`, update ALL THREE files:

| Location | File | Lines |
|----------|------|-------|
| Flight Controller | `ESP_NOW_RX.cpp` | 17-38 |
| Standard TX | `NEW_FPV_Transmitter_Code_ESP32_ESPNOW.ino` | 76-95 |
| **This file** | `Transmitter_ESP32_No_Spring_Joy.ino` | 57-75 |

**Structures:**
- `struct_message` (7 bytes): throttle, yaw, pitch, roll, AUX1, AUX2, switches
- `struct_ack` (11 bytes): vbat, rssi, heading, pitch, roll, alt, flags

**Joystick Button Mapping:**
- `switches` bit0 (left button) → AUX3
- `switches` bit1 (right button) → AUX4 (can trigger BOXBEEPERON)

## Key Differences from Standard TX

| Feature | Standard | This Variant |
|---------|----------|--------------|
| Throttle | Center-based deadzone | Linear 0-255 |
| ADC | 10-bit | 12-bit |
| Smoothing | None | EMA (α=0.4) |
| Deadzone | Calibration-based | Fixed 30 units |

## Pin Safety Rules

**Safe ADC pins (input-only, use for joysticks):**
- GPIO 34, 35, 36, 39

**Safe GPIO pins (use for switches/buttons):**
- GPIO 4, 16, 26, 27

**Avoid these pins:**
- GPIO 0, 2, 5, 12, 15 - Strapping pins
- GPIO 6-11 - Connected to internal flash
