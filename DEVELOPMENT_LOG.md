# Development Log

This document records development sessions for the ESP-NOW Transmitter (No-Spring Throttle Variant).

---

## Initial Version

**Features:**
- Linear throttle mapping (no center deadzone)
- 12-bit ADC resolution
- EMA input smoothing (α=0.4)
- Fixed deadzone (30 units) for Yaw/Pitch/Roll
- Throttle safety check at boot
- `__attribute__((packed))` structs for binary compatibility

---

## Session: January 16, 2026

**Developer:** jlmyra
**AI Assistant:** Claude Opus 4.5

### Summary

Added MAC address validation and documented joystick button functionality.

### Changes Made

#### 1. MAC Address Validation

**Problem:** Users often forget to configure the drone's MAC address

**Solution:** Added validation check in `setup()` that warns if MAC is still default (all 0xFF)

**Code Added (lines 206-222):**
```cpp
bool macIsDefault = true;
for (int i = 0; i < 6; i++) {
  if (RECEIVER_MAC[i] != 0xFF) {
    macIsDefault = false;
    break;
  }
}
if (macIsDefault) {
  Serial.println("\n*** WARNING: RECEIVER_MAC not configured! ***");
  lcd.print("SET DRONE MAC!");
  delay(3000);
}
```

#### 2. Joystick Button Documentation

**Feature:** Joystick buttons are transmitted in `switches` byte and mapped to AUX3/AUX4 on the flight controller

**Mapping:**
| Button | Switches Bit | RC Channel | Function |
|--------|--------------|------------|----------|
| Left joystick | bit0 | AUX3 | User-defined |
| Right joystick | bit1 | AUX4 | BOXBEEPERON (buzzer) |

**Usage:** Press right joystick button to activate buzzer (requires BOXBEEPERON configured on AUX4 HIGH)

---

## Data Structure Reference

### Control Data (TX → Drone) - 7 bytes

```cpp
typedef struct __attribute__((packed)) struct_message {
  uint8_t throttle;  // 0-255, linear mapping
  uint8_t yaw;       // 0-255, with 30-unit deadzone
  uint8_t pitch;     // 0-255, with 30-unit deadzone
  uint8_t roll;      // 0-255, with 30-unit deadzone
  uint8_t AUX1;      // 0 or 1
  uint8_t AUX2;      // 0 or 1
  uint8_t switches;  // bit0=left btn, bit1=right btn
} struct_message;
```

### Telemetry Data (Drone → TX) - 11 bytes

```cpp
typedef struct __attribute__((packed)) struct_ack {
  uint8_t vbat;       // Battery (0.1V units)
  uint8_t rssi;       // Signal strength (0-100%)
  int16_t heading;    // Compass heading
  int16_t pitch;      // Pitch angle
  int16_t roll;       // Roll angle
  int16_t alt;        // Altitude (cm)
  uint8_t flags;      // bit0=armed, bit1=angle, bit2=horizon, bit3=baro
} struct_ack;
```

---

## Future Considerations

- Joystick calibration wizard with EEPROM storage
- Adjustable deadzone via configuration
- Battery voltage display for transmitter itself
