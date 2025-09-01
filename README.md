## RC F1 Car Firmware (Arduino Uno)

Single-file Arduino firmware for a 1/10 RC F1 car. It reads two PWM channels
(throttle, steering) from an RC receiver and drives a brushless ESC + servo with
calibration, deadband, smoothing, expo, slew-rate limiting, and safety states.

## What this code does

- **RC PWM capture via interrupts** (D2 INT0 = throttle, D3 INT1 = steering).
- **State machine**: `DISARMED → ARMED → FAULT`.  
  - Arm only via button (D7) with throttle near neutral.  
  - Signal timeout → `FAULT` (neutral outputs until recovered).
- **Signal processing**: deadband → expo → EMA smoothing → slew-rate limiting.
- **Calibration mode** (hold arm button on boot): learns min/mid/max for each channel and stores to EEPROM.
- **Outputs at 50 Hz** using Servo lib: ESC on D10, steering on D9.
- **Status LED** on D13: slow blink (DISARMED), solid (ARMED), fast blink (FAULT).
- Optional **VBAT monitor** on A0 (simple divider).

## Pins (Uno)

| Function              | Pin |
|---------------------- |----:|
| Throttle (RC CH2)     | D2 (INT0) |
| Steering (RC CH1)     | D3 (INT1) |
| ESC signal            | D10 |
| Steering servo        | D9  |
| Arm button (→ GND)    | D7  |
| Status LED            | D13 |
| Battery sense (opt)   | A0  |

> Tie all grounds together (ESC/servo/receiver/Arduino). Power the servo/receiver from the ESC BEC (not the Arduino 5V).

## Build & Upload

- **Arduino IDE**: open `RC_F1_Firmware.ino`, select **Arduino Uno**, upload.
- **CLI**:
  ```bash
  arduino-cli core install arduino:avr
  arduino-cli compile -b arduino:avr:uno -e
  arduino-cli upload -b arduino:avr:uno -p <YOUR_COM_PORT>
