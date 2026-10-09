# Soldering Station Firmware

A DIY soldering station controller based on Arduino/ATmega328P with PID temperature control, rotary encoder interface, OLED display, and smart features like tip detection and iron position sensing.

## Features

- **PID Temperature Control** - Dual-mode PID (aggressive/conservative) for fast heat-up and stable temperature maintenance
- **Rotary Encoder Interface** - Adjust temperature with coarse/fine modes
- **OLED Display (SSD1306 128x64)** - Shows set temp, current temp, status, and uptime
- **Tip Detection** - Automatically detects if tip is connected/loose
- **Reed Switch Position Sensing** - Reduces temperature when iron is in stand (standby mode)
- **ADC Error Detection** - Detects ground faults, loose tips, and ADC bouncing
- **Buzzer Feedback** - Audio cues for mode changes, errors, and standby

## Hardware

- **MCU**: ATmega328P (Arduino Nano/Uno compatible)
- **Display**: SSD1306 128x64 OLED (I2C)
- **Temperature Sensor**: Thermocouple/RTD via OpAmp to ADC
- **Heater Control**: N-channel MOSFET (PWM)
- **Input**: Rotary encoder with push button
- **Position Sensor**: Reed switch (normally closed, opens when iron lifted)
- **Audio**: Piezo buzzer

## Pinout

| Function | Pin |
|----------|-----|
| Tip Temperature Sense | A0 |
| Input Voltage Sense | A1 |
| Buzzer | D5 |
| Rotary Encoder Button | D6 |
| Rotary Encoder A | D7 |
| Rotary Encoder B | D8 |
| Heater MOSFET PWM | D9 |
| Reed Switch | D10 |
| OLED SCL | A5 |
| OLED SDA | A4 |

## Temperature Calibration

The firmware uses a 3-point calibration curve. Adjust these values for your specific tip/sensor:

```cpp
#define TEMP200  216   // ADC reading at 200°C
#define TEMP280  308   // ADC reading at 280°C
#define TEMP360  390   // ADC reading at 360°C
```

## Building & Flashing

### PlatformIO (Recommended)
```bash
cd v1/pio/soldering\ station
pio run -t upload
```

### Arduino IDE
1. Open `v1/pio/soldering station/src/main.cpp`
2. Install libraries: `RotaryEncoder`, `PID`, `U8g2`
3. Select board: Arduino Uno
4. Upload

## Configuration

Key settings in `main.cpp`:

```cpp
#define TEMP_MIN      150   // Minimum selectable temperature (°C)
#define TEMP_MAX      400   // Maximum selectable temperature (°C)
#define TEMP_DEFAULT  320   // Startup temperature (°C)
#define TEMP_BOOST    50    // Boost temperature when picking up iron (°C)
#define TEMP_STEP     10    // Coarse adjustment step (°C)
#define TEMP_FINE     1     // Fine adjustment step (°C)
```

### PID Tuning
```cpp
// Aggressive (large temperature difference)
double aggKp = 11.0, aggKi = 0.5, aggKd = 1.0;

// Conservative (near setpoint)
double consKp = 11.0, consKi = 3.0, consKd = 5.0;
```

## Usage

1. **Power on** - Beeps once, shows current/set temperature
2. **Rotate encoder** - Adjust set temperature (10°C steps)
3. **Long press encoder** - Enter fine mode (1°C steps, double beep)
4. **Place iron in stand** - Temperature reduces by 50% (single beep, shows "STANDBY")
5. **Lift iron** - Restores temperature, boosts if cold (single beep)

## Error Codes

| Display | Meaning | Action |
|---------|---------|--------|
| NO TIP | Tip not detected | Check tip connection |
| TIP LOOSE | Tip connection unstable | Reseat tip |
| GND ERROR | Sensor shorted to ground | Check wiring |
| ADC ERROR | ADC reading out of range | Check sensor/wiring |

## License

MIT License - See LICENSE file for details.

## Credits

- PID library by Brett Beauregard
- RotaryEncoder library by Matthias Hertel
- U8g2 library by Oliver Kraus