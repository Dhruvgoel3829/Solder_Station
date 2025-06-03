#include <Arduino.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include <RotaryEncoder.h>
#include <avr/sleep.h>
#include "main.h"

// Pins
#define SENSOR_PIN    A0        // tip temperature sense
#define VIN_PIN       A1        // input voltage sense
#define BUZZER_PIN    5         // buzzer
#define BUTTON_PIN    6         // rotary encoder switch
#define ROTARY_1_PIN  7         // rotary encoder 1
#define ROTARY_2_PIN  8         // rotary encoder 2
#define CONTROL_PIN   9         // heater MOSFET PWM control
#define REED_PIN      10        // Reed switch input
#define OLED_SCL      A5        // OLED I2C Clock
#define OLED_SDA      A4        // OLED I2C Data

// Temperature control values
#define TEMP_MIN      150       // min selectable temperature
#define TEMP_MAX      400       // max selectable temperature
#define TEMP_DEFAULT  320       // default start setpoint
#define TEMP_BOOST    50        // temperature increase in boost mode
#define TEMP_STEP     10        // rotary encoder temp change steps
#define TEMP_FINE     1         // fine adjustment step

// Default tip temperature calibration values
#define TEMP200       216       // temperature at ADC = 200
#define TEMP280       308       // temperature at ADC = 280
#define TEMP360       390       // temperature at ADC = 360

// ADC Error thresholds
#define ADC_MIN_VALID 50        // Minimum valid ADC reading
#define ADC_MAX_VALID 500       // Maximum valid ADC reading
#define ADC_ERROR_COUNT 2       // Reduced from 3 to 2 for faster detection
#define ADC_GND_THRESHOLD 10    // Threshold for GND connection detection
#define ADC_BOUNCE_THRESHOLD 20 // Reduced from 30 to 20 for faster bounce detection
#define ADC_BOUNCE_COUNT 3      // Reduced from 5 to 3 for faster bounce detection
#define TIP_DETECT_PULSE 1000   // Reduced from 2000 to 1000ms for faster tip detection
#define TIP_TEMP_THRESHOLD 15   // Reduced from 20 to 15 for faster tip detection

// Control values
#define TIME2SETTLE   950       // time in microseconds to allow OpAmp output to settle
#define SMOOTHIE      0.05      // OpAmp output smooth factor
#define LONG_PRESS    500       // long press threshold in milliseconds
#define NUM_READINGS  8         // number of analog readings to average

// MOSFET control values (N-channel)
#define HEATER_ON     0         // Full power for N-channel MOSFET
#define HEATER_OFF    255       // Off for N-channel MOSFET

// PID tuning parameters
double aggKp = 11.0, aggKi = 0.5, aggKd = 1.0;
double consKp = 11.0, consKi = 3.0, consKd = 5.0;

// Variables for temperature control
int16_t SetTemp = TEMP_DEFAULT;
double Input, Output, Setpoint;
int16_t RawTemp, CurrentTemp;    // Already int16_t

// Reed switch debouncing
#define REED_DEBOUNCE_TIME 50   // Debounce time in milliseconds
#define REED_STABLE_COUNT 3     // Number of stable readings required
#define TEMP_REDUCTION 0.5      // Temperature reduction factor when lying down
#define REED_HYSTERESIS 100     // Hysteresis time in milliseconds to prevent rapid switching

// State variables
bool inFineMode = false;
bool buttonPressed = false;
bool buttonLongPressed = false;
bool adcError = false;
bool checkingTip = false;
bool isGndConnected = false;
bool isIronLyingDown = false;   // New state for iron position
uint8_t adcErrorCount = 0;
const char* errorMessage = "";
uint32_t tipCheckStartTime = 0;
int16_t initialTemp = 0;

// Reed switch debouncing variables
uint32_t lastReedRead = 0;
uint32_t lastReedChange = 0;    // Track last state change time
bool lastReedState = false;
uint8_t reedStableCount = 0;
int16_t originalSetTemp = TEMP_DEFAULT;  // Store original temperature

// Variables for ADC bounce detection
int16_t lastAdcReadings[ADC_BOUNCE_COUNT];
uint8_t adcReadingIndex = 0;
bool isAdcBouncing = false;

// Timing variables
uint32_t buttonmillis;
uint32_t startTime = 0;  // Track when the device started

// Create PID controller
PID ctrl(&Input, &Output, &Setpoint, aggKp, aggKi, aggKd, REVERSE);

// Create rotary encoder
RotaryEncoder encoder(ROTARY_1_PIN, ROTARY_2_PIN);

// Create OLED display instance
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// Pre-defined strings to save RAM
const char str_set[] = "SET:";
const char str_fine[] = "FINE";
const char str_heat[] = "HEAT";
const char str_hold[] = "HOLD";
const char str_no_tip[] = "NO TIP";
const char str_adc_err[] = "ADC ERROR";
const char str_check[] = "Check tip";
const char str_conn[] = "connection";

void calculateTemp() {
  if (RawTemp < 200) {
    CurrentTemp = map(RawTemp, 0, 200, 21, TEMP200);
  } else if (RawTemp < 280) {
    CurrentTemp = map(RawTemp, 200, 280, TEMP200, TEMP280);
  } else {
    CurrentTemp = map(RawTemp, 280, 360, TEMP280, TEMP360);
  }
}

bool checkTip() {
  if (!checkingTip) {
    // Start tip check
    checkingTip = true;
    tipCheckStartTime = millis();
    initialTemp = CurrentTemp;
    analogWrite(CONTROL_PIN, HEATER_ON); // Full power for tip detection
    return false;
  } else {
    // Check if tip detection pulse is complete
    if (millis() - tipCheckStartTime >= TIP_DETECT_PULSE) {
      checkingTip = false;
      analogWrite(CONTROL_PIN, HEATER_OFF);
      delayMicroseconds(TIME2SETTLE);
      
      // Read temperature after pulse
      uint32_t sum = 0;
      for (int i = 0; i < NUM_READINGS; i++) {
        sum += analogRead(SENSOR_PIN);
        delayMicroseconds(100);
      }
      double finalTemp = sum / NUM_READINGS;
      
      // Calculate temperature rise
      double tempRise = finalTemp - initialTemp;
      
      if (tempRise < TIP_TEMP_THRESHOLD) {
        adcError = true;
        errorMessage = "NO TIP";  // Changed to use const char*
        return true;
      } else {
        adcError = false;
        return false;
      }
    }
    return false;
  }
}

void beepError() {
  // Triple beep for error
  for (uint8_t j = 0; j < 3; j++) {
    for (uint8_t i = 0; i < 255; i++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(125);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(125);
    }
    delay(100);
  }
}

bool checkADCError() {
  if (isAdcBouncing) {
    adcError = true;
    errorMessage = "TIP LOOSE";
    beepError();
    return true;
  }
  
  if (RawTemp < ADC_GND_THRESHOLD) {
    // Check for GND connection
    adcErrorCount++;
    if (adcErrorCount >= ADC_ERROR_COUNT) {
      adcError = true;
      isGndConnected = true;
      errorMessage = "GND ERROR";
      beepError();
      return true;
    }
  } else if (RawTemp < ADC_MIN_VALID) {
    // Check for no tip
    adcErrorCount++;
    if (adcErrorCount >= ADC_ERROR_COUNT) {
      return checkTip(); // Start tip detection
    }
  } else if (RawTemp > ADC_MAX_VALID) {
    // Check for ADC error
    adcErrorCount++;
    if (adcErrorCount >= ADC_ERROR_COUNT) {
      adcError = true;
      isGndConnected = false;
      errorMessage = "ADC ERROR";
      beepError();
      return true;
    }
  } else {
    adcErrorCount = 0;
    adcError = false;
    isGndConnected = false;
    return false;
  }
  return false;
}

void readTemperature() {
  analogWrite(CONTROL_PIN, HEATER_OFF);
  delayMicroseconds(TIME2SETTLE);
  
  uint32_t sum = 0;
  for (int i = 0; i < NUM_READINGS; i++) {
    sum += analogRead(SENSOR_PIN);
    delayMicroseconds(100);
  }
  int16_t temp = sum / NUM_READINGS;
  
  // Store reading for bounce detection
  lastAdcReadings[adcReadingIndex] = temp;
  adcReadingIndex = (adcReadingIndex + 1) % ADC_BOUNCE_COUNT;
  
  // Check for ADC bouncing
  if (adcReadingIndex == 0) {  // We have a full set of readings
    int16_t maxReading = lastAdcReadings[0];
    int16_t minReading = lastAdcReadings[0];
    for (int i = 1; i < ADC_BOUNCE_COUNT; i++) {
      if (lastAdcReadings[i] > maxReading) maxReading = lastAdcReadings[i];
      if (lastAdcReadings[i] < minReading) minReading = lastAdcReadings[i];
    }
    isAdcBouncing = (maxReading - minReading) > ADC_BOUNCE_THRESHOLD;
  }
  
  analogWrite(CONTROL_PIN, Output);
  
  RawTemp += (temp - RawTemp) * SMOOTHIE;
  
  if (!checkADCError()) {
    calculateTemp();
  }
}

void checkReedSwitch() {
  if (millis() - lastReedRead >= REED_DEBOUNCE_TIME) {
    bool currentReedState = !digitalRead(REED_PIN);  // Inverted because reed connects to ground
    
    if (currentReedState == lastReedState) {
      reedStableCount++;
      if (reedStableCount >= REED_STABLE_COUNT) {
        // Only process state change if enough time has passed since last change
        if (currentReedState != isIronLyingDown && 
            (millis() - lastReedChange) >= REED_HYSTERESIS) {
          isIronLyingDown = currentReedState;
          lastReedChange = millis();
          
          if (isIronLyingDown) {
            // Iron is lying down - reduce temperature
            originalSetTemp = SetTemp;
            SetTemp = (int16_t)(SetTemp * TEMP_REDUCTION);
            Setpoint = (double)SetTemp;
            beep();  // Single beep to indicate temperature reduction
          } else {
            // Iron is picked up - restore temperature and boost if needed
            SetTemp = originalSetTemp;
            Setpoint = (double)SetTemp;
            // Only boost if temperature is significantly below setpoint
            if (abs(CurrentTemp - SetTemp) > TEMP_BOOST) {
              SetTemp = TEMP_DEFAULT;
              Setpoint = (double)SetTemp;
              beep();  // Single beep to indicate temperature restoration
            }
          }
        }
      }
    } else {
      reedStableCount = 0;
    }
    
    lastReedState = currentReedState;
    lastReedRead = millis();
  }
}

void updateDisplay() {
  u8g2.clearBuffer();
  
  if (adcError) {
    // Display error state
    u8g2.setFont(u8g2_font_6x10_tf);
    
    // Center the error message
    int16_t width = u8g2.getStrWidth(errorMessage);
    u8g2.drawStr((128 - width) / 2, 25, errorMessage);
    
    // Display instructions based on error type
    u8g2.setFont(u8g2_font_5x7_tf);
    if (isGndConnected) {
      u8g2.drawStr(0, 45, "Check sensor");
      u8g2.drawStr(0, 55, "connection");
    } else if (strcmp(errorMessage, "TIP LOOSE") == 0) {
      u8g2.drawStr(0, 45, "Check tip");
      u8g2.drawStr(0, 55, "connection");
    } else if (strcmp(errorMessage, "NO TIP") == 0) {
      u8g2.drawStr(0, 45, "Check tip");
      u8g2.drawStr(0, 55, "connection");
    } else {
      u8g2.drawStr(0, 45, "Check wiring");
      u8g2.drawStr(0, 55, "and power");
    }
  } else {
    // Normal display
    u8g2.setFont(u8g2_font_6x10_tf);
    
    // Display set temperature
    u8g2.drawStr(0, 10, str_set);
    char tempStr[6];
    sprintf(tempStr, "%d", (int)Setpoint);
    u8g2.drawStr(30, 10, tempStr);

    // Display status
    const char* status;
    if (inFineMode) {
      status = str_fine;
    } else if (Output < 180) {
      status = str_heat;
    } else {
      // Calculate and display uptime
      uint32_t uptime = (millis() - startTime) / 1000; // Convert to seconds
      uint8_t hours = uptime / 3600;
      uint8_t minutes = (uptime % 3600) / 60;
      char timeStr[6];
      sprintf(timeStr, "%02d:%02d", hours, minutes);
      status = timeStr;
    }
    u8g2.drawStr(90, 10, status);

    // Display current temperature with larger font
    u8g2.setFont(u8g2_font_fub30_tf);
    sprintf(tempStr, "%d", (int)CurrentTemp);
    int16_t width = u8g2.getStrWidth(tempStr);
    u8g2.drawStr((128 - width) / 2, 45, tempStr);
    
    // Add degree symbol
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr((128 - width) / 2 + width + 2, 45, "C");

    // Display iron position status
    if (isIronLyingDown) {
      u8g2.drawStr(0, 63, "STANDBY");
    }
  }
  
  u8g2.sendBuffer();
}

void beep() {
  for (uint8_t i = 0; i < 255; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(125);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(125);
  }
}

void setup() {
  // Set pin modes
  pinMode(SENSOR_PIN, INPUT);
  pinMode(VIN_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CONTROL_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(REED_PIN, INPUT_PULLUP);  // Enable pull-up for reed switch
  
  // Initialize reed switch state
  lastReedState = !digitalRead(REED_PIN);
  isIronLyingDown = lastReedState;
  reedStableCount = REED_STABLE_COUNT;
  
  // Initialize heater and buzzer
  analogWrite(CONTROL_PIN, HEATER_OFF);
  digitalWrite(BUZZER_PIN, LOW);

  // Initialize display
  u8g2.begin();
  u8g2.enableUTF8Print();  // Enable UTF8 support for degree symbol
  u8g2.setContrast(255);   // Set maximum contrast

  // Setup ADC
  ADCSRA |= bit(ADPS0) | bit(ADPS1) | bit(ADPS2);
  ADCSRA |= bit(ADIE);
  interrupts();

  // Setup PID
  ctrl.SetOutputLimits(0, 255);
  ctrl.SetMode(AUTOMATIC);

  // Initialize temperature
  readTemperature();
  Setpoint = SetTemp;

  // Initialize start time
  startTime = millis();

  // Beep to indicate startup
  beep();
}

void loop() {
  // Check reed switch status
  checkReedSwitch();

  // Check rotary encoder
  encoder.tick();
  int newPos = encoder.getPosition();
  if (newPos != 0) {
    int step = inFineMode ? TEMP_FINE : TEMP_STEP;
    SetTemp = constrain(SetTemp + (newPos * step), TEMP_MIN, TEMP_MAX);
    encoder.setPosition(0);
  }

  // Check button for fine adjustment mode
  if (!digitalRead(BUTTON_PIN)) {
    if (!buttonPressed) {
      buttonPressed = true;
      buttonmillis = millis();
    } else if (!buttonLongPressed && (millis() - buttonmillis > LONG_PRESS)) {
      buttonLongPressed = true;
      if (!inFineMode) {
        inFineMode = true;
        beep();
        beep(); // Double beep to indicate fine mode
      }
    }
  } else {
    if (buttonPressed) {
      buttonPressed = false;
      buttonLongPressed = false;
    }
  }

  // Read temperature regularly
  static uint32_t lastTempRead = 0;
  if (millis() - lastTempRead >= 100) {  // Read every 100ms
    readTemperature();
    lastTempRead = millis();
  }

  // Normal operation
  Setpoint = (double)SetTemp;
  double gap = abs(Setpoint - CurrentTemp);
  if (gap < 30) {
    ctrl.SetTunings(consKp, consKi, consKd);
  } else {
    ctrl.SetTunings(aggKp, aggKi, aggKd);
  }
  Input = CurrentTemp;
  ctrl.Compute();
  analogWrite(CONTROL_PIN, Output);

  // Update display
  updateDisplay();
}

EMPTY_INTERRUPT(ADC_vect);
