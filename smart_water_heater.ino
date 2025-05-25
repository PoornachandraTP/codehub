#include <Arduino.h>

// --- Pin Definitions ---
#define NTC_PIN A0        // Analog pin for NTC Thermistor
#define HEATER_TRIAC_PIN 3 // Digital pin to control TRIAC
#define BUZZER_PIN 4      // Digital pin for Buzzer
#define HEATER_STATUS_LED_PIN 5 // Digital pin for heater status LED 

// --- Temperature Thresholds ---
const float OVERHEAT_TEMP_C = 90.0; // Turn off heater if temperature exceeds this
const float RESTART_TEMP_C = 60.0;  // Turn on heater if temperature drops below this

// --- NTC Thermistor Parameters 
// The value is for a common 10k NTC thermistor at 25°C
const float NOMINAL_RESISTANCE = 10000;   // Resistance at nominal temperature (25°C)
const float NOMINAL_TEMPERATURE = 25;     // Nominal temperature in Celsius
const float B_VALUE = 3950;               // B-value of the thermistor
const float SERIES_RESISTOR = 10000;      // Resistance of the series resistor in the voltage divider

// --- State Variables ---
bool heaterState = false; // true = ON, false = OFF
bool overheatDetected = false;

// --- Function Prototypes ---
float readTemperatureNTC();
void controlHeater(bool state);
void activateBuzzer(int duration);
void checkTemperatureLogic(float currentTemp);
void logData();

void setup() {
  Serial.begin(9600); // Initialize UART communication

  pinMode(HEATER_TRIAC_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(HEATER_STATUS_LED_PIN, OUTPUT);

  // Ensure heater is off initially
  controlHeater(false);

  Serial.println("Smart Heater Module Starting...");
  Serial.println("--------------------------------");
  Serial.println("Time (ms)\tTemperature (C)\tHeater State");
}

void loop() {
  float currentTemperature = readTemperatureNTC();

  // Implement heater control logic based on temperature
  checkTemperatureLogic(currentTemperature);

  // Update heater and LED based on current heaterState
  controlHeater(heaterState);

  // Log data via UART
  logData();

  delay(1000); // Read and process every 1 second
}

// --- Function Implementations ---

// Reads temperature from NTC thermistor using Steinhart-Hart approximation
float readTemperatureNTC() {
  int analogValue = analogRead(NTC_PIN);

  // Convert the analog reading to resistance
  float resistance = SERIES_RESISTOR / ((1023.0 / analogValue) - 1);

  // Calculate temperature using the Steinhart-Hart equation
  // 1/T = 1/T0 + (1/B) * ln(R/R0)
  // T = 1 / (1/T0 + (1/B) * ln(R/R0))
  float temperatureK = 1.0 / (1.0 / (NOMINAL_TEMPERATURE + 273.15) + (1.0 / B_VALUE) * log(resistance / NOMINAL_RESISTANCE));
  float temperatureC = temperatureK - 273.15; // Convert Kelvin to Celsius

  return temperatureC;
}

// Controls the TRIAC to turn the heater ON/OFF
void controlHeater(bool state) {
  if (state) {
    digitalWrite(HEATER_TRIAC_PIN, HIGH); // Turn on TRIAC driver (heater ON)
    digitalWrite(HEATER_STATUS_LED_PIN, HIGH); // Turn on status LED
  } else {
    digitalWrite(HEATER_TRIAC_PIN, LOW);  // Turn off TRIAC driver (heater OFF)
    digitalWrite(HEATER_STATUS_LED_PIN, LOW); // Turn off status LED
  }
}

// Activates the buzzer for a specified duration
void activateBuzzer(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

// Implements the temperature-based heater control logic
void checkTemperatureLogic(float currentTemp) {
  if (currentTemp > OVERHEAT_TEMP_C) {
    if (!overheatDetected) { // Only trigger alert once
      Serial.print("OVERHEAT ALERT! Temperature: ");
      Serial.print(currentTemp);
      Serial.println("C. Heater OFF.");
      activateBuzzer(500); // Long beep for overheat
      overheatDetected = true;
    }
    heaterState = false; // Force heater off
  } else if (currentTemp < RESTART_TEMP_C) {
    if (overheatDetected) { // If previously overheated, clear the flag
      Serial.print("Temperature dropped to ");
      Serial.print(currentTemp);
      Serial.println("C. Overheat cleared.");
      activateBuzzer(100); // Short beep when cleared
      overheatDetected = false;
    }
    // Only turn on if not currently in an overheat state
    if (!overheatDetected) {
      heaterState = true; // Turn heater on
    }
  }
  // If temperature is between RESTART_TEMP_C and OVERHEAT_TEMP_C, maintain current state
  // (unless an overheat just happened, then it stays off until below RESTART_TEMP_C)
}

// Logs data to the Serial Monitor (UART)
void logData() {
  Serial.print(millis()); // Time since start in milliseconds
  Serial.print("\t\t");
  Serial.print(readTemperatureNTC(), 2); // Temperature with 2 decimal places
  Serial.print("\t\t");
  Serial.println(heaterState ? "ON" : "OFF");
}