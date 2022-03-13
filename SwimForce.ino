/**
 * MIT License
 * 
 * Copyright (c) 2021 thermalbarker
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <Arduino.h>
#include "HX711.h"
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

#include <Adafruit_BluefruitLE_SPI.h>
#include "src/BluefruitConfig/BluefruitConfig.h"

//#define DEBUG

#define APPEND_BUFFER(buffer,base,field) \
    memcpy(buffer+base,&field,sizeof(field)); \
    base += sizeof(field);

#define DOUT  3
#define CLK  2
byte boardLED = 6;
byte powerLED = 9;
byte buttonLED = 10;
HX711 scale;

Adafruit_7segment matrix = Adafruit_7segment();
Adafruit_7segment matrix2 = Adafruit_7segment();
Adafruit_24bargraph bar = Adafruit_24bargraph();

const int buttonPin = 5;

float calibration_factor = 2219; // Should be Newtons

float distance = 0.0;
float energy = 0.0;
long movingTime = 0;
long totalTime = 0;

long lastTime = 0;
long lastMeasure = 0;

bool simulate = true;

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

bool blueToothSetup = false;

/* The service information */

int32_t cscServiceId;
int32_t cscFeatureId;
int32_t cscMeasureId;
int32_t cscLocationId;
int32_t cscControlPointId;

int32_t pwrServiceId;
int32_t pwrFeatureId;
int32_t pwrMeasureId;
int32_t pwrLocationId;

bool BLEsetChar(Adafruit_BLE& ada_ble, uint8_t charID, uint8_t const data[], uint8_t size)
{
  uint16_t argtype[] = { AT_ARGTYPE_UINT8, (uint16_t) (AT_ARGTYPE_BYTEARRAY+ size) };
  uint32_t args[] = { charID, (uint32_t) data };

  return ada_ble.atcommand_full(F("AT+GATTCHAR"), NULL, 2, argtype, args);
}

bool setupBluetooth() {
  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    Serial.println(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    return false;
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (!ble.factoryReset() ){
       Serial.println(F("Couldn't factory reset"));
       return false;
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();


  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'SwimForce': "));

  if (!ble.sendCommandCheckOK(F("AT+GAPDEVNAME=SwimForce")) ) {
    Serial.print(F("Could not set device name?"));
    return false;
  }

  Serial.println(F("Adding the Cycling Service (UUID = 0x1816): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x1816"), &cscServiceId)) {
    Serial.println(F("Could not add HRM service"));
    return false;
  }

  Serial.println(F("Adding the CTC characteristic (UUID = 0x2A5C): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A5C, PROPERTIES=0x02, MIN_LEN=2, MAX_LEN=2, VALUE=0"), &cscFeatureId)) {
    Serial.println(F("Could not add CTC characteristic"));
    return false;
  }

  Serial.println(F("Adding the ctc characteristic (UUID = 0x2A5B): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A5B, PROPERTIES=0x12, MIN_LEN=1, MAX_LEN=11, VALUE=00-00-00-00-00-00-00-00-00-00-00"), &cscMeasureId)) {
    Serial.println(F("Could not add CTC characteristic"));
    return false;
  }

  Serial.println(F("Adding the ctc Location (UUID = 0x2A5D): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A5D, PROPERTIES=0x02, MIN_LEN=1, MAX_LEN=1, VALUE=0"), &cscLocationId)) {
    Serial.println(F("Could not add CTC characteristic"));
    return false;
  }

  Serial.println(F("Adding the ctc Location (UUID = 0x2A55): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A55, PROPERTIES=0x28, MIN_LEN=1, MAX_LEN=5, VALUE=0"), &cscControlPointId)) {
    Serial.println(F("Could not add CTC characteristic"));
    return false;
  }

   Serial.println(F("Adding the Cycling Power Service (UUID = 0x1818): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID=0x1818"), &pwrServiceId)) {
    Serial.println(F("Could not add HRM service"));
    return false;
  }

  Serial.println(F("Adding the power characteristic (UUID = 0x2A65): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A65, PROPERTIES=0x02, MIN_LEN=4, MAX_LEN=4, VALUE=0"), &pwrFeatureId)) {
    Serial.println(F("Could not add CTC characteristic"));
    return false;
  }

  Serial.println(F("Adding the power measurement characteristic (UUID = 0x2A63): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A63, PROPERTIES=0x12, MIN_LEN=4, MAX_LEN=14, VALUE=00-00-00-00-00-00"), &pwrMeasureId)) {
    Serial.println(F("Could not add CTC characteristic"));
    return false;
  }

  Serial.println(F("Adding the power Location (UUID = 0x2A5D): "));
  if (!ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A5D, PROPERTIES=0x02, MIN_LEN=1, MAX_LEN=1, VALUE=0"), &pwrLocationId)) {
    Serial.println(F("Could not add CTC characteristic"));
    return false;
  }

  if (!ble.sendCommandCheckOK(
            F("AT+GAPSETADVDATA="
              "02-01-06-"
              "02-0a-00-"
              "11-06-9e-ca-dc-24-0e-e5-a9-e0-93-f3-a3-b5-01-00-40-6e-"
              "05-02-18-18-16-18"
              ))) {
    Serial.print(F("Could not set Advertising data?"));
    return false;
  }

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  uint8_t data[2] = {0};
  uint8_t base = 0;

  uint16_t featureFlags = 0x0003;
  APPEND_BUFFER(data, base, featureFlags);
  BLEsetChar(ble, cscFeatureId, data, base);

  base = 0;
  uint8_t locationFlags = 0x03;
  APPEND_BUFFER(data, base, locationFlags);
  BLEsetChar(ble, cscLocationId, data, base);

  base = 0;
  APPEND_BUFFER(data, base, locationFlags);
  BLEsetChar(ble, pwrLocationId, data, base);

  base = 0;
  // Set bits 2,3,7 (rev data + accumulated energy)
  // Just do accumulated energy (bit 7)
  uint32_t powerFeatureFlags = 0x00000080;
  APPEND_BUFFER(data, base, powerFeatureFlags);
  BLEsetChar(ble, pwrFeatureId, data, base);

  Serial.println();

  return true;
}

uint32_t lastRevs = 0;
float timeToLastRev = 0.0;
uint16_t nPedalRevs = 0;
long lastCrankEvent = 0; // in ms

void writeBluetooth(long time, float distance, float power, float energy, bool stroke) {

  uint8_t data[11] = {0};
  uint8_t base = 0;

  // Need to transform velocity into wheel revolutions of an imaginary bike
  // Assume wheel has c = 2.340 m
  // Distance since last call
  const uint32_t c = 2340;
  uint32_t nRevs = (distance * 1000) / c; 

  if (nRevs > lastRevs) {
    float aveSpeed = (time > 0) ? (distance * 1.0e6) / time : 0;
    timeToLastRev = (nRevs * c) / aveSpeed; // in s
  }

  if (stroke) {
    nPedalRevs++;
    lastCrankEvent = time;
  }

  if (((blueToothSetup) && (nRevs > lastRevs)) || stroke) {

    // Convert to 1/1024 s units
    uint16_t lastWheelEvent = (uint16_t) (timeToLastRev * 1024);
    uint16_t crankTimeForBle = (uint16_t) (lastCrankEvent * 128 / 125);

#ifdef DEBUG
    Serial.print("Sending BLE CSC: nRevs: ");
    Serial.print(nRevs, 1);
    Serial.print(" timeToLastRev: ");
    Serial.print(timeToLastRev);
    Serial.print( " s ");
    Serial.print(lastWheelEvent);
    Serial.print( " 1/1024 s");
    Serial.print(" nCrankRevs: ");
    Serial.print(nPedalRevs);
    Serial.print(" timeToLastCrankRev: ");
    Serial.print(lastCrankEvent);
    Serial.print( " s ");
    Serial.print(crankTimeForBle);
    Serial.println( " 1/1024 s");
#endif
    const uint8_t flags = 0x03;


    APPEND_BUFFER(data, base, flags);
    APPEND_BUFFER(data, base, nRevs);
    APPEND_BUFFER(data, base, lastWheelEvent);
    APPEND_BUFFER(data, base, nPedalRevs);
    APPEND_BUFFER(data, base, crankTimeForBle);

    BLEsetChar(ble, cscMeasureId, data, base);

    // Now add power measurement
    base = 0;
    // flags 4,5,11 (revs + accumulated energy)
    // TODO: Check endianness
    uint16_t pwrFlags = 0x0800;
    APPEND_BUFFER(data, base, pwrFlags); // 16
    // Instantaneous power in W
    int16_t pwrInt = (int16_t) power;
    APPEND_BUFFER(data, base, pwrInt); // 16
    // Accumulated power (kJ)
    uint16_t eInt = (uint16_t) energy;
    APPEND_BUFFER(data, base, eInt); // 16

    BLEsetChar(ble, pwrMeasureId, data, base);

    lastRevs = nRevs;
  }

}

void setup() {
  Serial.begin(9600);
  Serial.println("Swim Force");

  matrix.begin(0x70);
  matrix2.begin(0x71);
  bar.begin(0x72);

  matrix2.print("Swim");
  matrix.print("Force");
  matrix.writeDisplay();
  matrix2.writeDisplay();

  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor);
  scale.tare(); //Reset the scale to 0

  pinMode(powerLED, OUTPUT);
  pinMode(buttonLED, OUTPUT);
  pinMode(boardLED, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  blueToothSetup = setupBluetooth();

  resetStrokeRate();

  testDisplays();
  // Reset the time
  lastMeasure = millis();
}

void testDisplays() {

  digitalWrite(powerLED, HIGH);
  digitalWrite(buttonLED, HIGH);

  for (int i = 1; i < 200; i++) {
    writeNumberToBarChart(((float) i) / 100);
    delay(10);
  }
  for (int i = 200; i >= 0; i--) {
    writeNumberToBarChart(((float) i) / 100);
    delay(10);
  }

  digitalWrite(powerLED, LOW);
  digitalWrite(buttonLED, LOW);

}

void writeNumberToBarChart(float num) {
  // Maximum number for the input
  const float maxNum = 2.0;
  // Maximum number of LEDs
  const int ledNum = 24;
  // LED boundaries
  const int green = 8;
  const int yellow = 16;
  const int red = ledNum;

  int maxToLightUp = (int) ((num / maxNum) * 24);

  for (int i = 0; i < ledNum; i++) {
    if (i < maxToLightUp) {
      if (i < green) {
        bar.setBar(i, LED_GREEN);
      } else if (i < yellow) {
        bar.setBar(i, LED_YELLOW);
      } else {
        bar.setBar(i, LED_RED);
      }
    } else {
      bar.setBar(i, LED_OFF);
    }
  }

  bar.writeDisplay();
}

long lastDisplay = 0;

void displayTime(long millis) {

    long minutes = millis / 60000;
    long seconds = (millis / 1000) - (minutes * 60);
    long displayNum = minutes * 100 + seconds;
    bool blinkColon = (seconds % 2) == 0;

    matrix2.print(displayNum, DEC);
    // Some zero padding
    if (displayNum < 1000) {
      matrix2.writeDigitNum(0, 0);
      if (displayNum < 100) {
        matrix2.writeDigitNum(1, 0);
        if (displayNum < 10) {
          // Seems like the colon is digit 2??
          matrix2.writeDigitNum(3, 0);
          if (displayNum < 1) {
            matrix2.writeDigitNum(4, 0);
          }
        }
      }
    }
    matrix2.drawColon(blinkColon);
    matrix2.writeDisplay();
}

float simulateForce(long t) {
    // Make a sinusoidal simulated force on top of a constant with some noise
    return 30.0 * sin(2.0 * 3.14159 * ((float)t / 500.0)) + 70.0 + random(0, 1);
}

/// Functions and variables for computing stroke rate
const int maxAveragePoints = 30; // corresponds to 3 seconds
float forceHistory[maxAveragePoints];
int lastPoint = 0;

void resetStrokeRate() {
  for (int i = 0; i < maxAveragePoints; i++) {
    forceHistory[i] = 0;
  }
  lastPoint = 0;
}

float getForceAt(int i = -1) {
  i = lastPoint + i;
  if (i < 0) {
    i = maxAveragePoints + i;
  }
  return forceHistory[i];
}

float updateMovingAverage(float forceNow) {
  forceHistory[lastPoint] = forceNow;
  lastPoint++;
  if (lastPoint >= maxAveragePoints) {
    lastPoint = 0;
  }
  float average = 0.0;
  for (int i = 0; i < maxAveragePoints; i++) {
    average += forceHistory[i];
  }
  average /= (float) maxAveragePoints; 
  return average;
}

bool computeStrokeRate(float forceNow, float &aveForce) {
  // The force from the last update
  float lastForce = getForceAt();
  aveForce = updateMovingAverage(forceNow);
  bool stroke = ((lastForce > aveForce) && (forceNow < aveForce));
  
#ifdef DEBUG
  Serial.print("ForceNow: ");
  Serial.print(forceNow, 2);
  Serial.print(" lastForce: ");
  Serial.print(lastForce, 2);
  Serial.print(" average: ");
  Serial.print(aveForce);
  Serial.print(" stroke: ");
  Serial.print(stroke);
  Serial.println();
#endif

  // This is the equivalent of zero crossing
  if (stroke) {
    // Invert LED
    if (digitalRead(boardLED) == LOW) {
      digitalWrite(boardLED, HIGH);
    } else {
      digitalWrite(boardLED, LOW);
    }
    return true;
  } else {
    return false;
  }
}

void loop() {

  // read the state of the pushbutton value:
  int buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    digitalWrite(buttonLED, HIGH);
    distance = 0;
    movingTime = 0;
    energy = 0;
    lastDisplay = 0;
    lastRevs = 0;
    timeToLastRev = 0.0;
    nPedalRevs = 0;
    lastCrankEvent = 0;
    scale.tare(); //Reset the scale to 0
    resetStrokeRate();
  } else {
    digitalWrite(buttonLED, LOW);
  }

  long deltaT = millis() - lastMeasure;
  if (deltaT > 100) {
    lastMeasure = millis();
    totalTime += deltaT;
    float force = simulate ? simulateForce(totalTime) : scale.get_units(); // Newtons

    // Force = 0.5 * rho * v^2 * C_D * A
    // v = K * sqrt(F)
    // K = sqrt(2/(rho * C_D * A))
    // rho = 997 kg/m^3 (Density of water)
    // C_D = between 0.5 - 1.0 (Drag constant)
    // A = 0.1 m^2 ?? (Cross section)
    // K = 0.1416 (Precompute constant K)
    // 
    // According to https://ftvs.cuni.cz/FTVS-2339-version1-the_determination_of_drag_in_front_crawl.pdf
    // F = k * v ** n
    // Empirically measure 1.5 < n < 2.5 and 16 < k < 25
    // and K = 1 / sqrt(k)
    // Assuming n = 2 to simplify calculations, then
    // 0.2 < K < 0.25 
    float velocity = 0.0;
    // Only measure if the force is positive
    // to avoid negative velocities
    if (force > 0.0) {
        velocity = 0.1416 * sqrt(force); // ms-1
    }
    float deltaD = velocity * deltaT * 1e-3; // m
    float power = velocity * force; // W
    if (power > 0.0) {
      energy += power * deltaT * 1e-6; // kJ
    }

    // Add a threshold for moving
    if (velocity > 0.1) {
      distance += deltaD; 
      movingTime += deltaT;
    }

    float aveForce = force;
    bool stroke = computeStrokeRate(force, aveForce);

    matrix.print(distance);
    matrix.writeDisplay();

    displayTime(movingTime);

    writeNumberToBarChart(velocity);

    writeBluetooth(movingTime, distance, power, energy, stroke);

#ifdef DEBUG
    Serial.print("Force: ");
    Serial.print(force, 1);
    Serial.print(" Velocity: ");
    Serial.print(velocity, 1);
    Serial.print(" DeltaT: ");
    Serial.print(deltaT, 1);
    Serial.print(" DeltaD: ");
    Serial.print(deltaD, 1);
    Serial.print(" distance: ");
    Serial.print(distance, 1);
    Serial.println();
#endif
  }

  //Each second blink the status LED
  if (millis() - lastTime > 1000)
  {
    lastTime = millis();
    if (digitalRead(powerLED) == LOW) {
      digitalWrite(powerLED, HIGH);
    } else {
      digitalWrite(powerLED, LOW);
    }
  
  }
}