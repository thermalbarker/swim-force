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
#include "HX711.h"
#include <Wire.h> // Enable this line if using Arduino Uno, Mega, etc.
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>

#include <Adafruit_BluefruitLE_SPI.h>
#include "src/BluefruitConfig/BluefruitConfig.h"

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
long time = 0;

long lastTime = 0;
long lastMeasure = 0;

bool simulate = false;

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

  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
 // Serial.print(F("Adding Heart Rate Service UUID to the advertising payload: "));
 // ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );
  if (!ble.sendCommandCheckOK(
            F("AT+GAPSETADVDATA="
              "02-01-06-"
              "02-0a-00-"
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

  Serial.println();

  return true;
}

uint32_t lastRevs = 0;

void writeBluetooth(long time, float distance, float power) {

  uint8_t data[11] = {0};
  uint8_t base = 0;

  // Need to transform velocity into wheel revolutions of an imaginary bike
  // Assume wheel has c = 2.340 m
  // Distance since last call
  const uint32_t c = 2340;
  uint32_t nRevs = (distance * 1000) / c; 

  if ((blueToothSetup) && (nRevs > lastRevs)) {
    float aveSpeed = (time > 0) ? (distance * 1.0e6) / time : 0;
    float timeToLastRev = (nRevs * c) / aveSpeed; // in s
    // Convert to 1/1024 s units
    uint16_t lastWheelEvent = (uint16_t) (timeToLastRev * 1024);

    Serial.print("Sending BLE CSC: nRevs: ");
    Serial.print(nRevs, 1);
    Serial.print(" timeToLastRev: ");
    Serial.print(timeToLastRev);
    Serial.print( " s ");
    Serial.print(lastWheelEvent);
    Serial.println( " 1/1024 s");

    uint8_t flags = 0x03;
    uint16_t crankRevs = 0x0000;
    uint16_t lastCrankEvent = 0x0000;

    APPEND_BUFFER(data, base, flags);
    APPEND_BUFFER(data, base, nRevs);
    APPEND_BUFFER(data, base, lastWheelEvent);
    APPEND_BUFFER(data, base, crankRevs);
    APPEND_BUFFER(data, base, lastCrankEvent);

    BLEsetChar(ble, cscMeasureId, data, base);
    lastRevs = nRevs;
  }

}

void setup() {
  Serial.begin(9600);
  Serial.println("Swim Force");
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor);
  scale.tare(); //Reset the scale to 0

  matrix.begin(0x70);
  matrix2.begin(0x71);
  bar.begin(0x72);

  matrix.print("Swim");
  matrix2.print("Force");
  matrix.writeDisplay();
  matrix2.writeDisplay();

  pinMode(powerLED, OUTPUT);
  pinMode(buttonLED, OUTPUT);
  pinMode(boardLED, OUTPUT);

  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  blueToothSetup = setupBluetooth();

  delay(5000);
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

void loop() {

  // read the state of the pushbutton value:
  int buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    digitalWrite(buttonLED, HIGH);
    distance = 0;
    time = 0;
    lastDisplay = 0;
    scale.tare(); //Reset the scale to 0
  } else {
    digitalWrite(buttonLED, LOW);
  }

  long deltaT = millis() - lastMeasure;
  if (deltaT > 100) {
    lastMeasure = millis();
    float force = simulate ? random(50, 150) : scale.get_units();

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
        velocity = 0.1416 * sqrt(force);
    }
    float deltaD = velocity * deltaT * 1e-3;
    float power = velocity * force;

    // Add a threshold for moving
    if (velocity > 0.1) {
      distance += deltaD; 
      time += deltaT;
      digitalWrite(boardLED, HIGH);
    } else {
      digitalWrite(boardLED, LOW);
    }

    matrix.print(distance);
    matrix.writeDisplay();

    displayTime(time);

    writeNumberToBarChart(velocity);

    writeBluetooth(time, distance, power);

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