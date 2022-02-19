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
#include "Adafruit_LEDBackpack.h"


#define DOUT  3
#define CLK  2
byte statLED = 13; //On board status LED

HX711 scale;

Adafruit_7segment matrix = Adafruit_7segment();
Adafruit_7segment matrix2 = Adafruit_7segment();
Adafruit_24bargraph bar = Adafruit_24bargraph();

const int buttonPin = 4;

float calibration_factor = 2219; // Should be Newtons

float distance = 0.0;
long time = 0;

long lastTime = 0;
long lastMeasure = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Swim Force");
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor);
  scale.tare(); //Reset the scale to 0

  matrix.begin(0x70);
  matrix2.begin(0x71);
  bar.begin(0x72);

  pinMode(LED_BUILTIN, OUTPUT);

  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void writeNumberToBarChart(float num) {
  // Maximum number for the input
  const float maxNum = 1.0;
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
    distance = 0;
    time = 0;
    lastDisplay = 0;
    scale.tare(); //Reset the scale to 0
  }

  long deltaT = millis() - lastMeasure;
  if (deltaT > 100) {
    lastMeasure = millis();
    float force = scale.get_units();

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
    }

    matrix.print(distance);
    matrix.writeDisplay();

    displayTime(time);

    writeNumberToBarChart(velocity);

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
    if (digitalRead(LED_BUILTIN) == LOW) {
      digitalWrite(LED_BUILTIN, HIGH);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }
  
  }
}