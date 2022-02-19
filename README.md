# swim-force
An arduino swimming force measurement project.

# Pre-requisites

## Arduino Development
You need to install the Arduino development environment for your system: https://www.arduino.cc/en/software

## Install 3rd Party Libraries

In the Arduino GUI, go to Sketch -> Include Library -> Manage Libraries. Then search for and install:

 * "Adafruit LED Backpack Library" (https://learn.adafruit.com/adafruit-led-backpack/)
 * "HX711 Arduino Library" (https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide/all)

# Build and Upload
Build and program the SwimForce.ino file for:

 * Arduino UNO Board
 * Arduino ISP programmer

# Pins

 * Digital Pins
   - D0: RX
   - D1: TX
   - D2: HX711 CLK
   - D3: HX711 DAT
   - D4: SPI RST
   - D5: Switch
   - D6: Board LED
   - D7: SPI IRQ
   - D8: SPI CS
   - D9: Power LED
   - D10: Switch LED
   - D11: SPI MOSI
   - D12: SPI MISO
   - D13: SPI SCK
 * Analog Pins
   - A0: 
   - A1: I2C CLK
   - A2: I2C DAT 
   - A3:   
   - A4:   
   - A5:   
