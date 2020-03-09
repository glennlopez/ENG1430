// Arduino Library
#include <Arduino.h>

// Color Sensor Library
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Color Sensor Pin Definitions
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
byte gammatable[256];

// TCS (Color Sensor) Object Initialization
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


// Function Prototypes
void Serial_Setup();
void RGBLED_Setup();
void TCStoRGB_Output();


void setup() {
  Serial_Setup();
  RGBLED_Setup();

}

void loop() {
  TCStoRGB_Output();
 
}





/*
 *  SERIAL OUTPUT SETUP ROUTINE
 *  Setup routine for serial output/debugging
 */
void Serial_Setup(){
  // Serial Port Initialization
  Serial.begin(9600);
  Serial.println("Serial Color Output");

  // Color Sensor Serial Output (debugger)
  if (tcs.begin()) {
  Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found");
    while (1);
  }
}

/*
 *  RGBLED SETUP ROUTINE (pin and logic)
 *  Setup routine RGBLED using gamatable conversion
 */
void RGBLED_Setup(){
    pinMode(redpin, OUTPUT);
    pinMode(greenpin, OUTPUT);
    pinMode(bluepin, OUTPUT);

  // RGB Human-readable-Gama-conversion
  for (int i=0; i<256; i++) {
    float x = i;
    x /= 255;
    x = pow(x, 2.5);
    x *= 255;

    if (commonAnode) {
      gammatable[i] = 255 - x;
    } else {
      gammatable[i] = x;
    }

    Serial.println(gammatable[i]);
  }
}

/*
 *  TSC TO RGB OUTPUT
 *  Converts TSC sensor values to human-readable gamatable values
 *  (outputs the RGB vals to the RGBLED)
 */
void TCStoRGB_Output(){
  float red, green, blue;
  tcs.setInterrupt(false);  // turn on LED

  // Read sensor input (takes 50ms to read)
  delay(60);  
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED

  // Serial Output of RGB values
  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print("\n");

  // Output sensor vals to LED
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
}