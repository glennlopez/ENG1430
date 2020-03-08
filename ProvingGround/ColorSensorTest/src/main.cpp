#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true

byte gammatable[256];
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Initial setup - only executes once
void setup() {
  Serial.begin(9600);
  Serial.println("Serial Color Output");

  if (tcs.begin()) {
  Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found");
    while (1); // halt!
  }

// RGB LED Setup
#if defined(ARDUINO_ARCH_ESP32)
  ledcAttachPin(redpin, 1);
  ledcSetup(1, 12000, 8);
  ledcAttachPin(greenpin, 2);
  ledcSetup(2, 12000, 8);
  ledcAttachPin(bluepin, 3);
  ledcSetup(3, 12000, 8);
#else
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
#endif

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

void loop() {
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

#if defined(ARDUINO_ARCH_ESP32)
  ledcWrite(1, gammatable[(int)red]);
  ledcWrite(2, gammatable[(int)green]);
  ledcWrite(3, gammatable[(int)blue]);
  
#else
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
#endif


}
