/*
 * Change Boardtype [AlienHead] > Projects & Config.. > Projects > Configure 
 * Proving ground for working with vscode as an IDE instead of vanila arduino ide
 * Also tests PlatforIO compatability withh diffrent arduino bootloaders
 */

#include <Arduino.h>

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a second
  delay(500);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a second
  delay(500);
}
