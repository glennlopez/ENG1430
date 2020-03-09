#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>

// Color Sensor Pin Definitions
#define redpin 3
#define greenpin 5
#define bluepin 6
#define commonAnode true
byte gammatable[256];

// Adafruit Object Initializations
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2); // 200 steps per rev (M3 and M4)


// Function Prototypes
void Serial_Setup();
void RGBLED_Setup();
void TCStoRGB_Output();
void StepperSpeed_Set(int speed);


void setup() {
  Serial_Setup();
  RGBLED_Setup();
  StepperSpeed_Set(50);

}

void loop() {
  TCStoRGB_Output();

  Serial.println("Single coil steps");
  myMotor->step(100, FORWARD, SINGLE); 
  myMotor->step(100, BACKWARD, SINGLE); 

  Serial.println("Double coil steps");
  myMotor->step(100, FORWARD, DOUBLE); 
  myMotor->step(100, BACKWARD, DOUBLE);
  
  Serial.println("Interleave coil steps");
  myMotor->step(100, FORWARD, INTERLEAVE); 
  myMotor->step(100, BACKWARD, INTERLEAVE); 
  
  Serial.println("Microstep steps");
  myMotor->step(50, FORWARD, MICROSTEP); 
  myMotor->step(50, BACKWARD, MICROSTEP);
 
}





/*
 *  SERIAL OUTPUT SETUP ROUTINE
 *  Setup routine for serial output/debugging
 */
void Serial_Setup(){
  // Serial Port Initialization
  Serial.begin(9600);
  Serial.println("Serial Debugger Started");

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

/*
 * STEPPER MOTOR SPEED (in RPM)
 * Change the stepper motor speed in RPM
 */
void StepperSpeed_Set(int speed){
  AFMS.begin(); // default 1.6KHz frq
  myMotor->setSpeed(speed);  // default is 10 rpm   
}