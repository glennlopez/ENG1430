#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Servo.h>

// GPIO Pin Definition
#define redpin 3    // PWM
#define greenpin 5  // PWM
#define bluepin 6   // PWM

// Global Variables and Definition 
#define commonAnode true
byte gammatable[256];
float red, green, blue;

// Adafruit Object Initializations
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *Stepper1 = AFMS.getStepper(400, 2); // 0.9* stepping angle is 400 steps per revolution
Servo vacuumServo, zServo;

// Function Prototypes
void Serial_Setup();
void RGBLED_Setup();
void POST();
void TCStoRGB_Output();
void POST_StepperMotor();
void POST_Servos();
void StepperSpeed_Set(int speed);
void Carriage_MoveLeft(int cm, int speed);
void Carriage_MoveRight(int cm, int speed);
void RGBLED_Set(int red, int green, int blue);
void POST_RGBLED();
void Servo_Setup();

// ARDUINO SETUP ROUTINE
void setup() {
  Serial_Setup(); // Setup Serial-Com
  RGBLED_Setup(); // Setup RGBLED GPIO
  Servo_Setup();
  POST();         // Power-On-Self-Test

}

// ARDUINO LOOP ROUTINE
void loop() {

  

 /*
 // TODO: Logic for RGBLED_Ident
 if ((red > 100 && red < 255) ){
   Serial.print("Looks like: RED");
   Serial.print("\n");  
   Carriage_MoveLeft(400, 200);
   
 }
 */

  // Debug Run
  TCStoRGB_Output();
  vacuumServo.write(0); 
  zServo.write(90);
  Carriage_MoveLeft(50, 200);
  delay(1000);
  vacuumServo.write(180);  
  zServo.write(0);
  Carriage_MoveRight(50, 200);
  delay(1000);
  vacuumServo.write(90);  
  zServo.write(180);
  delay(1000);
  

 


  

}





















/*
 * POWER-ON-SELF-TEST
 * Tests Stepper Motor, RGB, Color Sensor, Servos
 * Credit: TEAM 211 
 */
void POST(){
  POST_StepperMotor();
  POST_RGBLED();
}

/*
 * STEPPER MOTOR TEST ROUTINE
 * System check for stepper motor
 * Dependency: Adafruit_MotorShield.h
 * Credit: Adafruit Library
 */
void POST_StepperMotor(){
  Stepper1->setSpeed(10);
  Serial.println("Single coil steps");
  Stepper1->step(100, FORWARD, SINGLE); 
  Stepper1->step(100, BACKWARD, SINGLE); 

  Stepper1->setSpeed(50);
  Serial.println("Double coil steps");
  Stepper1->step(100, FORWARD, DOUBLE); 
  Stepper1->step(100, BACKWARD, DOUBLE);
  
  Stepper1->setSpeed(10);
  Serial.println("Interleave coil steps");
  Stepper1->step(50, FORWARD, INTERLEAVE); 
  Stepper1->step(50, BACKWARD, INTERLEAVE); 
  
  Stepper1->setSpeed(100);
  Serial.println("Microstep steps");
  Stepper1->step(20, FORWARD, MICROSTEP); 
  Stepper1->step(20, BACKWARD, MICROSTEP);
}


/*
 * RGB LED TEST ROUTINE
 * System check for stepper motor
 * Credit: TEAM 211
 */
void POST_RGBLED(){
  Serial.println("RGB LED Power-On-Self-Test");
  int timeout = 300; //in ms

  delay(timeout);
  RGBLED_Set(255, 255, 255);

  delay(timeout);
  RGBLED_Set(255, 0, 0);

  delay(timeout);
  RGBLED_Set(0, 255, 0);

  delay(timeout);
  RGBLED_Set(0, 0, 255);

  delay(timeout);
  RGBLED_Set(255, 255, 0);

  delay(timeout);
  RGBLED_Set(255, 0, 255);

  delay(timeout);
  RGBLED_Set(0, 255, 255);

  delay(timeout);
  RGBLED_Set(0, 0, 0);
  delay(timeout);
}

/*
 *  SERIAL OUTPUT SETUP ROUTINE
 *  Setup routine for serial output/debugging
 *  Credit: Adafruit Library
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
 *  Credit: Adafruit Library
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

    //Serial.println(gammatable[i]); // Comment out
  }
}

void Servo_Setup(){
  vacuumServo.attach(9);
  zServo.attach(10);
}

/*
 *  SET RGB LED OUTPUT
 *  If RGB is Common Anode, values are to be inverted (255 is 0)
 *  Credit: TEAM 211
 */
void RGBLED_Set(int red, int green, int blue){
  analogWrite(redpin, red);
  analogWrite(greenpin, green);
  analogWrite(bluepin, blue); 
}

/*
 *  TSC TO RGB OUTPUT
 *  Converts TSC sensor values to human-readable gamatable values
 *  (outputs the RGB vals to the RGBLED)
 *  Credit: Adafruit Library
 */
void TCStoRGB_Output(){

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
 * Credit: Adafruit Library
 */
void StepperSpeed_Set(int speed){
  AFMS.begin(); // default 1.6KHz frq
  Stepper1->setSpeed(speed);  // default is 10 rpm   
}

/*
 * CARRIAGE MOVE-LEFT SUBROUTINE
 * Logic for moving carriage to the left, with distance and speed params 
 * Dependency: Adafruit_MotorShield.h
 * Credit: TEAM 211
 */
void Carriage_MoveLeft(int cm, int speed){
  int stepsPerCm = 1; // TODO: Calibrate this as per Stepper motor
  int stepToCM = cm * stepsPerCm;

  StepperSpeed_Set(speed);
  Stepper1->step(stepToCM, FORWARD, SINGLE); 
}

/*
 * CARRIAGE MOVE-RIGHT SUBROUTINE
 * Logic for moving carriage to the left, with distance and speed params 
 * Dependency: Adafruit_MotorShield.h
 * Credit: TEAM 211
 */
void Carriage_MoveRight(int cm, int speed){
  int stepsPerCm = 1; // TODO: Calibrate this as per Stepper motor
  int stepToCM = cm * stepsPerCm;

  StepperSpeed_Set(speed);
  Stepper1->step(stepToCM, BACKWARD, SINGLE); 
}