// Libs and header files
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <Arduino.h>

// GPIO Pin Definition
#define redpin 3    // PWM
#define greenpin 5  // PWM
#define bluepin 6   // PWM

// Global Variables and Parametric Settings 
#define commonAnode true  // Change as per RGBLED type
int stepsPerCm = 1;       // Calibrate this

byte gammatable[256];
float red, green, blue;


// Function Prototypes
void Serial_Setup();
void AdafruitMotorShield_Setup();
void Stepper_Setup();
void ColorSensor_Setup();
void RGBLED_Setup();
void Servo_Setup();

void POST_StepperMotor();
void POST_RGBLED();
void POST_Servos();

void TCStoRGB_Output();
void MoveLeft(int cm, int speed);
void MoveRight(int cm, int speed);
void RGBLED_Set(int red, int green, int blue);


// OOP Object Initializations
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *Stepper1 = AFMS.getStepper(400, 2); // 0.9* stepping angle is 400 steps per revolution
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo vacuumServo, zServo;


// ARDUINO SETUP
void setup() {
  // Module Setup Routine
  //Serial_Setup();
  //AdafruitMotorShield_Setup();
  //Stepper_Setup();
  //ColorSensor_Setup();
  RGBLED_Setup();
  //Servo_Setup();

  // Power-On-Self-Test Routine
  //POST_StepperMotor();
  //delay(300);
  POST_RGBLED();
  //delay(300);

}

// ARDUINO INFINITE LOOP
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
  /*
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
  */




  

 


  

}
























/*
 *  SERIAL OUTPUT SETUP SUBROUTINE
 *  Sets up the baudrate
 *  Serial port used mainly for debugging
 *  Credit: TEAM 211
 */
void Serial_Setup(){
  Serial.begin(9600);
  Serial.println("Serial Initialized");
}

/*
 *  ADAFRUIT MOTORSHIELD SETUP SUBROUTINE
 *  Initializes everything needed to get the Motorshield working properly
 *  Credit: Adafruit Library
 */
void AdafruitMotorShield_Setup(){
  AFMS.begin(); // default 1.6KHz frq
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

/*
 *  STEPPER MOTOR SETUP SUBROUTINE
 *  Initializes Adafruit Motorshield and default speed for stepper motors
 *  Credit: Adafruit Library
 */
void Stepper_Setup(){
  // Initial Stepper motor speed
  Stepper1->setSpeed(10);  // default is 10 rpm   
}

/*
 *  TCS34725 SETUP SUBROUTINE
 *  Initializes and tests to see if TSC is found and running
 *  Credit: Adafruit Library
 */
void ColorSensor_Setup(){
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
    } 
    else {
      gammatable[i] = x;
    }
    //Serial.println(gammatable[i]);
  }
}

/*
 * SERVO SETUP SUBROUTINE
 * Setup the pins for vacuum servo and z-axis servo
 * Credit: TEAM 211
 */
void Servo_Setup(){
  vacuumServo.attach(9);
  zServo.attach(10);
}

/*
 *  RGBLED OUTPUT
 *  Manually changes the LED color
 *  If RGB is Common Anode, values are to be inverted (255 is 0)
 *  Credit: TEAM 211
 */
void RGBLED_Set(int red, int green, int blue){
  analogWrite(redpin, red);
  analogWrite(greenpin, green);
  analogWrite(bluepin, blue); 
}

/*
 * STEPPER MOTOR TEST SUBROUTINE
 * Runs Forward, Backward, Single, Interleave, and Microstep function
 * Dependency: Adafruit_MotorShield.h
 * Credit: Adafruit Library
 */
void POST_StepperMotor(){
  Stepper1->setSpeed(10);
  //Serial.println("Single coil steps");
  Stepper1->step(100, FORWARD, SINGLE); 
  Stepper1->step(100, BACKWARD, SINGLE); 

  Stepper1->setSpeed(50);
  //Serial.println("Double coil steps");
  Stepper1->step(100, FORWARD, DOUBLE); 
  Stepper1->step(100, BACKWARD, DOUBLE);
  
  Stepper1->setSpeed(10);
  //Serial.println("Interleave coil steps");
  Stepper1->step(50, FORWARD, INTERLEAVE); 
  Stepper1->step(50, BACKWARD, INTERLEAVE); 
  
  Stepper1->setSpeed(100);
  //Serial.println("Microstep steps");
  Stepper1->step(20, FORWARD, MICROSTEP); 
  Stepper1->step(20, BACKWARD, MICROSTEP);
}

/*
 * RGB LED TEST SUBROUTINE
 * Runs every diode in RGBLED
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
 * CARRIAGE MOVE-LEFT SUBROUTINE
 * Logic for moving carriage to the left, with distance and speed params 
 * Dependency: Adafruit_MotorShield.h
 * Credit: TEAM 211
 */
void MoveLeft(int cm, int speed){
  int stepToCM = cm * stepsPerCm;

  Stepper1->setSpeed(speed);
  Stepper1->step(stepToCM, FORWARD, SINGLE); 
}

/*
 * CARRIAGE MOVE-RIGHT SUBROUTINE
 * Logic for moving carriage to the left, with distance and speed params 
 * Dependency: Adafruit_MotorShield.h
 * Credit: TEAM 211
 */
void MoveRight(int cm, int speed){
  int stepToCM = cm * stepsPerCm;

  Stepper1->setSpeed(speed);
  Stepper1->step(stepToCM, BACKWARD, SINGLE); 
}