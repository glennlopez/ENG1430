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
#define homepin 7   

// Global Variables and Parametric Settings 
#define commonAnode true    // Change as per RGBLED type
int stepsPerCm = 1;         // Calibrate this
bool serialDebugger = false; // Enables Serial debugger (slows down runtime)

byte gammatable[256];
float red, green, blue;     // <Color Sensor Values>
int homeState = 0;


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
void RGBLED_Set(int red, int green, int blue);
void MoveLeft(int cm, int speed);
void MoveRight(int cm, int speed);
void MoveHome(int homeSpeed);


// OOP Object Initializations
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *Stepper1 = AFMS.getStepper(400, 2); // 0.9* stepping angle is 400 steps per revolution
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo vacuumServo, zServo;


// ARDUINO SETUP
void setup() {
  // Module Setup Routine
  Serial_Setup();
  AdafruitMotorShield_Setup();
  Stepper_Setup();
  //RGBLED_Setup();
  //ColorSensor_Setup();
  //Servo_Setup();

  // Visual Power-On-Self-Test Routine
  //POST_StepperMotor();  delay(300);
  //POST_RGBLED();  delay(300);
  //POST_Servos();  delay(300);

}

// ARDUINO INFINITE LOOP
void loop() {
  //TCStoRGB_Output();

  

  MoveHome(50);
  MoveRight(1200, 500);
  delay(700);
  




  

 


  

}










/*
 * CARRIAGE HOMING SUBROUTINE
 * 
 */ 
void MoveHome(int homeSpeed){
  do{
    homeState = digitalRead(homepin);
    if(homeState == LOW){
      if (serialDebugger){
        Serial.println("Carriage is Home");
      }
      break;
    }
    MoveLeft(1, homeSpeed);
    //MoveRight(1, homeSpeed);
    if (serialDebugger){
      Serial.println("Waiting for carriage return");
    }
  }
  while(homeState == HIGH);    
}





/*
 * MICROSWITCH (HOME) SETUP ROUTINE
 * 
 */
void uSwitchHome_Setup(){
  pinMode(homepin, INPUT);
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
  if (serialDebugger){
    Serial.println("Stepper Motor: Single Coil");
  }
  Stepper1->setSpeed(10);
  //Serial.println("Single coil steps");
  Stepper1->step(100, FORWARD, SINGLE); 
  Stepper1->step(100, BACKWARD, SINGLE); 

  if (serialDebugger){
    Serial.println("Stepper Motor: Double Coil");
  }
  Stepper1->setSpeed(50);
  //Serial.println("Double coil steps");
  Stepper1->step(100, FORWARD, DOUBLE); 
  Stepper1->step(100, BACKWARD, DOUBLE);
  
  if (serialDebugger){
    Serial.println("Stepper Motor: Interleave Mode");
  }
  Stepper1->setSpeed(10);
  //Serial.println("Interleave coil steps");
  Stepper1->step(50, FORWARD, INTERLEAVE); 
  Stepper1->step(50, BACKWARD, INTERLEAVE); 
  
  if (serialDebugger){
    Serial.println("Stepper Motor: Microstepping Mode");
  }
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
  if (serialDebugger){
    Serial.println("RGB LED Power-On-Self-Test");
  }
  int timeout = 500; //in ms

  /* NOTE:
   * Common Anode inverts the value of RGB. The code below 
   * takes into account the two types of RGBLED avail
   * COTS (Commercial off the shelf)
   */

  if (commonAnode == true){
    // ON-OFF-ON-OFF
    RGBLED_Set(255, 255, 255);
    delay(timeout);
    RGBLED_Set(0, 0, 0);
    delay(timeout);
    RGBLED_Set(255, 255, 255);
    delay(timeout);
    RGBLED_Set(0, 0, 0);
    delay(timeout);
    RGBLED_Set(255, 255, 255);
    delay(timeout);

    // Test RED
    if (serialDebugger){
    Serial.println("RGBLED: Red");
    }
    RGBLED_Set(0, 255, 255);
    delay(timeout);

    // Test GREEN
    if (serialDebugger){
    Serial.println("RGBLED: Green");
    }
    RGBLED_Set(255, 0, 255);
    delay(timeout);

    // Test BLUE
    if (serialDebugger){
    Serial.println("RGBLED: Blue");
    }
    RGBLED_Set(255, 255, 0);
    delay(timeout);

    // Test WHITE
    if (serialDebugger){
    Serial.println("RGBLED: WHITE");
    }
    RGBLED_Set(0, 0, 0);
    delay(timeout);
  }
  else{
        // ON-OFF-ON-OFF
    RGBLED_Set(0, 0, 0);
    delay(timeout);
    RGBLED_Set(255, 255, 255);
    delay(timeout);
    RGBLED_Set(0, 0, 0);
    delay(timeout);
    RGBLED_Set(255, 255, 255);
    delay(timeout);
    RGBLED_Set(0, 0, 0);
    delay(timeout);

    // Test RED
    if (serialDebugger){
    Serial.println("RGBLED: Red");
    }
    RGBLED_Set(255, 0, 0);
    delay(timeout);

    // Test GREEN
    if (serialDebugger){
    Serial.println("RGBLED: Green");
    }
    RGBLED_Set(0, 255, 0);
    delay(timeout);

    // Test BLUE
    if (serialDebugger){
    Serial.println("RGBLED: Blue");
    }
    RGBLED_Set(0, 0, 255);
    delay(timeout);

    // Test WHITE
    if (serialDebugger){
    Serial.println("RGBLED: WHITE");
    }
    RGBLED_Set(255, 255, 255);
    delay(timeout);
  }
}

/*
 * SERVO TEST SUBROUTINE
 * Runs both servos in 90*, 180* 0* pos
 * Credit: TEAM 211
 */
void POST_Servos(){  
  int timeout = 1000;

  if (serialDebugger){
    Serial.println("Servo(1): 0* | Servo(2): 0*");
  }
  vacuumServo.write(0);  
  zServo.write(0);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 90* | Servo(2): 180*");
  }
  vacuumServo.write(90);  
  zServo.write(180);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 0* | Servo(2): 90*");
  }
  vacuumServo.write(0); 
  zServo.write(90);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 180* | Servo(2): 0*");
  }
  vacuumServo.write(180);  
  zServo.write(0);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 90* | Servo(2): 180*");
  }
  vacuumServo.write(135);  
  zServo.write(45);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 0* | Servo(2): 0*");
  }
  vacuumServo.write(0);  
  zServo.write(0);
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
  Stepper1->step(stepToCM, FORWARD, DOUBLE); 
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
  Stepper1->step(stepToCM, BACKWARD, DOUBLE); 
}