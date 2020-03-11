// Libs and header files
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <Arduino.h>

// GPIO Pin Definition
#define redpin      3     // PWM GPIO           - RGBLED
#define greenpin    5     // PWM GPIO           - RGBLED
#define bluepin     6     // PWM GPIO           - RGBLED
#define vacuumpin   10    // PWM GPIO           - Servo
#define zaxispin    9     // PWM GPIO           - Servo
#define blockpin    7     //                    - Microswitch
#define returnpin   2     // ISR Capable GPIO   - Microswitch

// Global Variables and Parametric Settings 
#define commonAnode true    // Change as per RGBLED type
int stepsPerCm = 1;         // Change as per stepper calibration
bool serialDebugger = false; // Serial Debugger (SLOWS DOWN RUNTIME - NOT FOR PRODUCTION)

byte gammatable[256];
float red, green, blue;                         // <Color Sensor Values>
uint16_t red_raw, green_raw, blue_raw, clear_raw;  // <Color Sensor Raw Values>
int blocks_homeState = 0;
int Servo1_pos = 0;
int Servo2_pos = 0;


// Color Block Samples @ 10cm
int toleranceBlockRGB = 5;
int yellowBlockRGB[] = {121, 81, 51};
int purpleBlockRGB[] = {90, 86, 78};
int redBlockRGB[] = {144, 61, 55};
int greenBlockRGB[] = {68, 115, 69};


// Function Prototypes
void Serial_Setup();
void AdafruitMotorShield_Setup();
void Stepper_Setup();
void ColorSensor_Setup();
void RGBLED_Setup();
void Servo_Setup();
void uSwitch_Setup();

void POST_Steppers();
void POST_RGBLED();
void POST_Servos();

void TCStoRGB_Output();
void RGBLED_Set(int red, int green, int blue);
void PushBack(int cm, int speed);
void Retract(int cm, int speed);
void VacuumServo(int pos, int slowdown);
void Z_AxisServo(int pos, int slowdown);

void HomeBlocks(int homeSpeed);


// OOP Object Initializations
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *Stepper1 = AFMS.getStepper(400, 1); // Carriage Stepper (MoveLeft, MoveRight)
Adafruit_StepperMotor *Stepper2 = AFMS.getStepper(400, 2); // Pushback Stepper (Pushback, Retract)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo Servo1, Servo2;


// ARDUINO SETUP
void setup() {
  /*-- Module Setup Routine --*/
  Serial_Setup();
  //AdafruitMotorShield_Setup();
  //Stepper_Setup();
  RGBLED_Setup();
  ColorSensor_Setup();
  //Servo_Setup();

  /*-- Visual Power-On-Self-Test Routine --*/
  //POST_Steppers();  delay(300);
  //POST_RGBLED();  delay(300);
  //POST_Servos();  delay(300);
}


// ARDUINO INFINITE LOOP
void loop() {

  bool yellowR_Range = (int(red) >= yellowBlockRGB[0] - toleranceBlockRGB) && (int(red) <= yellowBlockRGB[0] + toleranceBlockRGB);
  bool yellowG_Range = (int(green) >= yellowBlockRGB[1] - toleranceBlockRGB) && (int(green) <= yellowBlockRGB[1] + toleranceBlockRGB);
  bool yellowB_Range = (int(blue) >= yellowBlockRGB[2] - toleranceBlockRGB) && (int(blue) <= yellowBlockRGB[2] + toleranceBlockRGB);

  bool purpleR_Range = (int(red) >= purpleBlockRGB[0] - toleranceBlockRGB) && (int(red) <= purpleBlockRGB[0] + toleranceBlockRGB);
  bool purpleG_Range = (int(green) >= purpleBlockRGB[1] - toleranceBlockRGB) && (int(green) <= purpleBlockRGB[1] + toleranceBlockRGB);
  bool purpleB_Range = (int(blue) >= purpleBlockRGB[2] - toleranceBlockRGB) && (int(blue) <= purpleBlockRGB[2] + toleranceBlockRGB);

  bool redR_Range = (int(red) >= redBlockRGB[0] - toleranceBlockRGB) && (int(red) <= redBlockRGB[0] + toleranceBlockRGB);
  bool redG_Range = (int(green) >= redBlockRGB[1] - toleranceBlockRGB) && (int(green) <= redBlockRGB[1] + toleranceBlockRGB);
  bool redB_Range = (int(blue) >= redBlockRGB[2] - toleranceBlockRGB) && (int(blue) <= redBlockRGB[2] + toleranceBlockRGB);

  bool greenR_Range = (int(red) >= greenBlockRGB[0] - toleranceBlockRGB) && (int(red) <= greenBlockRGB[0] + toleranceBlockRGB);
  bool greenG_Range = (int(green) >= greenBlockRGB[1] - toleranceBlockRGB) && (int(green) <= greenBlockRGB[1] + toleranceBlockRGB);
  bool greenB_Range = (int(blue) >= greenBlockRGB[2] - toleranceBlockRGB) && (int(blue) <= greenBlockRGB[2] + toleranceBlockRGB);


  if ( yellowR_Range && yellowG_Range && yellowB_Range ){
    Serial.println("Yellow Detected");
  }

  if ( purpleR_Range && purpleG_Range && purpleB_Range ){
    Serial.println("Purple Detected");
  }

  if ( redR_Range && redG_Range && redB_Range ){
    Serial.println("Red Detected");
  }

  if ( greenR_Range && greenG_Range && greenB_Range ){
    Serial.println("Green Detected");
  }

  

  TCStoRGB_Output();
  //delay(2000);
  Serial.print("\n");

  






  



  

}













/*
 *  SERIAL OUTPUT SETUP SUBROUTINE
 *  Sets up the baudrate
 *  Serial port used mainly for debugging
 *  Credit: TEAM 211
 */
void Serial_Setup(){
  Serial.begin(9600);
  if (serialDebugger){
    Serial.println("Setup: Serial Initialized");
  }
}

/*
 *  ADAFRUIT MOTORSHIELD SETUP SUBROUTINE
 *  Initializes everything needed to get the Motorshield working properly
 *  Credit: Adafruit Library
 */
void AdafruitMotorShield_Setup(){
  AFMS.begin(); // default 1.6KHz frq
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  if (serialDebugger){
    Serial.println("Setup: Adafruit Library - AFSM started");
  }
}

/*
 *  STEPPER MOTOR SETUP SUBROUTINE
 *  Initializes Adafruit Motorshield and default speed for stepper motors
 *  Credit: Adafruit Library
 */
void Stepper_Setup(){
  // Initial Stepper motor speed
  Stepper1->setSpeed(10);   // default is 10 rpm   
  Stepper2->setSpeed(10);   // default is 10 rpm
  if (serialDebugger){
    Serial.println("Setup: Stepper motors speed set");
  }
}

/*
 *  TCS34725 SETUP SUBROUTINE
 *  Initializes and tests to see if TSC is found and running
 *  Credit: Adafruit Library
 */
void ColorSensor_Setup(){
  if (tcs.begin()) {
    if (serialDebugger){
      Serial.println("Found sensor");
    }
  } else {
    if (serialDebugger){
      Serial.println("No TCS34725 found");
    }
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
    if (serialDebugger){
      Serial.println(gammatable[i]);
    }
  }
  if (serialDebugger){
    Serial.println("Setup: RGBLED pins set to OUTPUT");
  }
}

/*
 * SERVO SETUP SUBROUTINE
 * Setup the pins for vacuum servo and z-axis servo
 * Credit: TEAM 211
 */
void Servo_Setup(){
  Servo1.attach(vacuumpin);
  Servo2.attach(zaxispin);
  if (serialDebugger){
    Serial.println("Setup: Servo pins have been software-attached");
  }
}

/*
 * MICROSWITCH SETUP ROUTINE
 * Sets microswitch pins as digital input pins
 * Credit: TEAM 211
 */
void uSwitch_Setup(){
  pinMode(blockpin, INPUT);
  pinMode(returnpin, INPUT);
  if (serialDebugger){
    Serial.println("Setup: Micro-switch pins set as INPUT");
  }
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
void POST_Steppers(){
  if (serialDebugger){
    Serial.println("Stepper Motor 1: Single Coil");
  }
  Stepper1->setSpeed(10);
  //Serial.println("Single coil steps");
  Stepper1->step(100, FORWARD, SINGLE); 
  Stepper1->step(100, BACKWARD, SINGLE); 

  if (serialDebugger){
    Serial.println("Stepper Motor 1: Double Coil");
  }
  Stepper1->setSpeed(50);
  //Serial.println("Double coil steps");
  Stepper1->step(100, FORWARD, DOUBLE); 
  Stepper1->step(100, BACKWARD, DOUBLE);
  
  if (serialDebugger){
    Serial.println("Stepper Motor 1: Interleave Mode");
  }
  Stepper1->setSpeed(10);
  //Serial.println("Interleave coil steps");
  Stepper1->step(50, FORWARD, INTERLEAVE); 
  Stepper1->step(50, BACKWARD, INTERLEAVE); 
  
  if (serialDebugger){
    Serial.println("Stepper Motor 1: Microstepping Mode");
  }
  Stepper1->setSpeed(100);
  //Serial.println("Microstep steps");
  Stepper1->step(20, FORWARD, MICROSTEP); 
  Stepper1->step(20, BACKWARD, MICROSTEP);


  if (serialDebugger){
    Serial.println("Stepper Motor 2: Single Coil");
  }
  Stepper2->setSpeed(10);
  //Serial.println("Single coil steps");
  Stepper2->step(100, FORWARD, SINGLE); 
  Stepper2->step(100, BACKWARD, SINGLE); 

  if (serialDebugger){
    Serial.println("Stepper Motor 2: Double Coil");
  }
  Stepper2->setSpeed(50);
  //Serial.println("Double coil steps");
  Stepper2->step(100, FORWARD, DOUBLE); 
  Stepper2->step(100, BACKWARD, DOUBLE);
  
  if (serialDebugger){
    Serial.println("Stepper Motor 2: Interleave Mode");
  }
  Stepper2->setSpeed(10);
  //Serial.println("Interleave coil steps");
  Stepper2->step(50, FORWARD, INTERLEAVE); 
  Stepper2->step(50, BACKWARD, INTERLEAVE); 
  
  if (serialDebugger){
    Serial.println("Stepper Motor 2: Microstepping Mode");
  }
  Stepper2->setSpeed(100);
  //Serial.println("Microstep steps");
  Stepper2->step(20, FORWARD, MICROSTEP); 
  Stepper2->step(20, BACKWARD, MICROSTEP);
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
  Servo1.write(0);  
  Servo2.write(0);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 90* | Servo(2): 180*");
  }
  Servo1.write(90);  
  Servo2.write(180);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 0* | Servo(2): 90*");
  }
  Servo1.write(0); 
  Servo2.write(90);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 180* | Servo(2): 0*");
  }
  Servo1.write(180);  
  Servo2.write(0);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 90* | Servo(2): 180*");
  }
  Servo1.write(135);  
  Servo2.write(45);
  delay(timeout);

  if (serialDebugger){
    Serial.println("Servo(1): 0* | Servo(2): 0*");
  }
  Servo1.write(0);  
  Servo2.write(0);
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
  tcs.getRawData(&red_raw, &green_raw, &blue_raw, &clear_raw);
  tcs.setInterrupt(true);  // turn off LED

  if (serialDebugger){
    // Serial Output of RGB values
    Serial.print("R:\t"); Serial.print(int(red)); 
    Serial.print("\tG:\t"); Serial.print(int(green)); 
    Serial.print("\tB:\t"); Serial.print(int(blue));
    Serial.print("\n");

    // Serial Output of RGB RAW values
    /*
    Serial.print("r:\t"); Serial.print(int(red_raw)); 
    Serial.print("\tg:\t"); Serial.print(int(green_raw)); 
    Serial.print("\tb:\t"); Serial.print(int(blue_raw));
    Serial.print("\tc:\t"); Serial.print(int(clear_raw));
    Serial.print("\n");
    */
  }
  

  // Output sensor vals to LED
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);
}

/*
 * PUSHBACK SUBROUTINE
 * Logic for moving carriage to the left, with distance and speed params 
 * Dependency: Adafruit_MotorShield.h
 * Credit: TEAM 211
 */
void PushBack(int cm, int speed){
  int stepToCM = cm * stepsPerCm;

  Stepper2->setSpeed(speed);
  Stepper2->step(stepToCM, FORWARD, DOUBLE); 
}

/*
 * RETRACT SUBROUTINE
 * Logic for moving carriage to the left, with distance and speed params 
 * Dependency: Adafruit_MotorShield.h
 * Credit: TEAM 211
 */
void Retract(int cm, int speed){
  int stepToCM = cm * stepsPerCm;

  Stepper2->setSpeed(speed);
  Stepper2->step(stepToCM, BACKWARD, DOUBLE); 
}

/*
 * PSUEDORANDOM HEX INVERTER-CONVERTER (color chart)
 * Used for debugging color tables
 */
void HexConversionTable(){
  /*
  while(1){
    74 68 65 72 65 20 61 72 65 20 6f 6e 6c 79 20 74 77 6f 20 74 79 
    70 65 73 20 6f 66 20 65 6e 67 69 6e 65 65 72 73 2e 20 74 68 6f 
    73 65 20 77 68 6f 20 63 72 65 61 74 65 20 70 72 6f 62 6c 65 6d 
    73 2c 20 61 6e 64 20 74 68 6f 73 65 20 77 68 6f 20 73 6f 6c 76 
    65 20 74 68 65 6d 2e
  }
  */
}


/*
 * HOME BLOCK SUBROUTINE
 * Logic for moving a block to be picked up at a known pos
 * Dependency: Adafruit_MotorShield.h
 * Credit: TEAM 211
 */ 
void HomeBlocks(int homeSpeed){
  do{
    blocks_homeState = digitalRead(blockpin);
    if(blocks_homeState == LOW){
      if (serialDebugger){
        Serial.println("Pushback: Block Positioned");
      }
      break;
    }
    PushBack(1, homeSpeed);
    if (serialDebugger){
      Serial.println("Pushback: Homing...");
    }
  }
  while(blocks_homeState == HIGH);
  Retract(800, 700);    
}

/*
 * VACUUM SERVO CONTROL LOGIC
 * Controls speed and pos of the vacuum servo
 * Params: Position and delay rate per degree of movement
 * Dependency: Servo.h
 * Credit: TEAM 211
 */ 
void VacuumServo(int pos, int slowdown){
  
  while(Servo1_pos < pos){
    Servo1_pos++;
    Servo1.write(Servo1_pos);
    delay(slowdown);
    if (Servo1_pos == pos){
      break;
    }
  }
  
  while(Servo1_pos > pos){
    Servo1_pos--;
    Servo1.write(Servo1_pos);
    delay(slowdown);
    if (Servo1_pos == pos){
      break;
    }
  }
}

/*
 * Z-AXIS SERVO CONTROL LOGIC
 * Controls speed and pos of the Z-AXIS servo
 * Params: Position and delay rate per degree of movement
 * Dependency: Servo.h
 * Credit: TEAM 211
 */ 
void Z_AxisServo(int pos, int slowdown){
  
  while(Servo2_pos < pos){
    Servo2_pos++;
    Servo2.write(Servo2_pos);
    delay(slowdown);
    if (Servo2_pos == pos){
      break;
    }
  }
  
  while(Servo2_pos > pos){
    Servo2_pos--;
    Servo2.write(Servo2_pos);
    delay(slowdown);
    if (Servo2_pos == pos){
      break;
    }
  }
}