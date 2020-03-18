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
#define blockpin    7     //                    - Microswitch (Blocks)
#define returnpin   2     // ISR Capable GPIO   - Microswitch (Carriage)

// Significant Parametric X-AXIS Locations
#define FirstTile_pos   15
#define SecondTile_pos  65
#define ThirdTile_pos   115
#define ForthTile_pos   165

// Change as per RGBLED type
#define commonAnode true 

// Global Variables and Parametric Settings 
float stepsPerMM = 5.5;         // Change as per stepper calibration
bool serialDebugger = true; // Serial Debugger (SLOWS DOWN RUNTIME - NOT FOR PRODUCTION)

byte gammatable[256];
float red, green, blue;  // <Color Sensor Values>
int BlockColor_Scanned = 0;
//uint16_t red_raw, green_raw, blue_raw, clear_raw;  // <Color Sensor Raw Values>                             

int blocks_homeState = 0;
int carriage_homeState = 0;
int Servo1_pos = 0;
int Servo2_pos = 0;
int Carriage_pos = 0;


/* 
 * COLOR TABLE @ 10mm (indoor environment)
 * Calibration: use function TCStoRGB_Output() in loop() to get sensor rgb values
 * Adjust Tolerence as required
 * Credit: TEAM 211
 */
int toleranceBlockRGB   = 5;
/* BRIGHT ROOM (OFFICE) 
int yellowBlockRGB[]    = {122, 83, 49};    // Known YELLOW RGB PARAM
int purpleBlockRGB[]    = {86, 85, 85};     // Known PURPLE RGB PARAM
int redBlockRGB[]       = {152, 55, 55};    // Known RED RGB PARAM
int greenBlockRGB[]     = {60, 124, 68};    // Known GREEN RGB PARAM

int yellowTileRGB[]     = {117, 92, 44};
int purpleTileRGB[]     = {113, 68, 78};
int redTileRGB[]        = {164, 55, 45};
int greenTileRGB[]      = {87, 103, 67};
*/


/* DARK ROOM (BASEMENT) */
int yellowBlockRGB[]    = {121, 82, 49};    // Known YELLOW RGB PARAM
int purpleBlockRGB[]    = {81, 84, 86};     // Known PURPLE RGB PARAM
int redBlockRGB[]       = {149, 54, 56};    // Known RED RGB PARAM
int greenBlockRGB[]     = {61, 119, 70};    // Known GREEN RGB PARAM

int yellowTileRGB[]     = {113, 94, 41};
int purpleTileRGB[]    = {106, 68, 82};
int redTileRGB[]        = {161, 55, 45};
int greenTileRGB[]      = {78, 106, 69};


// (DO NOT EDIT) - Stores Tile Information
int tileData[4][2] = {
  {FirstTile_pos, 0},
  {SecondTile_pos, 0},
  {ThirdTile_pos, 0},
  {ForthTile_pos, 0}
};  

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
void VacuumServo(int pos, int slowdown);
void Z_AxisServo(int pos, int slowdown);
void TCS_SerialOut();

void HomeCarriage();
void PushBack(int cm, int speed);
void Retract(int cm, int speed);
void HomeCarriage(int homeSpeed);
void GoTo(int cm, int speed);

void GoTo_BlockTileMatch();

int BlockColorAcquisition();
int TileColorAcquisition();
void TileColorPosAcquisition();

void debugloop();


// OOP Object Initializations
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *Stepper1 = AFMS.getStepper(400, 1); // Carriage Stepper (MoveLeft, MoveRight)
Adafruit_StepperMotor *Stepper2 = AFMS.getStepper(400, 2); // Pushback Stepper (Pushback, Retract)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Servo Servo1, Servo2;

bool done = false;  // Loops into an infinite subroutine when cube sorter job is done
bool TileNotScanned = true;


// ARDUINO SETUP
void setup() {
  /*-- Module Setup Routine --*/
  Serial_Setup();
  AdafruitMotorShield_Setup();
  Stepper_Setup();
  RGBLED_Setup();
  ColorSensor_Setup();
  Servo_Setup();

  /*-- Visual Power-On-Self-Test Routine --*/
  //POST_Steppers();  delay(300);
  //POST_RGBLED();  delay(300);
  //POST_Servos();  delay(300);


}

// TODO: acquire tile color coords
void ScanTiles(){
  int scanDelay = 10;

  GoTo(FirstTile_pos,300);
  tileData[0][1] = TileColorAcquisition();
  TCStoRGB_Output(); //debug
  delay(scanDelay);

  GoTo(SecondTile_pos,300);
  tileData[1][1] = TileColorAcquisition();
  TCStoRGB_Output(); //debug
  delay(scanDelay);

  GoTo(ThirdTile_pos,300);
  tileData[2][1] = TileColorAcquisition();
  TCStoRGB_Output(); //debug
  delay(scanDelay);
  
  GoTo(ForthTile_pos,300);
  tileData[3][1] = TileColorAcquisition();
  TCStoRGB_Output(); //debug
  delay(scanDelay);

  // Set TileNotScanned to false when done scanning
  TileNotScanned = false;

}

// TODO:  
void PickAndPlace(){
  // push cubes home
  // x-carriage: go to last cube coord
  // scan cube color
  // pick cube up
  // x-carriage: go to matching tile color

  // check to see if no more cube (done if no more cube)

}


// ARDUINO INFINITE LOOP
void loop() {

  /*
   * WHERE IS ALL YOUR CODE?!
   * Oh Hai! This is called "clean" code. Each Function is focused, un-poluted, 
   * and only takes 1 second to understand what loop() should be doing.  
   * If you need to understand how each subroutine works, simply follow the
   * bread-crumbs (function calls inside each function call). 
   */
  
    /*
  if(TileNotScanned){
    ScanTiles();  
  }

  PickAndPlace();

  while(done);{ // Finished Sorting
    HomeCarriage();
  }
  */








  /*
  TCStoRGB_Output();
  BlockColorAcquisition();
  TileColorAcquisition();
  */

 
  /* TEST TILE COLOR POSITION QCQUISITION LOGIC 
  delay(2000); // debug delay
  //TCStoRGB_Output(); //DEBUG
  //TileColorAcquisition();

  //DEBUG - simulate carriage movement
  if (BlockColorAcquisition() == 1){
    Carriage_pos = 100; 
  }

  if (BlockColorAcquisition() == 2){
    Carriage_pos = 200; 
  }

  if (BlockColorAcquisition() == 3){
    Carriage_pos = 300; 
  }

  if (BlockColorAcquisition() == 4){
    Carriage_pos = 400; 
  }



  // TileColorPosAcquisition();
  // RECORD TILE DATA COLOR AND COORD
  // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  TileColorPosAcquisition();

  


  // DEBUG TILE OUTPUT
  if (serialDebugger){
      
      Serial.println();
      Serial.print("First Location: "); Serial.println(tileData[0][0]);
      Serial.print("Color Code: "); Serial.println(tileData[0][1]);
      Serial.println();

      Serial.println();
      Serial.print("Second Location: "); Serial.println(tileData[1][0]);
      Serial.print("Color Code: "); Serial.println(tileData[1][1]);
      Serial.println();

      Serial.println();
      Serial.print("Third Location: "); Serial.println(tileData[2][0]);
      Serial.print("Color Code: "); Serial.println(tileData[2][1]);
      Serial.println();

      Serial.println();
      Serial.print("Forth Location: "); Serial.println(tileData[3][0]);
      Serial.print("Color Code: "); Serial.println(tileData[3][1]);
      Serial.println();
  }  
  END OF TEST TILE COLOR POSITION QCQUISITION LOGIC  */



  /*



  // COLOR ACQUISITION SIMULATION
  tileData[0][1] = 2;   // Green
  tileData[1][1] = 3;   // Purple
  tileData[2][1] = 1;   // Red
  tileData[3][1] = 4;   // Yellow

    // DEBUG TILE OUTPUT
  if (serialDebugger){
      
      Serial.println();
      Serial.print("First Location: "); Serial.println(tileData[0][0]);
      Serial.print("Color Code: "); Serial.println(tileData[0][1]);
      Serial.println();

      Serial.println();
      Serial.print("Second Location: "); Serial.println(tileData[1][0]);
      Serial.print("Color Code: "); Serial.println(tileData[1][1]);
      Serial.println();

      Serial.println();
      Serial.print("Third Location: "); Serial.println(tileData[2][0]);
      Serial.print("Color Code: "); Serial.println(tileData[2][1]);
      Serial.println();

      Serial.println();
      Serial.print("Forth Location: "); Serial.println(tileData[3][0]);
      Serial.print("Color Code: "); Serial.println(tileData[3][1]);
      Serial.println();
  }  
  delay(1000);

  BlockColor_Scanned = BlockColorAcquisition();
  GoTo_BlockTileMatch();

  */

  //debugloop();
  //delay(1000);

  /*
  Z_AxisServo(45, 20);
  delay(1000);
  Z_AxisServo(80, 20);
  delay(1000);
  */
  

  //GoTo(350, 900);

  //BlockColorAcquisition();

  debugloop();



}


void debugloop(){

  delay(3000);
  HomeCarriage();


  // SCAN BLOCKS
  ScanTiles();




  // GO TO BLOCK -- THEN GO TO DROP OFF LOCATION
  GoTo(350, 400);

  if(BlockColorAcquisition() == tileData[0][1]){
    GoTo(tileData[0][0], 400);
  }
  if(BlockColorAcquisition() == tileData[1][1]){
    GoTo(tileData[1][0], 400);
  }
  if(BlockColorAcquisition() == tileData[2][1]){
    GoTo(tileData[2][0], 400);
  }
  if(BlockColorAcquisition() == tileData[3][1]){
    GoTo(tileData[3][0], 400);
  }


  delay(1000);



  // go home faster
  GoTo(10, 400);

  

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
  GoTo(10,400);
  HomeCarriage();
  GoTo(100,400);
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

  GoTo(10, 600);
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
  //tcs.getRawData(&red_raw, &green_raw, &blue_raw, &clear_raw);
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
  int stepToCM = cm * stepsPerMM;

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
  int stepToCM = cm * stepsPerMM;

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

/*
 *  COLOR ACQUISITION FOR BLOCK
 *  Takes a known color from an array and compares with sensors RGB
 *  The Array with RGB should be calibrated
 *  Returns <int>
 *  Credit: TEAM 211
 */
int BlockColorAcquisition(){
  int colorCodeValue = 0;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  tcs.setInterrupt(false);  // turn on LED

  // Read sensor input (takes 50ms to read)
  delay(60);  
  tcs.getRGB(&red, &green, &blue);
  //tcs.getRawData(&red_raw, &green_raw, &blue_raw, &clear_raw);
  tcs.setInterrupt(true);  // turn off LED


  // Yellow Truth Range
  bool yellowR_Range = (int(red) >= yellowBlockRGB[0] - toleranceBlockRGB) && (int(red) <= yellowBlockRGB[0] + toleranceBlockRGB);
  bool yellowG_Range = (int(green) >= yellowBlockRGB[1] - toleranceBlockRGB) && (int(green) <= yellowBlockRGB[1] + toleranceBlockRGB);
  bool yellowB_Range = (int(blue) >= yellowBlockRGB[2] - toleranceBlockRGB) && (int(blue) <= yellowBlockRGB[2] + toleranceBlockRGB);

  // Purple Truth Range
  bool purpleR_Range = (int(red) >= purpleBlockRGB[0] - toleranceBlockRGB) && (int(red) <= purpleBlockRGB[0] + toleranceBlockRGB);
  bool purpleG_Range = (int(green) >= purpleBlockRGB[1] - toleranceBlockRGB) && (int(green) <= purpleBlockRGB[1] + toleranceBlockRGB);
  bool purpleB_Range = (int(blue) >= purpleBlockRGB[2] - toleranceBlockRGB) && (int(blue) <= purpleBlockRGB[2] + toleranceBlockRGB);

  // Red Truth Range
  bool redR_Range = (int(red) >= redBlockRGB[0] - toleranceBlockRGB) && (int(red) <= redBlockRGB[0] + toleranceBlockRGB);
  bool redG_Range = (int(green) >= redBlockRGB[1] - toleranceBlockRGB) && (int(green) <= redBlockRGB[1] + toleranceBlockRGB);
  bool redB_Range = (int(blue) >= redBlockRGB[2] - toleranceBlockRGB) && (int(blue) <= redBlockRGB[2] + toleranceBlockRGB);

  // Green Truth Range
  bool greenR_Range = (int(red) >= greenBlockRGB[0] - toleranceBlockRGB) && (int(red) <= greenBlockRGB[0] + toleranceBlockRGB);
  bool greenG_Range = (int(green) >= greenBlockRGB[1] - toleranceBlockRGB) && (int(green) <= greenBlockRGB[1] + toleranceBlockRGB);
  bool greenB_Range = (int(blue) >= greenBlockRGB[2] - toleranceBlockRGB) && (int(blue) <= greenBlockRGB[2] + toleranceBlockRGB);

  // Test for Red
  if ( redR_Range && redG_Range && redB_Range ){
    if (serialDebugger){
      Serial.println("Red Block Detected");
      TCS_SerialOut();
    }
    colorCodeValue = 1;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  }

  // Test for Green
  if ( greenR_Range && greenG_Range && greenB_Range ){
    if (serialDebugger){
      Serial.println("Green Block Detected");
      TCS_SerialOut();
    }
    colorCodeValue = 2;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  }

  // Test for Purple
  if ( purpleR_Range && purpleG_Range && purpleB_Range ){
    if (serialDebugger){
      Serial.println("Purple Block Detected");
      TCS_SerialOut();
    }
    colorCodeValue = 3;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  }

  // Test for Yellow
  if ( yellowR_Range && yellowG_Range && yellowB_Range ){
    if (serialDebugger){
      Serial.println("Yellow Block Detected");
      TCS_SerialOut();
    }
    colorCodeValue = 4;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  }

  // Output sensor vals to LED
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);

  return colorCodeValue;

}

/*
 *  COLOR ACQUISITION FOR TILE
 *  Takes a known color from an array and compares with sensors RGB
 *  The Array with RGB should be calibrated
 *  Returns <int>
 *  Credit: TEAM 211
 */
int TileColorAcquisition(){
  int colorCodeValue = 0;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  tcs.setInterrupt(false);  // turn on LED

  // Read sensor input (takes 50ms to read)
  delay(60);  
  tcs.getRGB(&red, &green, &blue);
  //tcs.getRawData(&red_raw, &green_raw, &blue_raw, &clear_raw);
  tcs.setInterrupt(true);  // turn off LED


  // Yellow Truth Range
  bool yellowR_Range = (int(red) >= yellowTileRGB[0] - toleranceBlockRGB) && (int(red) <= yellowTileRGB[0] + toleranceBlockRGB);
  bool yellowG_Range = (int(green) >= yellowTileRGB[1] - toleranceBlockRGB) && (int(green) <= yellowTileRGB[1] + toleranceBlockRGB);
  bool yellowB_Range = (int(blue) >= yellowTileRGB[2] - toleranceBlockRGB) && (int(blue) <= yellowTileRGB[2] + toleranceBlockRGB);

  // Purple Truth Range
  bool purpleR_Range = (int(red) >= purpleTileRGB[0] - toleranceBlockRGB) && (int(red) <= purpleTileRGB[0] + toleranceBlockRGB);
  bool purpleG_Range = (int(green) >= purpleTileRGB[1] - toleranceBlockRGB) && (int(green) <= purpleTileRGB[1] + toleranceBlockRGB);
  bool purpleB_Range = (int(blue) >= purpleTileRGB[2] - toleranceBlockRGB) && (int(blue) <= purpleTileRGB[2] + toleranceBlockRGB);

  // Red Truth Range
  bool redR_Range = (int(red) >= redTileRGB[0] - toleranceBlockRGB) && (int(red) <= redTileRGB[0] + toleranceBlockRGB);
  bool redG_Range = (int(green) >= redTileRGB[1] - toleranceBlockRGB) && (int(green) <= redTileRGB[1] + toleranceBlockRGB);
  bool redB_Range = (int(blue) >= redTileRGB[2] - toleranceBlockRGB) && (int(blue) <= redTileRGB[2] + toleranceBlockRGB);

  // Green Truth Range
  bool greenR_Range = (int(red) >= greenTileRGB[0] - toleranceBlockRGB) && (int(red) <= greenTileRGB[0] + toleranceBlockRGB);
  bool greenG_Range = (int(green) >= greenTileRGB[1] - toleranceBlockRGB) && (int(green) <= greenTileRGB[1] + toleranceBlockRGB);
  bool greenB_Range = (int(blue) >= greenTileRGB[2] - toleranceBlockRGB) && (int(blue) <= greenTileRGB[2] + toleranceBlockRGB);

  // Test for Red
  if ( redR_Range && redG_Range && redB_Range ){
    if (serialDebugger){
      Serial.println("Red Tile Detected");
      TCS_SerialOut();
    }
    colorCodeValue = 1;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  }

  // Test for Green
  if ( greenR_Range && greenG_Range && greenB_Range ){
    if (serialDebugger){
      Serial.println("Green Tile Detected");
      TCS_SerialOut();
    }
    colorCodeValue = 2;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  }

  // Test for Purple
  if ( purpleR_Range && purpleG_Range && purpleB_Range ){
    if (serialDebugger){
      Serial.println("Purple Tile Detected");
      TCS_SerialOut();
    }
    colorCodeValue = 3;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  }

  // Test for Yellow
  if ( yellowR_Range && yellowG_Range && yellowB_Range ){
    if (serialDebugger){
      Serial.println("Yellow Tile Detected");
      TCS_SerialOut();
    }
    colorCodeValue = 4;   // 0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow
  }

  // Output sensor vals to LED
  analogWrite(redpin, gammatable[(int)red]);
  analogWrite(greenpin, gammatable[(int)green]);
  analogWrite(bluepin, gammatable[(int)blue]);

  return colorCodeValue;

}

/*
 *  DISPLAY RGB VALUE 
 *  Prints rgb values to serial terminal
 *  Requires: TCStoRGB_Output() function
 *  Credit: TEAM 211
 */
void TCS_SerialOut(){

  Serial.print("R:\t"); Serial.print(int(red)); 
  Serial.print("\tG:\t"); Serial.print(int(green)); 
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print("\n");
}

/*
 * RECORD TILE COLOR POSITION 
 * Takes converted color sensor value (color code) and puts that value into a database array
 * Credit: TEAM 211
 */
void TileColorPosAcquisition(){
  //0 - none, 1 - red, 2 - green, 3 - purple, 4 - yellow

  // First Tile
  if( (Carriage_pos == tileData[0][0]) && (TileColorAcquisition() != 0)){
    tileData[0][1] = TileColorAcquisition();    // store colorcode value

    if (serialDebugger){
       Serial.println("First Tile: Color Recorded");
    }
  }

  // Second Tile
  if( (Carriage_pos == tileData[1][0]) && (TileColorAcquisition() != 0)){
    tileData[1][1] = TileColorAcquisition();    // store colorcode value

    if (serialDebugger){
       Serial.println("Second Tile: Color Recorded");
    }
  }

  // Third Tile
  if( (Carriage_pos == tileData[2][0]) && (TileColorAcquisition() != 0)){
    tileData[2][1] = TileColorAcquisition();    // store colorcode value

    if (serialDebugger){
       Serial.println("Third Tile: Color Recorded");
    }
  }

  // Forth Tile
  if( (Carriage_pos == tileData[3][0]) && (TileColorAcquisition() != 0)){
    tileData[3][1] = TileColorAcquisition();    // store colorcode value

    if (serialDebugger){
       Serial.println("Fourth Tile: Color Recorded");
    }
  }
}

/*
 *  X-AXIS: GO-TO (coord, speed)
 *  Moves the X-CARRIAGE using cartesian coords
 *  Credit: TEAM 211
 */
void GoTo(int new_pos, int speed){
  Stepper1->setSpeed(speed);
  int diff_pos = 0; // diffrence from old pos to new pos

  // new position is greater than current pos | 400 - 0 
  if(new_pos > Carriage_pos){
    diff_pos = new_pos - Carriage_pos;          // ie: 400 - 0 = 400

    if (serialDebugger){
      Serial.print("Current Pos: "); Serial.print(Carriage_pos); Serial.print(" | "); Serial.print("GoTo: "); Serial.print(new_pos); Serial.print(" | Diff: "); Serial.print(diff_pos);
      Serial.print("\n");
    }

    Stepper1->step(diff_pos * stepsPerMM, BACKWARD, DOUBLE);  // <- 400 
    Carriage_pos = new_pos;
  }

  // new position is lessthan than current pos | 300 - 400
  if(new_pos < Carriage_pos){
    diff_pos = Carriage_pos - new_pos; // ie: 400 - 300 = 100

    if (serialDebugger){
      Serial.print("Current Pos: "); Serial.print(Carriage_pos); Serial.print(" | "); Serial.print("GoTo: "); Serial.print(new_pos); Serial.print(" | Diff: -"); Serial.print(diff_pos);
      Serial.print("\n");
    }

    Stepper1->step(diff_pos * stepsPerMM, FORWARD, DOUBLE);  // 100 ->
    Carriage_pos = new_pos;
  }

}

/*  X-AXIS: MOVE COLORED BLOCK TO MATCHING TILE
 *  Moves x-carriage to 
 *  Function Assumes Block has been scanned and picked up already
 *  (func does not scan block, pick up block or drop block)
 *  Credit: TEAM 211
 */
void GoTo_BlockTileMatch(){
  for(int i = 0; i <= 4; i++){
    if(BlockColor_Scanned == tileData[i][1]){
      GoTo(tileData[i][0], 100);
    }
  }
}

/*
 * HOME BLOCK SUBROUTINE
 * Logic for moving a block to be picked up at a known pos
 * Dependency: Adafruit_MotorShield.h
 * Credit: TEAM 211
 */ 
void HomeCarriage(){
  do{
    carriage_homeState = digitalRead(returnpin);
    if(carriage_homeState == LOW){
      Carriage_pos = 0; // set pos marker to 0 - home
      Stepper1->release(); // release motor current
      
      if (serialDebugger){
        Serial.println("Carriage: Home Position");
      }
      break;
    }
    Stepper1->step(1 * stepsPerMM, FORWARD, MICROSTEP);
    if (serialDebugger){
      Serial.println("Carriage: Homing...");
    }
  }
  while(carriage_homeState == HIGH); 
}