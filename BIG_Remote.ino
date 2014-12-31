// Bluetooth-Integrated Gyroscopic Remote (BIG Remote)
// Interfaced for CHARLOTTE (the Hexapod)
// Written by Samuel Hinshelwood JR. 
// Started: Sat Aug 23, 2013

/* Sourcecode for infrared remote embedded with ATMega328 Pro Mini microcontroller,
 *  GY-521 Gyroscope/Accelerometer and HC-05 (Master) Bluetooth transceiver.
 *  
 *  Each button on the IR remote is tethered to a specific command:
 *  
 *  Three Modes of Operation: MotionControl-[1], AutoRun-[2], and ManualRun-[3]
 *  Programmable Tricks: [4][5][6]
 *                       [7][8][9]
 *                       [*][0][#]
 * Directional buttons [←][→] correspond to left and right turning, while [↑][↓] correspond to forward and backward.
 
 *  MotionControl- User controls Hexapod like a Wii Remote; i.e. 3-axis gyroscopic feedback Control
 *    "OK" button initiates motion-mirroring, and movement of the remote is transmitted via bluetooth to 
 *    CHARLOTTE the Hexapod. All buttons enabled when not motion-mirroring.  
 
 *  AutoRun- Hexapod operates autonomously; crawls around, avoids objects, detects/responds to 
 *    motion, and performs tricks based on its environment. Operational-mode buttons enabled only. 
 
 *  ManualRun- Hexapod operates on manual commands only. All buttons enabled. 
 */ 

//TODO: 
 // Write motionReponse()
 // Write transmit()
 // Write manualRun()
 // Write autoRun() 
 // Add formal function comments
 // Replace SendSelection()'s switch table with cascading ifs and button enumerations
 // Remove excess code paths
 // Calibrate delays for maximum performance

#include "IRremote.h"
#include "I2Cdev.h"
#include "mpu6050.h"
#include <SoftwareSerial.h>
#include <Time.h>

// Button Address Enumerations
int ONE = 0xFF6897;
int TWO = 0xFF9867;
int THREE = 0xFFB04F;
int OK = 0xFF02FD;
int _REPEAT = 0xFFFFFFFF;

/* Unused formally
  LEFT = 0xFF22DD;
  RIGHT = 0xFFC23D;
  FORWARD = 0xFF629D;
  REVERSE = 0xFFA857;
  FOUR = 0xFF30CF;
  FIVE = 0xFF18E7;
  SIX = 0xFF7A85;
  SEVEN = 0xFF10EF;
  EIGHT = 0xFF38C7;
  NINE = 0xFF5AA5;
  ASTERISK = 0xFF42BD;
  ZERO = 0xFF4AB5;
  HASHMARK = 0xFF52AD;
*/

//Gyro/Acc Preparation
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Pins & Constants
const int IRPin = 2; // IR Receiver pin
int gRxPin = 3; // Bluetooth Rx Pin
int gTxPin = 4; // Bluetooth Tx Pin
const int transBasePin = 11; // transistor pin
const int AVG_OFFSET = 175; // in deg/sec
int leftLED = 6;
int rightLED = 7;

//Objects
IRrecv irrecv(IRPin); // IR Receiver Object
decode_results results; // IR Receiver Output Object
MPU6050 accelgyro; // Accelerometer/Gyroscope Module Object
SoftwareSerial BTSerial(gRxPin, gTxPin); // Bluetooth Serial Object

// Default Operational Mode
int mode = 1; // ManualRun-[3]
              // AutoRun-[2]
              // MotionControl-[1]
              
// Bluetooth Module Mode
int gIsMaster = 1; // 1 for master, 0 for slave

void setup(){
  // For I2C Bus preparation
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(9600); delay(5);
  Serial.println("Initializing...");
  pinMode(IRPin, INPUT); // IR Initialization
  pinMode(transBasePin, OUTPUT);
  pinMode(leftLED, OUTPUT); // LED Test Pad Initialization
  pinMode(rightLED, OUTPUT);
  irrecv.enableIRIn(); // enable IR to receive input
  Serial.println("bt2bt HC module - version a"); // Bluetooth Module Initialization
  Serial.println("Ready!");
  transmit(mode); // Operational Mode Initialization
}

void loop(){
  if(irrecv.decode(&results)){
    setMode();
    irrecv.resume();
    delay(500); // important to not receive multiple reads
  }
  //These three mode functions loop within themselves until mode change
  switch(mode){
    case 1:
      motionControl(); // Run in MotionControl mode
      break;
    case 2:
      autoRun(); // Run in AutoRun mode
      break;
    default:
      manualRun(); // Run in ManualRun mode
      break;
  }
}

// Helper Functions:
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Gyroscope/Accelerometer Helpers
// Gets/Sets 6-axes values of the gyroscope/accelerometer
void setGyro(int& gx, int& gy, int& gz, int& ax, int& ay, int& az){
  gx = (int) (accelgyro.GetGyroReading(0) * -1);
  gy = (int) (accelgyro.GetGyroReading(1));
  gz = (int) accelgyro.GetGyroReading(2);
  ax = (int) (accelgyro.GetAccelerometerReading(0) * -1);
  ay = (int) (accelgyro.GetAccelerometerReading(1) * -1);
  az = (int) accelgyro.GetAccelerometerReading(2);
}

// Prints Gyroscope/Accelerometer Readings
void printGyro(int gx, int gy, int gz, int ax, int ay, int az){
  Serial.print("Gryo X: "); 
    Serial.print(gx); Serial.print("\t");
    Serial.print("Y: "); 
    Serial.print(gy); Serial.print("\t");
    Serial.print("Z: "); 
    Serial.print(gz); Serial.print("\t");
    Serial.print("Accel X: "); 
    Serial.print(ax); Serial.print("\t");
    Serial.print("Y: "); 
    Serial.print(ay); Serial.print("\t");
    Serial.print("Z: "); 
    Serial.print(az); Serial.println("\t");
}

// Takes a sample of motion data and calculates average thresholds for each axis
void calibrateMirror(int& avgGX, int& avgGY, int& avgGZ){
  for(int i = 0; i < 100; i++){ //takes .5 sec
    accelgyro.Update();
    avgGX += (int) (accelgyro.GetGyroReading(0) * -1);
    avgGY += (int) (accelgyro.GetGyroReading(1) * -1);
    avgGZ += (int) (accelgyro.GetGyroReading(2) * -1);
    delay(5);
  }
  avgGX /= 100;
  avgGY /= 100;
  avgGZ /= 100;
  // Blink when done
  digitalWrite(leftLED, HIGH);
  digitalWrite(rightLED, HIGH);
  delay(300);
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
  delay(300);
  digitalWrite(leftLED, HIGH);
  digitalWrite(rightLED, HIGH);
  delay(300);
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
  delay(300);
}

// Monitors motion values and sends action commdands accordingly to values "out-of-bounds" over each axis
void mirror(int gx, int gy, int gz, int avgGX, int avgGY, int avgGZ){ //needs large delays for LEFT, RIGHT, FORWARD,
                                                                      //REVERSE, LEANLEFT ([*]), and LEANRIGHT ([#])
  // Find axis with max abs. speed, and perform that axis's funct.
  // check X 
  if((abs(gx) > abs(gy)) && (abs(gx) > abs(gz))){
    if(gx > (avgGX + 80 /*AVG_OFFSET*/)){ //positive x-axis
      //sendSelection(LEFT); // send LEFT
      Serial.println("X-axis pos");
      digitalWrite(leftLED, HIGH);
      //wait for reverse motion //(while gx > 0)
      while(gx >= -(AVG_OFFSET / 2)){ //going pos or still
        accelgyro.Update();
        gx = (int) (accelgyro.GetGyroReading(0) * -1);
        delay(5);
      }
      digitalWrite(leftLED, LOW);
      delay(150);
    }else if(gx < (avgGX - AVG_OFFSET)){ //negative x-axis
      //sendSelection(RIGHT); // send RIGHT
      Serial.println("X-axis, neg");
      // wait for reverse motion
      digitalWrite(rightLED, HIGH);
      // wait for reverse motion // (while gx > 0)
      while(gx <= (AVG_OFFSET / 2)){ //going pos or still
        accelgyro.Update();
        gx = (int) (accelgyro.GetGyroReading(0) * -1);
        delay(5);
      }
      digitalWrite(rightLED, LOW);
      delay(150);
    }
  } else if((abs(gz) > abs(gx)) && (abs(gz) > abs(gy))){
    
    // check Z
    if(gz > (avgGZ + AVG_OFFSET)){ //positive Z-axis
      //sendSelection(REVERSE); // send LEFT
      Serial.println("Z-axis pos");
      digitalWrite(leftLED, HIGH);
      //wait for reverse motion //(while gx > 0)
      while(gz >= -(AVG_OFFSET / 2)){ //going pos or still
        accelgyro.Update();
        gz = (int) (accelgyro.GetGyroReading(2) * -1);
        delay(5);
      }
      digitalWrite(leftLED, LOW);
      delay(150);
    } else if(gz < (avgGZ - AVG_OFFSET)){ // negative Z-axis
      //sendSelection(FORWARD); // send RIGHT
      Serial.println("Z-axis neg");
      // wait for reverse motion
      digitalWrite(rightLED, HIGH);
      // wait for reverse motion // (while gx > 0)
      while(gz <= (AVG_OFFSET / 2)){ // going pos or still
        accelgyro.Update();
        gz = (int) (accelgyro.GetGyroReading(2) * -1);
        delay(5);
      }
      digitalWrite(rightLED, LOW);
      delay(150);
    }
  } else if((abs(gy) > abs(gx)) && (abs(gy) > abs(gz))){
    
    // check Y
    if(gy > (avgGY + AVG_OFFSET)){ // positive Y-axis
      //sendSelection(ASTERISK); // send LEFT
      Serial.println("Y-axis pos");
      digitalWrite(leftLED, HIGH);
      // wait for reverse motion // (while gx > 0)
      while(gy >= -(AVG_OFFSET / 2)){ // going pos or still
        accelgyro.Update();
        gy = (int) (accelgyro.GetGyroReading(1) * -1);
        delay(5);
      }
      digitalWrite(leftLED, LOW);
      delay(150);
    } else if(gy < (avgGY - AVG_OFFSET)){ // negative Y-axis
      //sendSelection(HASHMARK) // send RIGHT
      Serial.println("Y-axis neg");
      // wait for reverse motion
      digitalWrite(rightLED, HIGH);
      // wait for reverse motion // (while gx > 0)
      while(gy <= (AVG_OFFSET / 2)){ // going pos or still
        accelgyro.Update();
        gy = (int) (accelgyro.GetGyroReading(1) * -1);
        delay(5);
      }
      digitalWrite(rightLED, LOW);
      delay(150);
    }
  }
} 

/* Function: motionControl
 * ----------------
 * When in MotionControl mode, BIG Remote repeatedly checks for button presses.
 * If "OK" is pressed, the remote starts recording its orientation and sends respective motion commands
 * to the Hexapod via the bluetooth master module. In "OK" mode, only "OK" can be pressed to 
 * exit mirroring mode. 
 *
 * When "OK" is not pressed, the remote is responsive to other button-presses, like mode-changes
 * (i.e. [2] and [3]), the directional buttons, and the trick buttons ([4] thru [9]).  
 *
 * Updates/transmits the mode upon exiting.  
 */
void motionControl(){
  // In-mode inits
  digitalWrite(transBasePin, HIGH);
  delay(50);
  accelgyro.Initialise();
  int avgGX, avgGY, avgGZ;
  calibrateMirror(avgGX, avgGY, avgGZ);
  
  while(true){ //Mode-loop
    // Selection scheme
    if(irrecv.decode(&results)){
        int selection = results.value;
        irrecv.resume();
        delay(50); // for mult sig
        
        // Press 'OK' once to engage mirroring, press again to disengage
        if((selection == OK) || (selection == _REPEAT)){ // Enter mirroring loop
          digitalWrite(leftLED, HIGH);
          digitalWrite(rightLED, HIGH);
          delay(300);
          digitalWrite(leftLED, LOW);
          digitalWrite(rightLED, LOW);
          Serial.println("Mirroring");
          
          while(true){
            if(irrecv.decode(&results)){ // exit mirroring-loop
              selection = results.value;
              irrecv.resume();
              if((selection == OK) || (selection == _REPEAT)){ 
                Serial.println("UnMirroring");
                delay(50); // multiple-feedback filter
                break; // break from mirroring
              }
            }
            
            // Records and prints movement of the remote
            accelgyro.Update();
            int gx, gy, gz, ax, ay, az;
            setGyro(gx, gy, gz, ax, ay, az);
            printGyro(gx, gy, gz, ax, ay, az);
            //mirror(gx, gy, gz, avgGX, avgGY, avgGZ);
            delay(5);         
          }
          
        // Button-press checks
        } else if((selection == TWO) || (selection == THREE)){ // if another mode was chosen
            digitalWrite(transBasePin, LOW);
            setMode();
            break;
        } else{ // any other button was pressed
            sendSelection(selection);
        }
        irrecv.resume();
        delay(1000); // important to not receive multiple reads
    }
  }
} 

/* Function: autoRun
 * ----------------
 * When in AutoRun mode, BIG Remote repeatedly checks only for operational-mode button presses.
 * Otherwise, CHARLOTTE will operate on autonomously. 
 * Updates/transmits the mode upon exiting.  
 */
void autoRun(){
  delay(500);
} 

/* Function: manualRun
 * ----------------
 * When in manualRun mode, BIG Remote repeatedly for all button presses, and sends commands to CHARLOTTE accordiingly.
 * Updates/transmits the mode upon exiting.  
 */
void manualRun(){
  // transmit()
  while(true){
    if(irrecv.decode(&results)){
      int selection = results.value;
      if((selection == ONE) || (selection == THREE)){
        setMode();
        break;
      } else{ //all other buttons
        sendSelection(selection);
      }
      irrecv.resume();
      delay(1000); //important to not receive multiple reads
    }
    delay(500);
  } 
}

// Gets and returns the mode
int getMode(){
  switch(results.value){
    case 0xFF6897: return 1;
    case 0xFF9867: return 2;
    case 0xFFB04F: return 3;
    default: return 3;
  }
}

// Sets the mode
void setMode(){
  mode = getMode();
  transmit((char) mode);
  Serial.println(results.value, HEX);  // See raw values
  Serial.print("Changing mode to: ");
  Serial.println(mode);
}

// Sends integer instruction via bluetooth
// TODO: Copy Slave Mode Instructions over to CHARLOTTE sourcecode
void transmit(char action){
  if(1 == gIsMaster){
    BTSerial.print(action);
    delay(1000);
  }else{ //Slave Mode Instructions
    /*
      if (BTSerial.available()){
        char tmpChar = BTSerial.read();
        Serial.println(tmpChar);
        if(tmpChar == '1'){
          digitalWrite(gLedPin, HIGH);
        } else if(tmpChar == '0'){
            digitalWrite(gLedPin, LOW);
        }
      }
    */
  }
}

//Transmits via bluetooth commands from BIG HC-05 (Master) to Hexapod HC-05 (Slave)
void sendSelection(int button){ //every button except [OK] and [1][2][3]
  switch(button){
    case 0xFF22DD: 
      transmit('l'); //left button
      Serial.println("Pressed left");
      break;
    case 0xFFC23D:
      transmit('r'); //right button
      Serial.println("Pressed right");
      break;
    case 0xFF629D:
      transmit('f'); //forward button
      Serial.println("Pressed forward");
      break;
    case 0xFFA857:
      transmit('r'); //reverse button
      Serial.println("Pressed reverse");
      break;
    case 0xFF30CF:
      transmit('4'); //button [4]
      Serial.println("Pressed 4");
      break;
    case 0xFF18E7:
      transmit('5'); //[5]
      Serial.println("Pressed 5");
      delay(1000);
      break;
    case 0xFF7A85: 
      transmit('6'); //[6]
      Serial.println("Pressed 6");
      delay(1000);
      break;
    case 0xFF10EF:
      transmit('7'); //[7]
      Serial.println("Pressed 7");
      delay(1000);
      break;
    case 0xFF38C7:
      transmit('8'); //[8]
      Serial.println("Pressed 8");
      delay(1000);
      break;
    case 0xFF5AA5:
      transmit('9'); //[9]
      Serial.println("Pressed 9");
      delay(1000);
      break;
    case 0xFF42BD:
      transmit('14'); //[*]
      Serial.println("Pressed *");
      delay(1000);
      break;
    case 0xFF52AD:
      transmit('15'); //[#]
      Serial.println("Pressed #");
      delay(1000);
      break;
    case 0xFF4AB5: 
      transmit('0'); //[0]
      Serial.println("Pressed 0");
      delay(1000);
      break;
    case 0xFFFFFFFF: //repeat
      Serial.println("Repeat");
      //delay(1000);
      break;
    default:
      Serial.println(results.value);
      Serial.println("Couldn't interpret command. No action taken"); 
      //delay(1000);
      break;
  }
}
