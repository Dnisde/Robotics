// This example shows how to make a Balboa balance on its two
// wheels and drive around while balancing.
//
// To run this demo, you will need to install the LSM6 library:
//
// https://github.com/pololu/lsm6-arduino
//
// To use this demo, place the robot on the ground with the
// circuit board facing up, and then turn it on.  Be careful to
// not move the robot for a few seconds after powering it on,
// because that is when the gyro is calibrated.  During the gyro
// calibration, the red LED is lit.  After the red LED turns off,
// turn the robot so that it is standing up.  It will detect that
// you have turned it and start balancing.
//
// Alternatively, you can press the A button while the robot is
// lying down and it will try to use its motors to kick up into
// the balancing position.
//
// This demo is tuned for the 50:1 high-power gearmotor with
// carbon brushes, 45:21 plastic gears, and 80mm wheels; you will
// need to adjust the parameters in Balance.h for your robot.
//
// After you have gotten the robot balance well, you can
// uncomment some lines in loop() to make it drive around and
// play a song.

#include <Balboa32U4.h>
#include <QTRSensors.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"

LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;
Balboa32U4ButtonB buttonB;
Balboa32U4ButtonC buttonC;
Balboa32U4LineSensors lineSensors;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

bool useEmitters = true;

void setup()
{
  // Uncomment these lines if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  
  Serial.begin(57600);
  Serial.println("Done starting up.");

  // For the 5 Channel Reflectance sensor:
  // This program assumes your sensors are mounted in the edge-aligned
  // configuration (CTRL on pin 5). If you are using the sensors in the
  // center-aligned configuration (CTRL on pin 12), then comment out the call
  // to setEdgeAligned() and uncomment the call to setCenterAligned().
  // lineSensors.setEdgeAligned();
  lineSensors.setCenterAligned();
}

// Prints a line with all the sensor readings to the serial
// monitor.

void printReadingsToSerial()
{ 
  char buffer[160];
  static uint16_t lastSampleTime = 0;

  if ((uint16_t)(millis() - lastSampleTime) >= 100)
  {
    lastSampleTime = millis();

    // Read the line sensors.
    lineSensors.read(sensorValues);

    // Send the results to the LCD and to the serial monitor.
    Serial.print('C');
    Serial.print('D');
    sprintf(buffer, "%4d %4d %4d %4d %4d %4x %4x %4x %4x %4x \n",
      sensorValues[0]%255,
      sensorValues[1]%255,
      sensorValues[2]%255,
      sensorValues[3]%255,
      sensorValues[4]%255,
      sensorValues[0]%255,
      sensorValues[1]%255,
      sensorValues[2]%255,
      sensorValues[3]%255,
      sensorValues[4]%255
    );
    Serial.print(buffer);
  }

  // If button C is pressed, toggle the state of the emitters.
  if (buttonC.getSingleDebouncedPress())
  {
    useEmitters = !useEmitters;
  }
  
  
}


void SendSensorDataToSerial(uint8_t *checksum){
//  char buffer[80];
//  uint8_t checksum = checksum
  sendSignedInt32MSBAndUpdateChecksum((sensorValues[0]%255), checksum); // The upper cases of ASCII MAP is 255, so make the value not go further than 255
  sendSignedInt32MSBAndUpdateChecksum((sensorValues[1]%255), checksum);
  sendSignedInt32MSBAndUpdateChecksum((sensorValues[2]%255), checksum);
  sendSignedInt32MSBAndUpdateChecksum((sensorValues[3]%255), checksum);
  sendSignedInt32MSBAndUpdateChecksum((sensorValues[4]%255), checksum);
//  sprintf(buffer, "%c\n", useEmitters ? 'E' : 'e');
//  Serial.print(buffer);
//  Serial.write(checksum);
}

void getValueFromSensor(uint8_t *checksum)
{
  static uint16_t lastSampleTime = 0;

  if ((uint16_t)(millis() - lastSampleTime) >= 100)
  {
    lastSampleTime = millis();

    // Read the line sensors.
    lineSensors.read(sensorValues);

    // Send the results to the LCD and to the serial monitor.
    SendSensorDataToSerial(checksum);
  }

  // If button C is pressed, toggle the state of the emitters.
  if (buttonC.getSingleDebouncedPress())
  {
    useEmitters = !useEmitters;
  }
}


const char song[] PROGMEM =
//  "!O6 T240"
//  "l32ab-b>cl8r br b-bb-a a-r gr g-4 g4"
//  "a-r gr g-gg-f er e-r d4 e-4"
//  "gr msd8d8ml d-4d4"
//  "l32efg-gl8r msd8d8ml d-4d4"
//  "<bcd-d e-efg- ga-ab- a4 gr";
  "V@$C718(f@@33 ASFF35785733";

void playSong()
{
  if (!buzzer.isPlaying())
  {
    buzzer.playFromProgramSpace(song);
  }
}

void driveAround()
{
  uint16_t time = millis() % 8192;
  uint16_t leftSpeed, rightSpeed;
  if (time < 1900)
  {
    leftSpeed = 20;
    rightSpeed = 20;
  }
  else if (time < 4096)
  {
    leftSpeed = 25;
    rightSpeed = 15;
  }
  else if (time < 4096 + 1900)
  {
    leftSpeed = 20;
    rightSpeed = 20;
  }
  else
  {
    leftSpeed = 15;
    rightSpeed = 25;
  }

  balanceDrive(leftSpeed, rightSpeed);
}

void standUp()
{
  motors.setSpeeds(0, 0);
  buzzer.play("!>grms>g16>g16>g2");
  ledGreen(1);
  ledRed(1);
  ledYellow(1);
  while (buzzer.isPlaying());
  motors.setSpeeds(-MOTOR_SPEED_LIMIT, -MOTOR_SPEED_LIMIT);
  delay(400);
  motors.setSpeeds(150, 150);
  for (uint8_t i = 0; i < 20; i++)
  {
    delay(UPDATE_TIME_MS);
    balanceUpdateSensors();
    if(angle < 60000)
    {
      break;
    }
  }
  motorSpeed = 150;
  balanceResetEncoders();
}


/**
 * Send debug data over the serial port in human readable format
 **/
void sendDebugData(){
 uint32_t currentMS = millis();
 static uint32_t lastMS = 0;
 //Only run this every once in a while.
 if((currentMS - lastMS) < 2000) return;

 Serial.println("\r\n----");
 Serial.print("Millis: "); Serial.println(millis());
 Serial.print("Batt: "); Serial.print(readBatteryMillivolts()); Serial.println("mV");
 Serial.print("Gyro Y: "); Serial.print(angle/1000); Serial.println("deg");
 Serial.print("Gyro X: "); Serial.print(angleX/1000); Serial.println("deg");
 Serial.print("Gyro Z: "); Serial.print(angleZ/1000); Serial.println("deg");
 Serial.print("Cmd Speed l: "); Serial.print(driveLeft);
 Serial.print(" r: "); Serial.println(driveRight);
 Serial.print("Speed l: "); Serial.print(speedLeft);
 Serial.print(" r: "); Serial.println(speedRight);
 Serial.print("Dist l: "); Serial.print(encoders.getCountsLeft());
 Serial.print(" r: "); Serial.println(encoders.getCountsRight());

 lastMS = currentMS;
}

/**
 * Sends a 32 bit value over the serial port with the most significant byte first.
 * Also updates the checksum.
 **/
void sendSignedInt32MSBAndUpdateChecksum(int32_t val, uint8_t *checksum){
  uint8_t v = (val >> 24) & 0xFF;
  *checksum += v;
  Serial.write(v);
  v = (val >> 16) & 0xFF;
  *checksum += v;
  Serial.write(v);
  v = (val >> 8) & 0xFF;
  *checksum += v;
  Serial.write(v);
  v = (val >> 0) & 0xFF;
  *checksum += v;
  Serial.write(v);   
}

/**
 * Send data packets to ROS
 **/
void sendDataToROS(){
 uint32_t currentMS = millis();
 static uint32_t lastMS = 0;
 //Only run this every once in a while.
 //N.B. there will be rollovers every ~65sec, but shouldn't be an issue
 if((currentMS - lastMS) < 100) return;

  //Start out with the checksum non-zero
  uint8_t checksum = 0xCD;
  //Start bytes
  Serial.write('C');
  Serial.write('D');
  sendSignedInt32MSBAndUpdateChecksum(currentMS,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(readBatteryMillivolts(),&checksum);
  sendSignedInt32MSBAndUpdateChecksum(angle,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(angleX,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(angleZ,&checksum);

  sendSignedInt32MSBAndUpdateChecksum(driveLeft,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(driveRight,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(speedLeft,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(speedRight,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(encoders.getCountsLeft(),&checksum);
  sendSignedInt32MSBAndUpdateChecksum(encoders.getCountsRight(),&checksum);
  getValueFromSensor(&checksum);
//  Serial.write('\n'); // To make output from Serial more clear
//  Serial.println(angleZ);
  /* //Some unit tests
  sendSignedInt32MSBAndUpdateChecksum(0,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(-1,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(1,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(0x7FFFFFFF,&checksum);
  sendSignedInt32MSBAndUpdateChecksum(0x80000000,&checksum);
  */
  
  Serial.write(checksum);

 lastMS = currentMS;
}

/**
 * Process commands from ROS
 **/
void processDataFromROS(){
  #define STATE_START 0
  #define STATE_LEFT 1
  #define STATE_RIGHT 2
  #define STATE_CHECKSUM 3

  //Variables that need to stick around as we iterate through our state machine.
  static int8_t checksum;
  static uint8_t state = STATE_START;
  static int8_t left;
  static int8_t right;

  int8_t c = 0;
  //While we have more bytes from the serial port
  while(Serial.available() > 0){  
    Serial.readBytes((char *)&c,1);
    //Go through our state machine
    switch(state){
      case STATE_START:
        //Check to see if it is our start byte, if so transition
        if(c == (int8_t)0xCD){
          state = STATE_LEFT;
          checksum = 0xCD;
        }
      break;
      case STATE_LEFT:
        checksum += c;
        left = c;
        state = STATE_RIGHT;
      break;
      case STATE_RIGHT:
        checksum += c;
        right = c;
        state = STATE_CHECKSUM;
      break;
      case STATE_CHECKSUM:
        if(c != checksum){
          //Serial.print("command "); Serial.print(left); Serial.print(", "); Serial.println(right);
          Serial.print("chksum failed "); Serial.print(c); Serial.print(" != "); Serial.println(checksum);
        }else{
          //Serial.print("command "); Serial.print(left); Serial.print(", "); Serial.println(right);
          balanceDrive(left,right);
        }
        state = STATE_START;
        break;
      default:
        state = STATE_START;
        break;
    }
  }
  
}

void loop()
{
  static bool enableSong = false;
  static bool enableDrive = false;
  static uint16_t lastSampleTime = 0;

  balanceUpdate();

  //Send the debug data (human readable) over the serial port. Probably better
  //to not do this while sending ROS packet, but it won't break anything
  //sendDebugData();

  sendDataToROS();
  processDataFromROS();
//  printReadingsToSerial();
  
/*
  if (isBalancing())
  {
    if (enableSong)   { playSong(); }
    if (enableDrive)  { driveAround(); }
  }
  else
  {
    buzzer.stopPlaying();
    balanceDrive(0, 0); // reset driving speeds

    if (buttonA.getSingleDebouncedPress())
    {
      enableSong = false;
      enableDrive = false;
      standUp();
    }
    else if (buttonB.getSingleDebouncedPress())
    {
      enableSong = false;
      enableDrive = true;
      standUp();
    }
    else if (buttonC.getSingleDebouncedPress())
    {
      enableSong = true;
      enableDrive = true;
      standUp();
    }
  }
  */

  // Illuminate the red LED if the last full update was too slow.
  ledRed(balanceUpdateDelayed());

  // Display feedback on the yellow and green LEDs depending on
  // the variable fallingAngleOffset.  This variable is similar
  // to the risingAngleOffset used in Balance.cpp.
  //
  // When the robot is rising toward vertical (not falling),
  // angleRate and angle have opposite signs, so this variable
  // will just be positive or negative depending on which side of
  // vertical it is on.
  //
  // When the robot is falling, the variable measures how far off
  // it is from a trajectory starting it almost perfectly
  // balanced then falling to one side or the other with the
  // motors off.
  //
  // Since this depends on ANGLE_RATE_RATIO, it is useful for
  // calibration.  If you have changed the wheels or added weight
  // to your robot, you can try checking these items, with the
  // motor power OFF (powered by USB):
  //
  // 1. Try letting the robot fall with the Balboa 32U4 PCB up.
  //    The green LED should remain lit the entire time.  If it
  //    sometimes shows yellow instead of green, reduce
  //    ANGLE_RATE_RATIO.
  //
  // 2. If it is tilted beyond vertical and given a push back to
  //    the PCB-up side again, the yellow LED should remain lit
  //    until it hits the ground.  If you see green, increase
  //    ANGLE_RATE_RATIO.
  //
  // In practice, it is hard to achieve both 1 and 2 perfectly,
  // but if you can get close, your constant will probably be
  // good enough for balancing.
  int32_t fallingAngleOffset = angleRate * ANGLE_RATE_RATIO - angle;
  if (fallingAngleOffset > 0)
  {
    ledYellow(1);
    ledGreen(0);
  }
  else
  {
    ledYellow(0);
    ledGreen(1);
  }
  
  
}
