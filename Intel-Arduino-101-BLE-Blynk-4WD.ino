#include <Adafruit_MotorShield.h>

/**************************************************************
 * This sketch is for an Arduino 101 BLE rover using Blynk and Adafruit Motor Shield V2
 * Code was copied from both Adafruit and Blynk librarie examples
 * Full documentation on building this rover yourself on Hackster.IO and electronhacks.com
 * Sketch is released under MIT license
 **************************************************************/


#define BLYNK_PRINT Serial
#include <BlynkSimpleCurieBLE.h>
#include <CurieBLE.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "dbe76033368e406c8b573bdf81df9ad7";

BLEPeripheral  blePeripheral;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *motorLF = AFMS.getMotor(2);
Adafruit_DCMotor *motorLR = AFMS.getMotor(3);
Adafruit_DCMotor *motorRF = AFMS.getMotor(1);
Adafruit_DCMotor *motorRR = AFMS.getMotor(4);


class Navigation {

  public:
    Navigation() {
    
      // start the IMU and filter
      CurieIMU.begin();
      CurieIMU.setGyroRate(25);
      CurieIMU.setAccelerometerRate(25);
      filter.begin(25);
    
      // Set the accelerometer range to 2G
      CurieIMU.setAccelerometerRange(2);
      // Set the gyroscope range to 250 degrees/second
      CurieIMU.setGyroRange(250);
    
      // initialize variables to pace updates to correct rate
      microsPerReading = 1000000 / 25;
      microsPrevious = micros();
    }
    
    void loop() {
      int aix, aiy, aiz;
      int gix, giy, giz;
      float ax, ay, az;
      float gx, gy, gz;
      float roll, pitch, heading;
      unsigned long microsNow;
    
      // check if it's time to read data and update the filter
      microsNow = micros();
      if (microsNow - microsPrevious >= microsPerReading) {
    
        // read raw data from CurieIMU
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
    
        // convert from raw data to gravity and degrees/second units
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);
    
        // update the filter, which computes orientation
        filter.updateIMU(gx, gy, gz, ax, ay, az);
    
        // print the heading, pitch and roll
        roll = filter.getRoll();
        pitch = filter.getPitch();
        heading = filter.getYaw();
        Serial.print("Orientation: ");
        Serial.print(heading);
        Serial.print(" ");
        Serial.print(pitch);
        Serial.print(" ");
        Serial.println(roll);
    
        // increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
      }
    }

    float getHeading() {
      return heading;
    }
  private:
    Madgwick filter;
    unsigned long microsPerReading, microsPrevious;
    float accelScale, gyroScale;
    
    float convertRawAcceleration(int aRaw) {
      // since we are using 2G range
      // -2g maps to a raw value of -32768
      // +2g maps to a raw value of 32767
      
      float a = (aRaw * 2.0) / 32768.0;
      return a;
    }
    
    float convertRawGyro(int gRaw) {
      // since we are using 250 degrees/seconds range
      // -250 maps to a raw value of -32768
      // +250 maps to a raw value of 32767
      
      float g = (gRaw * 250.0) / 32768.0;
      return g;
    }
};
Navigation* nav;

//######### SETUP ######################################
void setup() {
  Serial.begin(9600);
  delay(1000);

  // The name your bluetooth service will show up as, customize this if you have multiple devices
  blePeripheral.setLocalName("Arduino101DH");
  blePeripheral.setDeviceName("Arduino101DH");
  blePeripheral.setAppearance(384);

  Blynk.begin(blePeripheral, auth);

  blePeripheral.begin();
  Serial.println("Waiting for connections...");
    
  Serial.println("Adafruit Motorshield v2 - DC Motor");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorLF->setSpeed(0);
  motorLR->setSpeed(0);
  motorRF->setSpeed(0);
  motorRR->setSpeed(0);  

  nav = new Navigation();

  // Accelerometer setup
  CurieIMU.begin();
  CurieIMU.attachInterrupt(CrashEventCallback);   // Enable Shock Detection 
 /* 
  * The Threshold value should be adjusted through some experimentation 
  * so that crashes are detected at reasonable shock levels
  * this is to ensure the safety of the operator and to minimize stress on the motors
  * typical values for the function are from 1500 - 2000
  * there were updates to the Arduino 101 firmware and CurieIMU library in July 2016
  * please ensure you have the latest version using the board manager of the Arduino IDE
  * 
  */
  CurieIMU.setDetectionThreshold(CURIE_IMU_SHOCK, 1500);   // 2.0g = 2000mg; 1.5g = 1500mg, etc.
  CurieIMU.setDetectionDuration(CURIE_IMU_SHOCK, 50);   // 50ms
  CurieIMU.interrupts(CURIE_IMU_SHOCK);
}

// Callback function from the CurieIMU ShockDetect Sample
static void CrashEventCallback(void) {
  Serial.println("CrashEventCallback");
//  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
//    if (CurieIMU.shockDetected(X_AXIS, POSITIVE))
//      Serial.println("Negative shock detected on X-axis");
//    if (CurieIMU.shockDetected(X_AXIS, NEGATIVE))
//      Serial.println("Positive shock detected on X-axis");
//    if (CurieIMU.shockDetected(Y_AXIS, POSITIVE))
//      Serial.println("Negative shock detected on Y-axis");
//    if (CurieIMU.shockDetected(Y_AXIS, NEGATIVE))
//      Serial.println("Positive shock detected on Y-axis");
//    if (CurieIMU.shockDetected(Z_AXIS, POSITIVE))
//      Serial.println("Negative shock detected on Z-axis");
//    if (CurieIMU.shockDetected(Z_AXIS, NEGATIVE))
//      Serial.println("Positive shock detected on Z-axis");
//  }
//  if (CurieIMU.getInterruptStatus(CURIE_IMU_SHOCK)) {
  Serial.println("STOP!!!");
    motorLF->run(RELEASE);
    motorLR->run(RELEASE);
    motorRF->run(RELEASE);
    motorRR->run(RELEASE);
//  }
}
//########## LOOP ######################################
void loop() {
  Blynk.run();
  blePeripheral.poll();
  nav->loop();
}


//######### Subrutines ################################

// This function will set the speed
BLYNK_WRITE(V0)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  // You can also use:
  // String i = param.asStr();
  // double d = param.asDouble();
  Serial.print("V0 Slider value is: ");
  Serial.println(pinValue);
  motorLF->setSpeed(pinValue);
  motorLR->setSpeed(pinValue); 
  motorRF->setSpeed(pinValue);
  motorRR->setSpeed(pinValue);   
}

// Motor 1 Forward
BLYNK_WRITE(V1)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.print("Motor 1 Forward: ");
  Serial.println(pinValue);
  if(pinValue == 1) {
    motorLF->run(FORWARD);
    motorLR->run(FORWARD);
    motorRF->run(FORWARD);
    motorRR->run(FORWARD);
  }
  if(pinValue == 0) {
    motorLF->run(RELEASE);
    motorLR->run(RELEASE);    
    motorRF->run(RELEASE);    
    motorRR->run(RELEASE);    
  }
}


// Motor 2 Left
BLYNK_WRITE(V2)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.print("Motor 2 Forward: ");
  Serial.println(pinValue);
  if(pinValue == 1) {
    motorLF->run(BACKWARD);
    motorLR->run(BACKWARD);
    motorRF->run(FORWARD);
    motorRR->run(FORWARD);   
  }
  if(pinValue == 0) {
    motorLF->run(RELEASE);
    motorLR->run(RELEASE);
    motorRF->run(RELEASE);
    motorRR->run(RELEASE);
  }
}

// Motor 1 Right
BLYNK_WRITE(V3)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.print("Motor 1 Backward: ");
  Serial.println(pinValue);
  if(pinValue == 1) {
    motorLF->run(FORWARD);
    motorLR->run(FORWARD);    
    motorRF->run(BACKWARD);    
    motorRR->run(BACKWARD);    
  }
  if(pinValue == 0) {
    motorLF->run(RELEASE);
    motorLR->run(RELEASE);    
    motorRF->run(RELEASE);    
    motorRR->run(RELEASE);    
  }
}

// Motor 2 Backward
BLYNK_WRITE(V4)
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  Serial.print("Motor 2 Backward: ");
  Serial.println(pinValue);
  if(pinValue == 1) {
    motorLF->run(BACKWARD);
    motorLR->run(BACKWARD);
    motorRF->run(BACKWARD);
    motorRR->run(BACKWARD);
  }
  if(pinValue == 0) {
    motorLF->run(RELEASE);
    motorLR->run(RELEASE);
    motorRF->run(RELEASE);
    motorRR->run(RELEASE);    
  }
}
