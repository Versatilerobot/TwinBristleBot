/* Sketch TBB#3
   VersatileRobot Dec.7th, 2024

   MIT License
   Copyright (c) 2024 Versatilerobot
  
   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:
  
   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.
  
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.   
*/

#include <Wire.h>
#include <VL53L0X.h>
#include "src/QTRSensorsESP32.h"      // Pololu libray that i modified to be compatible with 12 bits ESP32 (4096 resolution)

// Battery check
#define PIN_ADC_BAT 35                // Pin connected to battery inside the T-Energy module
#define BlueLed 5                     // Inboard LED used as indicator of discharge
int in = 0;                           // Value read on battery pin 
int batteryLevelSamples = 5;          // Sampling level of battery reads
float v_bat = 0;                      // Converted battery voltage (V)
bool warnBat = LOW;                   // Battery discharged warning
volatile unsigned long timerBat = 0;  // Time from last battery check

// Analog QTR line sensors instance
QTRSensors qtr;
const uint8_t SensorCount = 4;
uint16_t sensorValues[SensorCount];
uint16_t KP = 0.2;                    // Proportional - start by 0.1
uint16_t KD = 3;                      // Derivate - start by 3
int16_t lastError = 0;                // Previous error level of position on the line


// Front LED pins
int RedLedPin = 2;
int GreenLedPin = 12;
volatile unsigned long timerBlinkSpeed = 0;

// Capacitive button sensors
int startButton = 14;                 // Inverted for avoid conflict when switching
int modeButton = 13;                  // Used for set speed
int threshold = 45;                   // Level of touch detection
bool startdetected = LOW;
bool previoustart = LOW;
bool speedSelect = LOW;
volatile unsigned long sinceLastTouchStart = 0;
volatile unsigned long sinceLastTouchSelector = 0;

// Motor pins
int motorPinLeft = 27;
int motorPinRight = 26; 

// Motor PWM properties for ESP32
const int freq = 20000;
const int MotorL = 0;                 // channel 0
const int MotorR = 1;                 // channel 1
const int resolution = 8;
int PWM []= {95, 110, 120};           // Speed values - Low, Medium, High
int dutyCycle = PWM [1];              // Default Medium
int pointer = 1;

// Motor speed
int16_t leftMotorSpeed = 0;
int16_t rightMotorSpeed = 0;

// Tof VL53L0X instance with XSHUT standby
VL53L0X sensor;
#define XSHUT 23
bool XSHUT_STATE = HIGH;
uint16_t Sensor_Offset = 42;          // Distance offset (mm)
uint16_t measureDistance = 0;         // Value to store the measured distance
uint16_t obstacle = 110;              // Warning distance treshold (mm)
bool warnObstacle = LOW;              // Obstacle warning
bool object = LOW;                    // Obstacle detection
unsigned long timerTof = 0;           // Time from last distance check
unsigned long periodObjCheck = 50;    // Period at witch check the object detection

// ------------------------------------------

void setup() {

  Serial.begin(115200);  
  Wire.begin();
  
  // Set VL53L0X 
  pinMode(XSHUT, OUTPUT);             // Standby pin 
  digitalWrite(XSHUT, XSHUT_STATE);
  delay(50);
  sensor.init();
  // Set timeout  
  if (!sensor.init())
  {
    Serial.println(F("Failed to detect and initialize sensor!"));
    while (1) {}
  }
  sensor.setTimeout(100);
  // Set Tof higher speed at the cost of lower accuracy
  // reduce timing budget to 10 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(10000);
  // Set the minimum amplitude of the signal reflected from the target and received by the sensor necessary for it to report a valid reading 
  // Units of MCPS (mega counts per second). Useful for reduce measurement error (default is 0.25 MCPS)
  sensor.setSignalRateLimit(2.5);  
  
  // Set battery pins and check
  pinMode(PIN_ADC_BAT, INPUT);
  pinMode(BlueLed,OUTPUT);
  read_adc_bat();
  if (warnBat == HIGH){
    while (1==1);
  }
  
  // Set capacitive touch interrupt for start bouton
  touchAttachInterrupt(startButton, gotTouch, threshold);

  // Configure the line sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){36, 39, 34, 32}, SensorCount);
  qtr.setEmitterPin(4);

  delay(500);
  
  // Set the motor pin as output
  pinMode(motorPinLeft, OUTPUT);
  pinMode(motorPinRight, OUTPUT);  
  
  // Configure motor PWM functionalities - specific ESP32
  ledcSetup(MotorL, freq, resolution);
  ledcSetup(MotorR, freq, resolution);  
  
  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(motorPinLeft, MotorL);
  ledcAttachPin(motorPinRight, MotorR);

  // Set front LED pins
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);

  // Calibrate the QTR line sensors
  digitalWrite(RedLedPin, HIGH);       // Turn on Front LED to indicate we are in calibration mode
  digitalWrite(GreenLedPin, HIGH);     // Turn on Front LED to indicate we are in calibration mode
  for (uint16_t i = 0; i < 200; i++)   // Calibration loop
  {
    qtr.calibrate();
  }
  digitalWrite(RedLedPin, LOW);        // Turn off Front LED to indicate we are through with calibration
  digitalWrite(GreenLedPin, LOW);      // Turn off front LED to indicate we are through with calibration

  // Indicate sketch uploaded
  Serial.println("TBB3_Circuit_V1.ino");
  Serial.println(F(__FILE__ " " __DATE__ " " __TIME__));
    
  // Indicate min and max values per sensor - can be skip
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]); // maximum reflectance (i.e. whiter)
    Serial.print(' ');
  }
  Serial.println();
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]); // minimum reflectance (i.e. blacker)
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
timerBat = millis();                  // init timer
  timerTof = timerBat;                // init timer
}

// ---------------------------------------------

void loop() {

  qtr.emittersOff(); // Turn Off line sensor for reduce the current load
  frontLed();
  read_adc_bat();
  read_obstacle();
   
  if (startdetected && !warnBat && !object){  // ON      
    if (!previoustart) previoustart = HIGH;   
    qtr.emittersOn();                         // Turn On line sensor
    // Get calibrated sensor values returned in the sensors array, along with the line position
    // Position will range from 0 to 3000, with 1500 corresponding to the line over the two middle sensors of module
    uint16_t position = qtr.readLineBlack(sensorValues); 
    // Compute our "error" from the line position. We will make it so that the error is zero when
    // the two middle sensors are over the line, because this is our goal.  Error will range from -1500 to +1500.
    int16_t error = position - 1500;          // error level of position on the line 

    // avoid drift
    if (error >= 1450 || error <=-1450) {
      dutyCycle = PWM [0];                    // Set low speed        
      }
      else {
        dutyCycle = PWM [pointer];            // Set the base motor speed
        }
     
     // Slow down if suspition of obstacle
     if (warnObstacle == HIGH){
      dutyCycle = PWM [0];                    // Set low speed
     }
        
    // Normalized error    
    error = error * dutyCycle / 1500;

    // Set the motor speed based on proportional and derivative PID terms:
    // KP is the floating-point proportional constant
    // KD is the floating-point derivative constant
    // KP + KD = 1
    // Note that when doing PID, it is very important you get your signs right, or
    // else the control loop will be unstable.
    int16_t motorSpeed = 0.6 * error + 0.4 * (error - lastError);
    lastError = error;

    // DutyCycle is base motor speed (the speed the motors should run if you
    // are perfectly on the line with no error).
    // When you start testing your PID loop, it might help to start with small
    // values for dutyCycle. You can then increase the speed as you fine-tune 
    // your PID constants KP and KD.
    leftMotorSpeed = dutyCycle - motorSpeed;
    rightMotorSpeed = dutyCycle + motorSpeed;
    
    // Keep the speed positive and add a similar check to keep the speeds 
    // from exceeding a maximum limit.
    if (leftMotorSpeed < 0) { leftMotorSpeed = 0; }
    if (rightMotorSpeed < 0) { rightMotorSpeed = 0; }
    if (leftMotorSpeed > 255) { leftMotorSpeed = 255; }
    if (rightMotorSpeed > 255) { rightMotorSpeed = 255; }    
           
    // Set motor speeds using the two motor speed variables above      
    ledcWrite(MotorL, leftMotorSpeed); 
    ledcWrite(MotorR, rightMotorSpeed); 
  }
  else { 
    if (previoustart){
      ledcWrite(MotorL, 0);                   // stop
      ledcWrite(MotorR, 0);                   // stop
      previoustart = LOW;
    } 
    speedSelector();                          // Speed selection when stopped
  }
}

//------------------------------------------------------------------

void gotTouch() {
  if (millis() - sinceLastTouchStart < 500) return; // simple debounce
  startdetected = !startdetected;
  sinceLastTouchStart = millis();  
}

void speedSelector() {
  if (millis() - sinceLastTouchSelector < 500) return; // simple debounce
  if(touchRead(modeButton) < threshold && speedSelect == LOW){
//      Serial.println("touch"); 
      speedSelect = HIGH;
      pointer++;
      if (pointer>2) pointer=0;
      dutyCycle = PWM [pointer];
      }
      else if (speedSelect == HIGH){
      speedSelect = LOW;
      }
//  Serial.println(PWM [pointer]);    
  sinceLastTouchSelector = millis();  
}

void frontLed() {
  if (previoustart == HIGH){      
    if (abs(leftMotorSpeed-rightMotorSpeed) < 50){
      // go straight
      digitalWrite(RedLedPin, HIGH);
      digitalWrite(GreenLedPin, HIGH);
    }
    else if (leftMotorSpeed > rightMotorSpeed){
      // turn right
      digitalWrite(RedLedPin, LOW);
      digitalWrite(GreenLedPin, HIGH);
    }
    else if (leftMotorSpeed < rightMotorSpeed){
      // turn left
      digitalWrite(RedLedPin, HIGH);
      digitalWrite(GreenLedPin, LOW);    
    }
  }
  else{  
      // Blinking speed selected
    digitalWrite(RedLedPin, LOW);
    digitalWrite(GreenLedPin, LOW); 
    int LED = RedLedPin;
    for (int loop=0; loop<2; loop++){
      for (int i=0; i<pointer+1; i++){
      timerBlinkSpeed = millis();
      while(millis() - timerBlinkSpeed < 400){
      if (startdetected == HIGH)return;
      }
      digitalWrite(LED,!digitalRead(LED));
      timerBlinkSpeed = millis();
      while(millis() - timerBlinkSpeed < 400){
      if (startdetected == HIGH)return;
      }
      digitalWrite(LED,!digitalRead(LED));
      }
      LED = GreenLedPin;
    }
  }
}

void read_obstacle(){
  if (millis() - timerTof < periodObjCheck) return;    
  if (periodObjCheck == 2000) periodObjCheck = 50; 
  // One shoot measurement for save time 
  measureDistance = sensor.readRangeSingleMillimeters()- Sensor_Offset;   
// Serial.println(measureDistance);
    if (measureDistance < obstacle && !sensor.timeoutOccurred()) { 
      if (warnObstacle == HIGH){
      periodObjCheck = 2000;                  // waiting time for restart
      object = HIGH;                          // obstacle detected
      }
    warnObstacle = HIGH;                      // suspition of obstacle
    }
    else {
      warnObstacle = LOW;
      object = LOW;
      }  
  timerTof = millis();
}

void read_adc_bat() {
if (millis() - timerBat < 30000) return;
for (int i = 0; i < batteryLevelSamples; i++){
in += analogRead(PIN_ADC_BAT);
}
v_bat = (float)in /(4096*batteryLevelSamples) * 2 * 3.7; // resolution; sampling; internal divider; nominal voltage 18650
in = 0;
if (v_bat < 3 && warnBat == LOW ){
  warnBat = HIGH;  
  digitalWrite(BlueLed, HIGH);               // Warn battery discharged
}
else if(v_bat < 3 && warnBat == HIGH ){ 
  digitalWrite(BlueLed, LOW);                
  digitalWrite(RedLedPin, LOW);
  digitalWrite(GreenLedPin, LOW);   
  while (1==1);                              // Please switch off the robot and go to battery charge
}
else {
  digitalWrite(BlueLed, LOW);  
  }
timerBat = millis();
}
