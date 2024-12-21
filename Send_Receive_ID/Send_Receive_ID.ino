// send receive ID

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRutils.h>


// Battery check
#define PIN_ADC_BAT 35                // Pin connected to battery inside the T-Energy module
#define BlueLed 5                     // Inboard LED used as indicator of discharge
int in = 0;                           // Value read on battery pin 
int batteryLevelSamples = 5;          // Sampling level of battery reads
float v_bat = 0;                      // Converted battery voltage (V)
volatile unsigned long timerBat = 0;  // Time from last battery check

// Front LED pins
int RedLedPin = 2;
int GreenLedPin = 12;

// Setting the infrared ID

// Emission
#define IR_LED 18                     // GPIO pin used for coding IR diodes
IRsend irsend(IR_LED);                
#define DAC_CH1 25                    // GPIO pin used for powering the IR diodes
// Define the distance of emission by adjusting the LEDs voltage
// If voltage too high the robot ID will be reflected by walls and detected whatever the position
// If voltage too low the robot ID will be detected at a too short distance 
int voltIRLedSupply = 180;            // value of the DAC output 0~255
byte RobotID = 2;                     // ID given to the robot uploaded with this sketch - The other robot's ID must be different < 10
unsigned long timerPE = 0;            // Timer for emission
uint16_t periodIdSend = 100;          // Emission period at witch send the robot ID

// reception
const uint16_t RECV_PIN = 19;         // GPIO pin of the TSOP38kHz receiver
IRrecv irrecv(RECV_PIN);
decode_results results;
bool RobotDetect = LOW;               // Enable TTB detection on the right 
unsigned long timerPR = 0;            // Timer for reception 
uint16_t periodIdCheck = 100;         // Reception period at witch check the ID from the other robots

// ------------------------------------------

void setup() {

  Serial.begin(115200);  
 
  delay(2000);

  // Set battery pins and check
  pinMode(PIN_ADC_BAT, INPUT);
  pinMode(BlueLed,OUTPUT);
  read_adc_bat();

  // Set front LED pins
  pinMode(RedLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);
  digitalWrite(RedLedPin, LOW);
  digitalWrite(GreenLedPin, LOW);   
  
  // Configure infrared ID
  
  // Emission
  irsend.begin();                     // Start the emitter
  dacWrite(DAC_CH1, voltIRLedSupply); // Set the DAC output DC voltage that supply the IR diodes
  // Reception
  irrecv.enableIRIn();                // Start the receiver

  Serial.println("Send_Receive_ID.ino");
  Serial.println(F(__FILE__ " " __DATE__ " " __TIME__));
  Serial.println("robot is now emitting ID and waiting for IR ID");
}

// ---------------------------------------------

void loop() {

  ID_receive();
  ID_emission();
  read_adc_bat();
    
}

void ID_emission(){
  if (millis() - timerPE < periodIdSend) return;
  irsend.sendNEC(RobotID); 
  timerPE = millis();
  digitalWrite(RedLedPin,!digitalRead(RedLedPin));        // Blink when ID emission
}

void ID_receive(){
  if (millis() - timerPR < periodIdCheck) return;
  // Test ID reception and print
  digitalWrite(GreenLedPin,LOW);
  if (irrecv.decode(&results)) {
    if (results.decode_type == decode_type_t::NEC_LIKE) { 
      if (results.value <10 && results.value != RobotID){ // Filter ID self-reflected and ID values
        digitalWrite(GreenLedPin,HIGH);                   // Light when ID received
        Serial.println(results.value,DEC);                // Print the ID received
      }
    }
    irrecv.resume();
  }
  timerPR = millis();
  return;
}

void read_adc_bat() {
if (millis() - timerBat < 30000) return;
for (int i = 0; i < batteryLevelSamples; i++){
in += analogRead(PIN_ADC_BAT);
}
v_bat = (float)in /(4096*batteryLevelSamples) * 2 * 3.7; // resolution; sampling; internal divider; nominal voltage 18650
in = 0;
if( v_bat < 3 ){ 
  digitalWrite(BlueLed, HIGH);                 
  while (1==1);                       // Please switch off the robot and go to battery charge
}
else {
  digitalWrite(BlueLed, LOW);  
  }
timerBat = millis();
}
