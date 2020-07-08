/*
A high speed balancing robot, running on an ESP32.

Wouter Klop
wouter@elexperiment.nl
For updates, see elexperiment.nl

Use at your own risk. This code is far from stable.

This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/
This basically means: if you use my code, acknowledge it.
Also, you have to publish all modifications.

*/

#include <Arduino.h>
#include <FlySkyIBus.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Streaming.h>
#include <MPU6050.h>
#include <PID.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <fastStepper.h>
#include <par.h>
#include <Preferences.h>  // for storing settings
#include <Adafruit_INA219.h>

// ----- Type definitions
typedef union {
  struct {
    float val; // Float (4 bytes) comes first, as otherwise padding will be applied
    uint8_t cmd;
    uint8_t checksum;
  };
  uint8_t array[6];
} command;

// Plot settings
struct {
  boolean enable = 0; // Enable sending data
  uint8_t prescaler = 4;
} plot;

#define FORMAT_SPIFFS_IF_FAILED true

// ----- Function prototypes
void sendWifiList(void);
void parseSerial();
void parseCommand(char* data, uint8_t length);
void calculateGyroOffset(uint8_t nSample);
void readSensor();
void initSensor(uint8_t n);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendConfigurationData(uint8_t num);
bool InitPIDs();

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();

// ----- Definitions and variables
// -- Web server
const char* http_username = "admin";
const char* http_password = "admin";
AsyncWebServer httpServer(80);
WebSocketsServer wsServer = WebSocketsServer(81);

// -- EEPROM
Preferences preferences;

// -- Stepper motors
#define motEnablePin 27

#define reverseLeftMotor true
#define reverseRightMotor false

fastStepper motLeft(5, 4, 0, reverseLeftMotor, motLeftTimerFunction);
fastStepper motRight(18, 15, 1, reverseRightMotor, motRightTimerFunction);

uint8_t microStep = 32;
uint8_t motorCurrent = 35;
//float maxStepSpeed = 410; //small wheeels
float maxStepSpeed = 360; //big wheeels 360

float absSpeedChange = 0;

// -- PID control
#define dT_MICROSECONDS 5000
#define dT dT_MICROSECONDS/1000000.0

#define PID_ANGLE_MAX 15
#define PID_POS_MAX 32
#define PID_SPEED_MAX 20
// default (go)
PID pidAngle(cPD, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
PID pidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);
PID pidSpeed(cPID, dT, PID_SPEED_MAX, -PID_SPEED_MAX);

// stay
PID pidAngleStay(cPD, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
PID pidPosStay(cPD, dT, PID_POS_MAX, -PID_POS_MAX);

// to save intial values
PID IntialPidAngle(cPD, dT, PID_ANGLE_MAX, -PID_ANGLE_MAX);
PID IntialPidPos(cPD, dT, PID_POS_MAX, -PID_POS_MAX);

uint8_t controlMode = 1; // 0 = only angle, 1 = angle+position, 2 = angle+speed

// -- IMU
MPU6050 imu;

// current and voltage 
Adafruit_INA219 ina219(0x40);

#define GYRO_SENSITIVITY 65.5

int16_t gyroOffset[3];
float accAngle = 0;
float filterAngle = 0;
float accAngle_X = 0;
float filterAngle_X = 0;
float angleOffset = 0.0;
float gyroFilterConstant = 0.998;
float gyroGain = 1.42;

// -- Others
#define ledPin 2
#define motorCurrentPin 25

float steerFilterConstant = 0.9;
float speedFilterConstant = 0.9;

// -- WiFi
const char host[] = "BalancingRobot";

// Noise source (for system identification)
boolean noiseSourceEnable = 0;
float noiseSourceAmplitude = 1;

float steerInput, speedInput;

uint32_t lastCntInput = 0;

boolean enableControl = 0;

boolean inOTA = false;

uint8_t pidTypeApply = 0; // PID to use 0=auto, 1= go PIDs, 2= stay PIDs 

float batVoltageArray[16];
float batVoltage = 0;
float batVoltageLowest = 26;
float DeadBatVoltage = 9.8;
float current_mA = 0;
float power_mW = 0;
float mA_h_count =0;

uint32_t energySave_ready =0;

boolean calibrationAutoDone = false;


// ----- Parameter definitions -----
// void updatePIDParameters() {
//   pidAngle.updateParameters();
//   pidSpeed.updateParameters();
//   pidPos.updateParameters();
// }
// par pidPar[] = {&pidAngle.K, &pidAngle.Ti, &pidAngle.Td, &pidAngle.N, &pidAngle.R, &pidAngle.minOutput, &pidAngle.maxOutput, &pidAngle.controllerType,
//   &pidPos.K, &pidPos.Ti, &pidPos.Td, &pidPos.N, &pidPos.R, &pidPos.minOutput, &pidPos.maxOutput, &pidPos.controllerType,
//   &pidSpeed.K, &pidSpeed.Ti, &pidSpeed.Td, &pidSpeed.N, &pidSpeed.R, &pidSpeed.minOutput, &pidSpeed.maxOutput, &pidSpeed.controllerType, &updatePIDParameters};
//
// parList pidParList(pidPar);

// par motorPar[] = {&motorCurrent, &maxStepSpeed};
// par wifiPar[] = {&wifiMode, &wifiSSID, &wifiKey};
// par sensorPar[] = {&gyroOffset, &gyroGain, &angleOffset, &updateGyroOffset, &updateAngleOffset};
// par controlPar[] = {&remoteType, &controlMode};

// struct {
//   struct {
//     uint8_t mode;
//     char ssid[30];
//     char key[30];
//   } wifi;
// } settings;

// ----- Interrupt functions -----
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR motLeftTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void sendData(uint8_t *b, uint8_t l) {
  wsServer.sendBIN(0,b,l);
}

void wirelessTask(void * parameters) {
  while (1) {
    IBus.loop();
    wsServer.loop();
    ArduinoOTA.handle();
    delay(2);
  }
}

// compare PIDs
bool InitPIDs()
{
  if(pidAngle.K==IntialPidAngle.K && pidAngle.Ti==IntialPidAngle.Ti && pidAngle.Td==IntialPidAngle.Td && pidAngle.N==IntialPidAngle.N)
  {
    if(pidPos.K==IntialPidPos.K && pidPos.Ti==IntialPidPos.Ti && pidPos.Td==IntialPidPos.Td && pidPos.N==IntialPidPos.N)
    return true;
  }
 return false;
}

// ----- Main code
void setup() {

  Serial.begin(115200);
  IBus.begin(Serial2);
  preferences.begin("settings", false);  // false = RW-mode
 // preferences.clear();  // Remove all preferences under the opened namespace

  pinMode(motEnablePin, OUTPUT);
  digitalWrite(motEnablePin, 1); // Disable steppers during startup


  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, 0);

  motLeft.init();
  motRight.init();
  motLeft.microStep = microStep;
  motRight.microStep = microStep;

  // SPIFFS setup
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
    Serial.println("SPIFFS mount failed");
    return;
  } else {
    Serial.println("SPIFFS mount success");
  }

  // Gyro setup
  delay(200);
  Wire.begin(21,22,400000);

  // ============== find all i2c adresses
  Serial.println("===============================");
  Serial.println(F("Finding all i2c adresses !"));
        
  byte error, address;
  int nDevices;
  nDevices = 0;
   for (address = 1; address < 127; address++ )  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0){
      Serial.println("-");
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.println("-");
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  } //for loop
  if (nDevices == 0){
    Serial.println("No I2C devices found");

  }
   Serial.println("===============================");

 // ============== i2c test end.

 
  imu.initialize();
  imu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  // Calculate and store gyro offsets
  delay(50);

  // sensor start
  ina219.begin();

  // Init EEPROM, if not done before
  #define PREF_VERSION 1  // if setting structure has been changed, count this number up to delete all settings
  if (preferences.getUInt("pref_version", 0) != PREF_VERSION) {
    preferences.clear();  // Remove all preferences under the opened namespace
    preferences.putUInt("pref_version", PREF_VERSION);
    Serial << "EEPROM init complete, all preferences deleted, new pref_version: " << PREF_VERSION << "\n";
  }

  // Read gyro offsets
  Serial << "Gyro calibration values: ";
  for (uint8_t i=0; i<3; i++) {
    char buf[16];
    sprintf(buf, "gyro_offset_%u", i);
    gyroOffset[i] = preferences.getShort(buf, 0);
    Serial << gyroOffset[i] << "\t";
  }
  Serial << endl;

  // Read angle offset
  angleOffset = preferences.getFloat("angle_offset", 0.0);

  // Perform initial gyro measurements
  initSensor(50);

  // Connect to Wifi and setup OTA if known Wifi network cannot be found
  boolean wifiConnected = 0;
  if (preferences.getUInt("wifi_mode", 0)==1) {
    char ssid[63];
    char key[63];
    preferences.getBytes("wifi_ssid", ssid, 63);
    preferences.getBytes("wifi_key", key, 63);
    Serial << "Connecting to '" << ssid << "'" << endl;
    // Serial << "Connecting to '" << ssid << "', '" << key << "'" << endl;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, key);
    if (!(WiFi.waitForConnectResult() != WL_CONNECTED)) {
      Serial.print("Connected to WiFi with IP address: ");
      Serial.println(WiFi.localIP());
      wifiConnected = 1;
    } else {
      Serial.println("Could not connect to known WiFi network");
    }
  }
  if (!wifiConnected) {
    Serial.println("Starting AP...");
    WiFi.mode(WIFI_AP_STA);
    // WiFi.softAPConfig(apIP, apIP, IPAddress(192,168,178,24));
    WiFi.softAP("balancingRobot", "turboturbo");
    Serial.print("AP started with IP address: ");
    Serial.println(WiFi.softAPIP());
  }

  ArduinoOTA.setHostname(host);
  ArduinoOTA
  .onStart([]() {
    // disable robot
    enableControl = 0;
    inOTA = true;
    motLeft.speed = 0;
    motRight.speed = 0;
    digitalWrite(motEnablePin, 1);
    //
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
    ESP.restart();
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  // Start DNS server
  if (MDNS.begin(host)) {
    Serial.print("MDNS responder started, name: ");
    Serial.println(host);
  } else {
    Serial.println("Could not start MDNS responder");
  }

  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("Loading index.htm");
    request->send(SPIFFS, "/index.htm");
  });

  httpServer.serveStatic("/", SPIFFS, "/");
  httpServer.onNotFound([](AsyncWebServerRequest *request){
      request->send(404, "text/plain", "FileNotFound");
  });

  httpServer.addHandler(new SPIFFSEditor(SPIFFS,http_username,http_password));
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(webSocketEvent);

  MDNS.addService("http", "tcp", 80);
  MDNS.addService("ws", "tcp", 81);
   

// PID (go)
  pidAngle.setParameters(1.2,0.57,0.19,5.8);
  pidPos.setParameters(1.1,1.85,0.72,20);
  pidSpeed.setParameters(1.8,3.6,0.65,0.6);

// PID (stay)  
  pidAngleStay.setParameters(1.9,0.57,0.64,8.2); // no swing pids, good for staying, bad for going
  pidPosStay.setParameters(1.0,1.85,0.30,10.0); // no swing pids, good for staying, bad for going


  IntialPidAngle = pidAngle; // copy intial PIDs
  IntialPidPos = pidPos;

  // pidParList.read();

  // Run wireless related tasks on core 0
  // xTaskCreatePinnedToCore(
  //                   wirelessTask,   /* Function to implement the task */
  //                   "wirelessTask", /* Name of the task */
  //                   10000,      /* Stack size in words */
  //                   NULL,       /* Task input parameter */
  //                   0,          /* Priority of the task */
  //                   NULL,       /* Task handle. */
  //                   0);  /* Core where the task should run */


 // ready beep
 dacWrite(motorCurrentPin, 7);
 digitalWrite(motEnablePin, 0); // enable
 motLeft.speed = -1000;
 motRight.speed = -1000;
 motLeft.update();
 motRight.update();
 delay(360);
 motLeft.speed = 650;
 motRight.speed = 650;
 motLeft.update();
 motRight.update();
 delay(170);
 motLeft.speed = 0.01; // slow led blinking
 motRight.speed = 0.01;
 motLeft.update();
 motRight.update();
 digitalWrite(motEnablePin, 1); // disable
 dacWrite(motorCurrentPin, motorCurrent);
 Serial.println("End of setup!");
}


void loop() {
  static unsigned long tLast = 0;
  float pidAngleOutput = 0;
  float avgMotSpeed;
  float steer = 0;
  static float avgSteer;
  static float avgSpeed;
  static float avgMotSpeedSum = 0;
  int32_t avgMotStep;
  float pidPosOutput = 0, pidSpeedOutput = 0;
  static uint8_t k = 0;
  static uint32_t lastInputTime = 0;
  uint32_t tNowMs;
  float noiseValue = 0;

  static uint32_t sendLaslWs =0;

  static uint32_t maxCurHold =0;

  static uint32_t measurment_last =0;
  static boolean measurment_switch = true; // just to minimize i2c use time

  static uint32_t maxSpeedCount =0;

  static float absSpeed = 0;

  unsigned long tNow = micros();
  tNowMs = millis();

  //check voltage every xx ms.
  if(millis()-measurment_last>100){ 
    measurment_last = millis();
    if(measurment_switch){ // true every 200ms
      for (int i = 0; i != 15; i++) { // move
      batVoltageArray[i] = batVoltageArray[i + 1];
      }
      batVoltageArray[15]= ina219.getBusVoltage_V()+(ina219.getShuntVoltage_mV()/1000) + 0.1; // write to last
      float batVoltageTemp = 0;
      for (int i = 0; i != 15; i++) {// add all
        batVoltageTemp+=batVoltageArray[i];
      }
      if(batVoltageArray[0]!=0){// if array full, calculate avg
            batVoltage=batVoltageTemp/15; // avg   
                if (batVoltage<batVoltageLowest)// updt lowest voltage
                {
                  batVoltageLowest = batVoltage;
                }
      }
    }

    if(!measurment_switch){
      current_mA = (current_mA + ina219.getCurrent_mA())/2;
      power_mW = (power_mW + ina219.getPower_mW())/2;
      if(current_mA<0)
      {
      mA_h_count += -((current_mA/3600)/5);
      }
    }

    measurment_switch=!measurment_switch; // invert
  }
          

  // blink led depending on battery level
  if(batVoltageLowest<DeadBatVoltage-0.2){ // dead battery
    digitalWrite(ledPin, millis()>>6 &1);
    enableControl = 0;
    motLeft.speed = 0.003; // very slow led blinking, dead battery
    motRight.speed = 0.003;
    digitalWrite(motEnablePin, 1); // disable
    }  
  else if(batVoltageLowest<DeadBatVoltage){digitalWrite(ledPin, millis()>>8 &1);}  
  else if(batVoltageLowest<DeadBatVoltage+0.5){digitalWrite(ledPin, millis()>>10 &1);} // 1 Hz blinking
  else{digitalWrite(ledPin, 0);}
      

  if (tNow-tLast > dT_MICROSECONDS && !inOTA) {
    readSensor();
    
    if (enableControl) {
      
//      // Read receiver inputs, comment out if not used
//      if (IBus.isActive()) { // Check if receiver is active
//        speedInput = ((float) IBus.readChannel(1)-1500)/5.0; // Normalise between -100 and 100
//        steerInput = ((float) IBus.readChannel(0)-1500)/5.0;
//
//        // update
//        lastCntInput = millis();
//      }

        //to 0, if not updated x ms, in case ws control suddently goes offline with not 0 as the last value
        if (lastCntInput<millis()-1300)
        {
          speedInput=0;
          steerInput=0;
        }

        avgSpeed = speedFilterConstant*avgSpeed + (1-speedFilterConstant)*speedInput/12;
        avgSteer = steerFilterConstant*avgSteer + (1-steerFilterConstant)*steerInput/2.0;

        // turn off on dead battery
        if(batVoltageLowest<DeadBatVoltage){
          avgSpeed=0;
          avgSteer=0;
         }

         


//        // step manipulation control (expirement)
//        motRight.setStep(motRight.getStep()+speedInput/4);
//        motLeft.setStep(motLeft.getStep()+speedInput/4);
//        if(abs(speedInput)>=0.2){
//        energySave_ready=0;
//        }
//        avgSpeed=0.0;
        

        if (abs(avgSpeed)<0.2) {
          // speedInput = 0;
        } else {
          lastInputTime = tNowMs;
          if (controlMode==1) {
            controlMode = 2;
            motLeft.setStep(0);
            motRight.setStep(0);
            pidSpeed.reset();
          }
        }

        steer = avgSteer;
        // if (abs(avgSteer)>1) {
        //   steer = avgSteer * (1 - abs(avgSpeed)/150.0);
        // } else {
        //   steer = 0;
        // }


      if (tNowMs-lastInputTime>300 && controlMode == 2) {
        controlMode = 1;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidPos.reset();
      }



      if (controlMode == 0) { // angle
        pidAngle.setpoint = avgSpeed*2;
        
      } else if (controlMode == 1) { // angle+position
        avgMotStep = (motLeft.getStep() + motRight.getStep())/2;
        pidPos.setpoint = avgSpeed;
        pidPos.input = -((float) avgMotStep) / 1000.0;
        pidPosOutput = pidPos.calculate();
        pidAngle.setpoint = pidPosOutput;
        
      } else if (controlMode == 2) { // angle+speed
        pidSpeed.setpoint = avgSpeed; // pid set target
        pidSpeed.input = -avgMotSpeedSum/35.0; // pid speed now (input)
        pidSpeedOutput = pidSpeed.calculate(); // pid out
        pidAngle.setpoint = pidSpeedOutput; // apply to angle
      }


      // Optionally, add some noise to angle for system identification purposes
      // if (noiseSourceEnable) {
      //   pidAngle.input = filterAngle + noiseSourceAmplitude*((random(1000)/1000.0)-0.5);
      // } else {
        pidAngle.input = filterAngle;
      // }

      pidAngleOutput = pidAngle.calculate();

      if (noiseSourceEnable) {
        noiseValue = noiseSourceAmplitude*((random(1000)/1000.0)-0.5);
        pidAngleOutput += noiseValue;
      }

      avgMotSpeedSum += pidAngleOutput/2;
      if (avgMotSpeedSum>maxStepSpeed) {
        avgMotSpeedSum  = maxStepSpeed;
      } else if (avgMotSpeedSum<-maxStepSpeed) {
        avgMotSpeedSum  = -maxStepSpeed;
      }
      avgMotSpeed = avgMotSpeedSum;

      float absSpeed_Before = absSpeed;
      absSpeed = abs(avgMotSpeed);

      absSpeedChange = abs(absSpeed_Before - absSpeed);


     // default control
     motLeft.speed = avgMotSpeed + steer;
     motRight.speed = avgMotSpeed - steer;


//     // control like in a car (expirement)
//     motLeft.speed = avgMotSpeed - ((steer/100.0)*(avgMotSpeed));
//     motRight.speed = avgMotSpeed + ((steer/100.0)*(avgMotSpeed));




      
//======
// count how many cycles robot is stable & not moving.
if (absSpeed<20 && abs(filterAngle)<5 && controlMode == 1 && abs(avgSteer)<0.01) // true if robot stable
{ 
  energySave_ready++;
}
else
{
  energySave_ready=0;
}
//=============================================



// count how many cycles motors at max speed without input.
if (maxStepSpeed==absSpeed && abs(avgSpeed)<0.2)
{ 
  maxSpeedCount++;
}
else
{
  maxSpeedCount=0;
}
//=============================================


//==================current control
int setCur = 0;
if (absSpeed<15){
  setCur = map(absSpeed, 0, 15, 42, 70);
  setCur = constrain(setCur, 42, 70); 
}

// lower current when robot stays for a long period of time
if(energySave_ready>=2500 && absSpeed<5 && abs(filterAngle)<4){
setCur = map(absSpeed, 0, 3, 28, 42);
}

if(absSpeed>=15)
{
  setCur = map(absSpeed, 15, 120, 70,  110); 
  setCur = constrain(setCur, 70,  110); 
}

// add current on angle
if(setCur<100){ 
int addCur = map(abs(filterAngle), 0, 8, 0, 25);
addCur = constrain(addCur, 0, 25);
setCur = setCur + addCur;
}

// on fast change max current for atleast XX ms
if(absSpeedChange>2 || millis()-maxCurHold < 250) 
{
  setCur = 180;
    if(absSpeedChange>2){
      maxCurHold=millis();
    }
}
motorCurrent = setCur;
dacWrite(motorCurrentPin, setCur);
//====



// apply spetial stay PIDs if robot stable
if((energySave_ready==400 && abs(filterAngle)<1.3) && InitPIDs() && pidTypeApply ==0){
  pidAngle = pidAngleStay; // apply stay (no swing pids), good for staying, bad for going
  pidPos = pidPosStay; // apply stay (no swing pids), good for staying, bad for going
  //Serial.println("Stay PIDs applied automatically");
}
if((abs(filterAngle)>=2.2 || energySave_ready<400) && !InitPIDs() && pidTypeApply ==0) // robot going, so revert PIDs
{
  pidAngle=IntialPidAngle; // revert to initial pids
  pidPos=IntialPidPos;
  //Serial.println("Go PIDs applied automatically");
}

if(pidTypeApply ==2) // only stay PIDs
{
  pidTypeApply=20; // it means applied
  pidAngle = pidAngleStay; // apply stay (no swing pids), good for staying, bad for going
  pidPos = pidPosStay; // apply stay (no swing pids), good for staying, bad for going
  //Serial.println("Stay PIDs applied manually");
}

if(pidTypeApply ==1) // only go PIDs
{
  pidTypeApply=10; // applied
  pidAngle=IntialPidAngle; // revert to initial pids
  pidPos=IntialPidPos;
  //Serial.println("Go PIDs applied manually");
}
//======================




//            //send data via websocket, for bebug purposes
//  if(millis()>sendLaslWs+400)
//  {
//    sendLaslWs = millis();
//    String sendTxt ="";  
//    sendTxt +="minusIN: ";
//    sendTxt += String(minusIN);
//    sendTxt +=" | maxStepSpeed-absSpeed: ";
//    sendTxt += String(maxStepSpeed-absSpeed);
////    sendTxt +=" | filterAngle: ";
////    sendTxt += String(filterAngle);
//    sendTxt +=" | speedInput: ";
//    sendTxt += String(speedInput);
//    wsServer.sendTXT(0, sendTxt);
//  }





// auto micro calibratation (once per boot when robot stable)
if(abs(filterAngle)>=1.8 && energySave_ready==4000 && !calibrationAutoDone){ // auto calibration
  angleOffset = angleOffset + (filterAngle*0.5);
  preferences.putFloat("angle_offset", angleOffset);
  calibrationAutoDone = true;
}

//======

      // Disable control if robot is almost horizontal. Re-enable if upright.
      if (abs(filterAngle)>50 || maxSpeedCount > 300) {  // (if motors at max spped for a long time, something happened, disable robot)
        enableControl = 0;
        motLeft.speed = 0.01; // slow led blinking, ready state
        motRight.speed = 0.01;
        digitalWrite(motEnablePin, 1); // Inverted action on enable pin
      }
    } else {
      if (abs(filterAngle)<0.5 && abs(filterAngle_X) < 13 && batVoltageLowest>DeadBatVoltage) { // (re-)enable and reset stuff
        enableControl = 1;
        controlMode = 1;
        avgMotSpeedSum = 0;
        motLeft.setStep(0);
        motRight.setStep(0);
        pidAngle.reset();
        pidPos.reset();
        pidSpeed.reset();
        digitalWrite(motEnablePin, 0); // Inverted action on enable pin
        // delay(1);
      }
    }

    motLeft.update();
    motRight.update();
    // updateStepper(&motLeft);
    // updateStepper(&motRight);

    if (k==plot.prescaler) {
      k = 0;

      if (wsServer.connectedClients(0)>0 && plot.enable) {
        union {
          struct {
            uint8_t cmd = 255;
            uint8_t fill1;
            uint8_t fill2;
            uint8_t fill3;
            float f[16]; //(+1) //13
          };
          uint8_t b[68]; //(+4) //56
        } plotData;

        plotData.f[0] = micros()/1000000.0;
        plotData.f[1] = accAngle;
        plotData.f[2] = filterAngle;
        plotData.f[3] = pidAngle.setpoint;
        plotData.f[4] = pidAngle.input;
        plotData.f[5] = pidAngleOutput;
        plotData.f[6] = pidPos.setpoint;
        plotData.f[7] = pidPos.input;
        plotData.f[8] = pidPosOutput;
        plotData.f[9] = pidSpeed.setpoint;
        plotData.f[10] = pidSpeed.input;
        plotData.f[11] = pidSpeedOutput;
        plotData.f[12] = noiseValue;
        plotData.f[13] = motorCurrent/10.0;
        plotData.f[14] = absSpeedChange;
        plotData.f[15] = avgMotSpeed/10.0;
        wsServer.sendBIN(0, plotData.b, sizeof(plotData.b));
      }
    }
    k++;

    parseSerial();

    // Serial << micros()-tNow << "\t";

    tLast = tNow;

    // Run other tasks
    ArduinoOTA.handle();
    IBus.loop();
    wsServer.loop();

    // Serial << micros()-tNow << endl;
  }

  // delay(1);
}

void parseSerial() {
  static char serialBuf[10];
  static uint8_t pos = 0;
  char currentChar;

  while (Serial.available()) {
    currentChar = Serial.read();
    serialBuf[pos++] = currentChar;
    if (currentChar == 'x') {
      parseCommand(serialBuf, pos);
      pos = 0;
    }
  }

}

void parseCommand(char* data, uint8_t length) {
  float val2;
  if ((data[length-1]=='x') && length>=3) {
    switch (data[0]) {

        //=== other
      case 'c': { // Change controller parameter
        uint8_t controllerNumber = data[1] - '0';
        char cmd2 = data[2];
        float val = atof(data+3);

        // Make a temporary pid object, in which parameters are updated
        PID pidTemp = pidAngle;
        switch (controllerNumber) {
          case 1: pidTemp = pidAngle; break;
          case 2: pidTemp = pidPos;   break;
          case 3: pidTemp = pidSpeed; break;
        }

        switch (cmd2) {
          case 'p': pidTemp.K = val;  break;
          case 'i': pidTemp.Ti = val; break;
          case 'd': pidTemp.Td = val; break;
          case 'n': pidTemp.N = val; break;
          case 't': pidTemp.controllerType = (uint8_t) val; break;
          case 'm': pidTemp.maxOutput = val; break;
          case 'o': pidTemp.minOutput = -val; break;
        }
        pidTemp.updateParameters();

        // Store temporary pid object in correct pid object
        switch (controllerNumber) {
          case 1: pidAngle = pidTemp; break;
          case 2: pidPos = pidTemp;   break;
          case 3: pidSpeed = pidTemp; break;
        }

        Serial << controllerNumber << "\t" << pidTemp.K << "\t" << pidTemp.Ti << "\t" << pidTemp.Td << "\t" << pidTemp.N << "\t" << pidTemp.controllerType << endl;
        break;
      }
      
      case 'y': // PID to use 0=auto 1= go PIDs 2= stay PIDs 
        pidTypeApply = atoi(&data[1]);
        Serial << pidTypeApply << endl;
      break;
          
      case 'a': // Change angle offset
        angleOffset = atof(data+1);
        Serial << angleOffset << endl;
        break;
      case 'f':
        gyroFilterConstant = atof(data+1);
        Serial << gyroFilterConstant << endl;
        break;
      case 'v':
        motorCurrent = atof(data+1);
        Serial << motorCurrent << endl;
        dacWrite(motorCurrentPin, motorCurrent);
        break;
      case 'm':
        val2 = atof(data+1);
        Serial << val2 << endl;
        controlMode = val2;
        break;
      case 'u':
      //
        break;
      case 'g':
        gyroGain = atof(data+1);
        break;
      case 'p': {
        switch (data[1]) {
          case 'e':
            plot.enable = atoi(data+2);
            break;
          case 'p':
            plot.prescaler = atoi(data+2);
            break;
          case 'n': // Noise source enable
            noiseSourceEnable = atoi(data+2);
            break;
          case 'a': // Noise source amplitude
            noiseSourceAmplitude = atof(data+2);
            break;
        }
        break;
      }
      // case 'h':
      //   plot.enable = atoi(data+1);
      //   break;
      // case 'i':
      //   plot.prescaler = atoi(data+1);
      //   break;
      case 'j':
        gyroGain = atof(data+1);
        break;
      case 'k': {
        uint8_t cmd2 = atoi(data+1);
        if (cmd2==1) {  // calibrate gyro
          calculateGyroOffset(100);
        } else if (cmd2==2) {  // calibrate acc
          Serial << "Updating angle offset from " << angleOffset;
          //angleOffset = filterAngle;
          angleOffset = angleOffset + (filterAngle*0.5);
          Serial << " to " << angleOffset << endl;
          preferences.putFloat("angle_offset", angleOffset);
        }
        break;}
      case 'l':
        maxStepSpeed = atof(&data[1]);
        break;
      case 'n':
        gyroFilterConstant = atof(&data[1]);
        break;
      case 'w': {
        char cmd2 = data[1];
        char buf[63];
        uint8_t len;

        switch (cmd2) {
          case 'r':
            Serial.println("Rebooting...");
            ESP.restart();
            // pidParList.sendList(&wsServer);
            break;
          case 'l': // Send wifi networks to WS client
            sendWifiList();
            break;
          case 's': // Update WiFi SSID
            len = length-3;
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            preferences.putBytes("wifi_ssid", buf, 63);
            break;
          case 'k': // Update WiFi key
            len = length-3;
            memcpy(buf, &data[2], len);
            buf[len] = 0;
            preferences.putBytes("wifi_key", buf, 63);
            break;
          case 'm': // WiFi mode (0=AP, 1=use SSID)
            Serial.println(atoi(&data[2]));
            preferences.putUInt("wifi_mode", atoi(&data[2]));
          }
        break;}
    }
  }
}

void sendWifiList(void) {
  char wBuf[200];
  uint8_t n;
  uint16_t pos = 2;

  wBuf[0] = 'w';
  wBuf[1] = 'l';

  Serial.println("Scan started");
  n = WiFi.scanNetworks();

  if (n>5) n = 5; // Limit to first 5 SSIDs

  // Make concatenated list, separated with commas
  for (uint8_t i=0; i<n; i++) {
    pos += sprintf(wBuf + pos, "%s,", WiFi.SSID(i).c_str());
  }
  wBuf[pos-1] = 0;

  Serial.println(wBuf);
  wsServer.sendTXT(0, wBuf);
}

void calculateGyroOffset(uint8_t nSample) {
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  int16_t x, y, z;

  for (uint8_t i=0; i<nSample; i++) {
    imu.getRotation(&x, &y, &z);
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(5);
  }

  gyroOffset[0] = sumX/nSample;
  gyroOffset[1] = sumY/nSample;
  gyroOffset[2] = sumZ/nSample;

  for (uint8_t i=0; i<3; i++) {
    char buf[16];
    sprintf(buf, "gyro_offset_%u", i);
    preferences.putShort(buf, gyroOffset[i]);
  }

  Serial << "New gyro calibration values: " << gyroOffset[0] << "\t" << gyroOffset[1] << "\t" << gyroOffset[2] << endl;
}

void readSensor() {
  int16_t ax, ay, az, gx, gy, gz;
  float deltaGyroAngle;
  float deltaGyroAngle_X;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    accAngle = atan2f((float) ay, (float) az) * 180.0/M_PI - angleOffset;
    deltaGyroAngle = ((float)((gx - gyroOffset[0])) / GYRO_SENSITIVITY) * dT * gyroGain;
    filterAngle = gyroFilterConstant * (filterAngle + deltaGyroAngle) + (1 - gyroFilterConstant) * (accAngle);
   //Serial << ay/1000.0 << "\t" << az/1000.0 << "\t" << accAngle << "\t" << filterAngle << endl;

   
   accAngle_X = atan2f((float) ax, (float) az) * 180.0/M_PI - 5.68;
   deltaGyroAngle_X = -((float)((gx - gyroOffset[1])) / GYRO_SENSITIVITY) * dT * 1;
   filterAngle_X = 0.98 * (filterAngle_X + deltaGyroAngle_X) + (1 - 0.98) * (accAngle_X);
   //Serial << ax/1000.0 << "\t" << az/1000.0 << "\t" << accAngle_X << "\t" << filterAngle_X << endl;
}

void initSensor(uint8_t n) {
  float gyroFilterConstantBackup = gyroFilterConstant;
  gyroFilterConstant = 0.8;
  for (uint8_t i=0; i<n; i++) {
    readSensor();
  }
  gyroFilterConstant = gyroFilterConstantBackup;

}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
                IPAddress ip = wsServer.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
                sendConfigurationData(num);
            }
            break;
        case WStype_TEXT:
            Serial.printf("[%u] get Text: %s\n", num, payload);
            parseCommand((char*) payload, length);
            break;
        case WStype_BIN: {
            //Serial.printf("[%u] get binary length: %u\n", num, length);

            if (length==6) {
              cmd c;
              memcpy(c.arr, payload, 6);
              // Serial << "Binary: " << c.grp << "\t" << c.cmd << "\t" << c.val << "\t" << sizeof(cmd) << endl;
            
              if (c.grp<parList::groupCounter) {
                // if (c.grp==0 && c.cmd<100) {
                //   pidParList.set(c.cmd,c.val);
            
                //   // pidPar[c.cmd].setFloat(c.val);
                // }
                // if (c.cmd==253) {
                //   pidParList.sendList(&wsServer);
                // }
                // if (c.cmd==254) {
                //   pidParList.read();
                // }
                // if (c.cmd==255) {
                //   pidParList.write();
                // }
              } else if (c.grp==100) {
                if (c.cmd==0) {
                  speedInput = c.val;             
                } else if (c.cmd==1) {
                  steerInput = c.val;
          
                }
              } else if (c.grp==105) { 
                lastCntInput = millis();
                String msg = "TELEMETRY: ";
                msg += String(batVoltageLowest);
                msg += "V (L) | ";
                msg += String(batVoltage);
                msg += "V (N) | ";
                msg += String(current_mA);
                msg += "mA | ";
                msg += String(power_mW);
                msg += "mW | Ang: ";
                msg += String(filterAngle);
                if(enableControl == 1) // enabled
                {
                  msg += " | Speed: ";
                  msg += String((motRight.speed+motLeft.speed)/2);
                  msg += " | Step: ";
                  msg += String(motRight.getStep());
                  msg += ":";
                  msg += String(motLeft.getStep());
                  
                  if(InitPIDs())
                  {
                    
                    msg += " [GO PIDs]";
                  }
                  else
                  {
                    msg += " [STAY PIDs]";
                  }
                  
                  if(energySave_ready)
                  {
                    msg += " ES: ";    
                    msg += String(energySave_ready);
                  }
                }
                else // disabled
                {
                  msg += "<font color=\"#0caded\"> [LAYING DOWN]</font>";
                }

                if(calibrationAutoDone)
                {
                  msg += " [CLBR DONE]";
                }
                
                if(batVoltageLowest<DeadBatVoltage) // dead battery
                {
                  msg += "<font color=\"#fe3437\"> [LOW BATTERY]</font>";
                }
                if(current_mA<0) // charging
                {
                  msg += "<font color=\"#08e18e\"> [CHARGING]</font>";
                  msg += " Capacity: ";
                  msg += String(mA_h_count);
                  msg += "mA-h";
                  
                }
                wsServer.sendTXT(0, msg);
              }
            }

            break;
          }
		case WStype_ERROR:
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
    }

}

void sendConfigurationData(uint8_t num) {
  // send message to client
  char wBuf[63];
  char buf[63];
  sprintf(wBuf, "c%dp%.4f", 1, pidAngle.K);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%di%.4f", 1, pidAngle.Ti);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dd%.4f", 1, pidAngle.Td);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dn%.4f", 1, pidAngle.N);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dr%.4f", 1, pidAngle.R);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dm%.4f", 1, pidAngle.maxOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%do%.4f", 1, -pidAngle.minOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dp%.4f", 2, pidPos.K);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%di%.4f", 2, pidPos.Ti);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dd%.4f", 2, pidPos.Td);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dn%.4f", 2, pidPos.N);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dr%.4f", 2, pidPos.R);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dm%.4f", 2, pidPos.maxOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%do%.4f", 2, -pidPos.minOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dp%.4f", 3, pidSpeed.K);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%di%.4f", 3, pidSpeed.Ti);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dd%.4f", 3, pidSpeed.Td);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dn%.4f", 3, pidSpeed.N);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dr%.4f", 3, pidSpeed.R);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%dm%.4f", 3, pidSpeed.maxOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "c%do%.4f", 3, -pidSpeed.minOutput);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "h%.4f", speedFilterConstant);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "i%.4f", steerFilterConstant);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "v%d", motorCurrent);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "j%.4f", gyroGain);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "n%.4f", gyroFilterConstant);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "a%.4f", angleOffset);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "l%.4f", maxStepSpeed);
  wsServer.sendTXT(num, wBuf);
  sprintf(wBuf, "wm%d", preferences.getUInt("wifi_mode", 0));  // 0=AP, 1=Client
  wsServer.sendTXT(num, wBuf);
  preferences.getBytes("wifi_ssid", buf, 63);
  sprintf(wBuf, "ws%s", buf);
  wsServer.sendTXT(num, wBuf);
}
