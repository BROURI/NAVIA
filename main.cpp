#include <Arduino.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteService.h>
#include <BLERemoteCharacteristic.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SoftwareSerial.h>

// Dans un fichier config.h
#define PRODUCTION 1  // 1 pour production, 0 pour dev

#if PRODUCTION
  #define LOG(x)
  #define LOGLN(x)
#else
  #define LOG(x) Serial.print(x)
  #define LOGLN(x) Serial.println(x)
#endif

const int RPWM_PIN = 33;
const int LPWM_PIN = 32;
const int ADC_PIN = 34;


#define PWM_FREQ       8000  // 
#define PWM_RESOLUTION 8      // 8-bit resolution (0-255)
#define PWM_CHANNEL_RPWM 0    // Choose any free channels 0-15
#define PWM_CHANNEL_LPWM 1

bool motorReverse=false;

void stopMotor() {
  ledcWrite(PWM_CHANNEL_LPWM, 0);
  ledcWrite(PWM_CHANNEL_RPWM, 0);
}

void motorSet(int motor_speed) {
  if(motorReverse)motor_speed=-motor_speed;
  bool positiveDirection = true;
  if (motor_speed < 0) {
    positiveDirection = false;
    motor_speed = -motor_speed;
  }

  //if (motor_speed > 255) motor_speed = 255;
  if (motor_speed > 200) motor_speed = 200;

  if (motor_speed == 0) {
    stopMotor();
  } else if (positiveDirection) {
    ledcWrite(PWM_CHANNEL_LPWM, 0);
    ledcWrite(PWM_CHANNEL_RPWM, motor_speed);
  } else { // negative direction
    ledcWrite(PWM_CHANNEL_RPWM, 0);
    ledcWrite(PWM_CHANNEL_LPWM, motor_speed);
  }
}

 unsigned long stopTime = 0;

void moveMotor(String direction) {
  if (direction == "plus") motorSet(200);
  if (direction == "minus") motorSet(-200);
  stopTime = millis() + 500;

}
// Déclaration des variables partagées
volatile bool windMode = false;
volatile int targetHeading = 0;
volatile int heading = 0;
volatile int targetWindAngle = 0;
volatile float windSpeedInKnots = 0;

volatile bool autoPilotOn = false;
volatile int intWindAngle=0;

volatile float Kp = 7.0;
volatile float Kd = 500.0;
volatile float reactivity = 0.5;

TaskHandle_t pidTaskHandle = NULL;

// Fonction de la tâche PID
void pidTask(void *pvParameters) {
  // Variables locales PID (conservent leur état entre les exécutions)
  float previousError = 0.0;

  int correction=0;
  int reverseCorrection=0;
  int windCorrection=0;
  float error =0.0;
  float windError =0.0;

  int adcValue=0;
  float filtereAdcValue=0.0;
  float voltage=0.0;
  float maxVoltage=0.0;
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS; //20Hz
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned long currentMillis = 0;
  unsigned long previousMillis = 0;
  const long windRefreshInterval =1000;
  float derivative=0.0;
  float filteredDerivative =0.0;
  float filteredCorrection =0.0;

  
  for(;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    if(autoPilotOn) {
      // Calcul PID
      adcValue=analogRead(ADC_PIN);
      voltage = adcValue * (3.3 / 4095.0); // Convert to voltage (12-bit ADC)
      if(voltage>maxVoltage)maxVoltage=voltage;
      //Serial.print("adcValue: ");
      //Serial.println(adcValue);

      //Serial.print("          maxVoltage: ");
      //Serial.println(maxVoltage);

      //Serial.print("    voltage: ");
      //Serial.println(voltage);

      filtereAdcValue = 0.96 * filtereAdcValue + 0.04 * adcValue;
      //Serial.print("                          filtereAdcValue: ");
      //Serial.println(filtereAdcValue);
      error = targetHeading - heading;
      if (error > 180) error -= 360;
      else if (error < -180) error += 360;
      
      if (windMode){
        currentMillis = millis();
        if(currentMillis-previousMillis>=windRefreshInterval && windSpeedInKnots>=2.0){
          previousMillis = currentMillis;
          windError=targetWindAngle-intWindAngle;
          if (windError > 180) windError -= 360;
          if (windError < -180) windError += 360;

          windCorrection = round(windError*0.1);
          if(windCorrection>=1)windCorrection=1;
          if(windCorrection<=-1)windCorrection=-1;
          targetHeading -= windCorrection;

          if(targetHeading>360)targetHeading-=360;
          if(targetHeading<0)targetHeading+=360;
        }
       
      }
      // Normalisation de l'erreur
      derivative = error - previousError;
      filteredDerivative = 0.4 * derivative + 0.6 * filteredDerivative; 

      correction = int(Kp * error + Kd * filteredDerivative);
      filteredCorrection = int(reactivity * correction + (1-reactivity) * filteredCorrection); 
      
      // Application de la correction
      if(abs(error) > 0) {
        if(filtereAdcValue<550){
          motorSet(filteredCorrection);
        }else{
          reverseCorrection = -filteredCorrection/7;
          if(reverseCorrection>0){reverseCorrection=100;}
          else {reverseCorrection=-100;}
          motorSet(int(reverseCorrection));
        }
      } else {
          stopMotor();
      }
      
      previousError = error;
      
      #if !PRODUCTION
        LOG("heading: "); LOG(heading);
        LOG(" | targetHeading: "); LOG(targetHeading);
        LOG(" | correction: "); LOG(correction);
        //LOG(" | error: "); LOG(error);
        if(windMode&&windSpeedInKnots>2.0){
          LOG(" | intWindAngle: "); LOG(intWindAngle);
          LOG(" | targetWindAngle: "); LOG(targetWindAngle);
          //LOG(" | windError: "); LOG(windError);
          LOG(" | windCorrection: "); LOG(windCorrection);
        }
        LOG(" | windSpeed: "); LOGLN(windSpeedInKnots);



      #endif
    } else {
      //stopMotor();
      if (millis() >= stopTime) {
        stopMotor();
      }
    }
  }
}


SoftwareSerial _mySerial;

void send2ST(const uint8_t cmd[], int bytes)
{
        for (int i = 0; (i < bytes); i++)
        {
            (i == 0) ? _mySerial.write(cmd[i], PARITY_MARK) : _mySerial.write(cmd[i], PARITY_SPACE);
            delay(1);
        }
}


void sendData(uint16_t data);

////////////// SERVER DEFINITION //////////////

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pLedCharacteristic = NULL;
BLECharacteristic* pHeadingCharacteristic = NULL;


bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

//// pilot signals
uint32_t  bitTime = 208;
const int txPin = 2; // GPIO pin for transmission
/*
uint16_t ST_Minus_1[] =  { 0x186, 0x21, 0x05, 0xFA };
uint16_t ST_Minus_10[] = { 0x186, 0x21, 0x06, 0xF9 };
uint16_t ST_Plus_1[] =   { 0x186, 0x21, 0x07, 0xF8 };
uint16_t ST_Plus_10[] =  { 0x186, 0x21, 0x08, 0xF7 };
uint16_t ST_Auto[] =     { 0x186, 0x21, 0x01, 0xFE };
uint16_t ST_Standby[] =  { 0x186, 0x21, 0x02, 0xFD };
*/
uint8_t ST_Minus_1[4] = {0x86, 0x11, 0x05, 0xFA};
uint8_t ST_Minus_10[4] = {0x86, 0x11, 0x06, 0xF9};
uint8_t ST_Plus_1[4] = {0x86, 0x11, 0x07, 0xF8};
uint8_t ST_Plus_10[4] = {0x86, 0x11, 0x08, 0xF7};
uint8_t ST_Auto[4] = {0x86, 0x11, 0x01, 0xFE};
uint8_t ST_Standby[4] = {0x86, 0x11, 0x02, 0xFD};


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define LED_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"
#define HEADING_CHARACTERISTIC_UUID "19b10003-e8f2-537e-4f6c-d104768a1214"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};
void sendData(uint16_t data) {
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW); 
  delayMicroseconds(bitTime); 
  for (int i = 0; i <= 8; i++) {
    uint8_t bit = (data >> i) & 0x01;
    if (bit == 0) {
      digitalWrite(txPin, LOW); 
    } else if (bit == 1){
      digitalWrite(txPin, HIGH);  
    }
    delayMicroseconds(bitTime); 
  }
  digitalWrite(txPin, HIGH); 
  delayMicroseconds(bitTime); 
}

void sendDataList(uint16_t dataArray[], int dataCount ){
  noInterrupts();
  digitalWrite(txPin, HIGH); 
  delayMicroseconds(11*bitTime); 
  for (int i = 0; i < dataCount; i++) {
   sendData(dataArray[i]);
  }
  interrupts();
 }


 bool isPositiveInteger(const String &s) {
  if (s.length() == 0) return false;
    if (s[0] == '-') return false;
  for (size_t i = 0; i < s.length(); i++) {
      if (!isdigit(s[i])) return false;
  }
  return true;
}
class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {

  void onRead(BLECharacteristic* pCharacteristic) {
      LOGLN("miiiiiiiiiw");
      pCharacteristic->setValue(String("miiiiiiiiiiw").c_str());

    }
   
    void onWrite(BLECharacteristic* pLedCharacteristic) {
    std::string value = pLedCharacteristic->getValue();
    if (value.length() > 0) {
      String receivedValue = value.c_str();
      //Serial.print("tablet data: ");
      //Serial.println(receivedValue); // Print the integer value
      if(isPositiveInteger(receivedValue)){
        heading = receivedValue.toInt();
      }

      else if (receivedValue==("autoPilotOn")) {
        digitalWrite(txPin, HIGH);
        targetHeading = heading;
        autoPilotOn = true;

      }
      
      else if (receivedValue==("motorReverse")) {
        motorReverse=!motorReverse;
        Serial.println("motorReverse");
      }
      else if (receivedValue == "autoPilotOff") {
        digitalWrite(txPin, LOW);
        stopMotor();
        autoPilotOn=false;
        windMode=false;
      }
      else if (receivedValue.startsWith("dGain")) {
        Kd=receivedValue.substring(6).toFloat();
        Serial.print("dGain ");
        Serial.println(Kd);
      }
      else if (receivedValue.startsWith("pGain")) {
        Kp=receivedValue.substring(6).toFloat();
        Serial.print("pGain ");
        Serial.println(Kp);
      }
      else if (receivedValue.startsWith("reactivity")) {
        reactivity=receivedValue.substring(11).toFloat();
        Serial.print("reactivity ");
        Serial.println(reactivity);
      }
      else if (receivedValue == "plus1") {
        targetHeading +=1;
        if(targetHeading>360)targetHeading-=360;
        if(!autoPilotOn){
          moveMotor("plus");
        }
      }
      else if (receivedValue == "minus1") {
        targetHeading -=1;
        if(targetHeading<0)targetHeading+=360;
        if(!autoPilotOn){
          moveMotor("minus");
        }
      }
     else if (receivedValue == "plus10") {
      targetHeading +=10;
      if(targetHeading>360)targetHeading-=360;
    }
    else if (receivedValue == "minus10") {
      targetHeading -=10;
      if(targetHeading<0)targetHeading+=360;
    }
    else if (receivedValue == "plus90") {
      targetHeading +=90;
      if(targetHeading>360)targetHeading-=360;
    }
    else if (receivedValue == "minus90") {
      targetHeading -=90;
      if(targetHeading<0)targetHeading+=360;
    }
    else if (receivedValue == "windModeOn") {
      if(windSpeedInKnots>=2.0){
        windMode =true;
        targetWindAngle = intWindAngle;
      }

    }
    else if (receivedValue == "windModeOff") {
      windMode =false;
      targetHeading = heading;
    }

    }
  }
};


////////////// CLIENT DEFINITION//////////////

std::string targetDeviceName = "ULTRASONIC";
BLEScan* pBLEScan;
BLEAdvertisedDevice* utrasonicDevice;
bool ultrasonicDeviceFound = false;
BLERemoteCharacteristic* pRemoteCharacteristic;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        if (advertisedDevice.getName() == targetDeviceName) {
            Serial.println("ULTRASONIC found !");
            ultrasonicDeviceFound = true;
            utrasonicDevice = new BLEAdvertisedDevice(advertisedDevice);
            pBLEScan->stop();
        }
    }
};

std::string windAngle ="";
std::string windSpeed ="";
std::string windInfos ="";
float windSpeedInMPS = 0;
float floatWindSpeed=0;
float windSpeedInKPH=0;
String windData = "-,-";


void ultrasonicNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    //Serial.print("received data: ");
    //Serial.write(pData, length);
    //Serial.println();
    //Serial.println(std::to_string(length).c_str());

    std::string windInfos(reinterpret_cast<char*>(pData), length);
    windAngle = windInfos.substr(3, 3);
    windSpeed = windInfos.substr(6, 5);

    //Serial.println(("wind angle :"+ windAngle).c_str());
    //Serial.println(("wind seed :"+ windSpeed).c_str());
    
    for (char& c : windSpeed) {
        if (!std::isdigit(c)) { // Si le caractère n'est pas un chiffre
            c = '0';
        }
    }
    intWindAngle = std::stof(windAngle);
    floatWindSpeed = std::stof(windSpeed);
    //Serial.println(("intWindAngle : "+std::to_string(intWindAngle)).c_str());
    //Serial.println(("floatWindSpeed : "+std::to_string(floatWindSpeed)).c_str());
    windSpeedInMPS = floatWindSpeed/1000;
    windSpeedInKnots = windSpeedInMPS * 1.94384;
        // send wind value to tablet
        if (deviceConnected) {
            windData=String(intWindAngle)+","+String(windSpeedInKnots);
            pSensorCharacteristic->setValue(windData.c_str());
            //Serial.print("                         wind data: ");
            LOGLN(pSensorCharacteristic->getValue().c_str());
        }
}

void connectToUltrasonicDevice() {
    BLEClient* pClient = BLEDevice::createClient();
    pClient->connect(utrasonicDevice);
    Serial.println("Connecté à ULTRASONIC");
    
    BLERemoteService* pRemoteService = pClient->getService("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    if (pRemoteService) {
        pRemoteCharacteristic = pRemoteService->getCharacteristic("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
        if (pRemoteCharacteristic) {
            Serial.println("Caractéristique de réception trouvée, activation des notifications...");
            pRemoteCharacteristic->registerForNotify(ultrasonicNotifyCallback);
        } else {
            Serial.println("Caractéristique non trouvée.");
        }
    } else {
        Serial.println("Service non trouvé.");
    }
}




void motorSetup() {
  ledcSetup(PWM_CHANNEL_RPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RPWM_PIN, PWM_CHANNEL_RPWM);

  ledcSetup(PWM_CHANNEL_LPWM, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LPWM_PIN, PWM_CHANNEL_LPWM);

  stopMotor();

  Serial.println("Motor setup with ledcWrite at 10kHz");
}


void setup() {

  pinMode(ADC_PIN, INPUT);

/////////// SERVER SETUP //////////
    Serial.begin(115200);
    motorSetup();
    pinMode(txPin, OUTPUT);
    // bool invert = false
    //_mySerial.begin(4800, SWSERIAL_8S1, 32, 33, false, 95);

    // Create the BLE SERVER
    BLEDevice::init("NAVIA");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pSensorCharacteristic = pService->createCharacteristic(
                        SENSOR_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ
                    );
    //pSensorCharacteristic->addDescriptor(new BLE2902());

    // Create the ON button Characteristic
    pLedCharacteristic = pService->createCharacteristic(
                        LED_CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ||BLECharacteristic::PROPERTY_WRITE
                    );
    // Register the callback for the ON button characteristic
    pLedCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pLedCharacteristic->addDescriptor(new BLE2902());

// Set the callback for the heading characteristic


    //pLedCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    
    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("Waiting a client connection to notify...");

///////////  CLIENT SETUP //////////

    BLEDevice::init("CLIENT");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);

    // Création de la tâche PID
  xTaskCreatePinnedToCore(
    pidTask,     // Fonction de la tâche
    "PIDTask",   // Nom de la tâche
    10000,       // Taille de la pile
    NULL,        // Paramètres
    1,           // Priorité (plus haut = plus prioritaire)
    &pidTaskHandle, // Handle de la tâche
    0            // Core (0 ou 1)
  );

}





void loop() {

    //// ultrasonic connection
    if (!ultrasonicDeviceFound) {
        //Serial.println("Scan BLE en cours...");
        pBLEScan->start(1, false);
        //="-,-";
    } else {
        connectToUltrasonicDevice();
        ultrasonicDeviceFound = false; // Pour éviter une reconnexion en boucle
    }

    //// Tablet connection
    if (!deviceConnected && oldDeviceConnected) {
        Serial.println("Device disconnected.");
        //delay(50); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
      }
      // connecting to tablet
      if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
        Serial.println("Tablet Connected");
      }

      delay(1000);

}
