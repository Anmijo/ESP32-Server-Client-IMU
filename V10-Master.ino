#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <SPI.h>
#include <SD.h>

//Sensor Stuff
#define SDA_1 41
#define SCL_1 40

TwoWire I2Cone = TwoWire(0);
// foot
Adafruit_LSM6DSOX accelgyro1;
Adafruit_LIS3MDL mag1;
sensors_event_t aevent1;
sensors_event_t wevent1;
sensors_event_t mevent1;
sensors_event_t temp1;


//Led Stuff
#define NUMPIXELS     1
#define NEOPIXELPIN  39
#define NEOPIXELPOWER 38
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXELPIN, NEO_GRB + NEO_KHZ800);

//Save stuff
File myFile;

const int CS = 16;

struct packet {
  unsigned long int t;
  int16_t a1[3];
  int16_t w1[3];
} data;  // 22 bytes

const unsigned long interval = 10;  //delay interval
unsigned long previousMillis = 0;   // previous time

//flush/save timer
const unsigned long saveTime = 10000;  //time between every save/flush of the data
unsigned long previousSave = 0;        // last time it saved


//Bluetooth stuff
#define bleServerName "SMART_PROSTHETIC" //BLE server name

bool deviceConnected = false;

// See the following for generating UUIDs:https://www.uuidgenerator.net/
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"

// Temperature Characteristic and Descriptor
BLECharacteristic serviceCharacteristics("cba1d466-344c-4be3-ab3f-189f80dd7518", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor serviceDescriptor(BLEUUID((uint16_t)0x2902));


//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};


void setup() {
  // Start serial communication 
  Serial.begin(115200);

//save stuff
  Serial.println("Initializing SD card...");

  if (!SD.begin(CS)) {
     Serial.println("initialization failed!");
     pixels.fill(0x0000FF);
    pixels.show();
    while (1);
  }

  Serial.println("initialization done.");
  openSD();

//sensor stuff
  I2Cone.begin(SDA_1, SCL_1, 400000);
  while (!accelgyro1.begin_I2C(106, &I2Cone) || !mag1.begin_I2C(LIS3MDL_I2CADDR_DEFAULT, &I2Cone)) {
    Serial.println("Ooops, no IMU-1 foot detected ... Check your wiring!");
    pixels.fill(0x0000FF);
    pixels.show();
  }

  while (!mag1.begin_I2C(LIS3MDL_I2CADDR_DEFAULT, &I2Cone)) {
    Serial.println("Ooops, no IMU-1 mgn foot detected ... Check your wiring!");
    pixels.fill(0x0000FF);
    pixels.show();
  }

  accelgyro1.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  accelgyro1.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  accelgyro1.setAccelDataRate(LSM6DS_RATE_104_HZ);
  accelgyro1.setGyroDataRate(LSM6DS_RATE_104_HZ);

  mag1.setDataRate(LIS3MDL_DATARATE_80_HZ);
  mag1.setRange(LIS3MDL_RANGE_8_GAUSS);



  //Led Stuff
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(NEOPIXELPOWER, OUTPUT);
  digitalWrite(NEOPIXELPOWER, HIGH);
  pixels.begin(); 
  pixels.setBrightness(100); 

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *masterService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics and Create a BLE Descriptor
    masterService->addCharacteristic(&serviceCharacteristics);
    serviceDescriptor.setValue("Slave On Command");
    serviceCharacteristics.addDescriptor(new BLE2902());

  
  // Start the service
  masterService->start();

  while(!deviceConnected){
    pixels.fill(0xFF0000);
    pixels.show();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a slave to connect to notify...");
  delay(10);
  }

  while(deviceConnected){  
   digitalWrite(LED_BUILTIN, HIGH); 
   Serial.println("Connected");
   delay(10000);

    // send value
    static char onValue[6];
    dtostrf(1, 6, 2, onValue);

    serviceCharacteristics.setValue(onValue);
    serviceCharacteristics.notify();

  return;
  }

}

void loop() {
  pixels.fill(0x00FF00);
  pixels.show();

  
  accelgyro1.getEvent(&aevent1, &wevent1, &temp1);
  data.a1[0] = aevent1.acceleration.x;
  data.a1[1] = aevent1.acceleration.y;
  data.a1[2] = aevent1.acceleration.z;
  data.w1[0] = wevent1.gyro.x;
  data.w1[1] = wevent1.gyro.y;
  data.w1[2] = wevent1.gyro.z;

  if (myFile) {
    unsigned long currentMillis = millis();  //remove this

    if (currentMillis - previousMillis >= interval) {  //remove this
      previousMillis = currentMillis;                  //remove this

      Serial.println("Writing to File...");
      myFile.print(millis());
      myFile.print(",");
      myFile.print(data.a1[0]);
      myFile.print(",");
      myFile.print(data.a1[1]);
      myFile.print(",");
      myFile.print(data.a1[2]);
      myFile.print(",");
      myFile.print(data.w1[0]);
      myFile.print(",");
      myFile.print(data.w1[1]);
      myFile.print(",");
      myFile.println(data.w1[2]);

      if (currentMillis - previousSave >= saveTime) {
        previousSave = currentMillis;
        myFile.flush();
        Serial.println("File Flushed and Saved");
      }
    }
  } else {
    Serial.println("error opening file ");
  }
  
  }






void openSD(void){
  char file_name[80];
  int file_name_count = 0;

  sprintf(file_name, "/dataFile%03d.csv", file_name_count);
  while (SD.exists(file_name)){sprintf(file_name, "/dataFile%03d.csv", ++file_name_count);}

  myFile = SD.open(file_name, FILE_WRITE);  //Open SD Card
  Serial.print("initialization succeeded! \nSaving to ");
  Serial.println(file_name);

}