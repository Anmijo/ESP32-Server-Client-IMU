#include "BLEDevice.h"
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
//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "SMART_PROSTHETIC"

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
static BLEUUID ServiceUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");

// BLE Characteristics
  static BLEUUID serviceCharacteristicUUID("cba1d466-344c-4be3-ab3f-189f80dd7518");

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;
 
//Characteristicd that we want to read
static BLERemoteCharacteristic* serviceCharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

//to store sent value
char* value;

//Flags to check whether new values are available
boolean newValue = false;

//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
   BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
 
  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(ServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(ServiceUUID.toString().c_str());
    return (false);
  }
 
  // Obtain a reference to the characteristics in the service of the remote BLE server.
  serviceCharacteristic = pRemoteService->getCharacteristic(serviceCharacteristicUUID);

  if (serviceCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");
 
  //Assign callback functions for the Characteristics
  serviceCharacteristic->registerForNotify(serviceNotifyCallback);
  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};
 
//When the BLE Server sends a new value reading with the notify property
static void serviceNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
  //store] value
  value = (char*)pData;
  newValue = true;
}

void setup() {

  //Start serial communication
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Slave application...");

    //Led Stuff
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(NEOPIXELPOWER, OUTPUT);
  digitalWrite(NEOPIXELPOWER, HIGH);
  pixels.begin(); 
  pixels.setBrightness(100); 

  Serial.println("pinmodes done");

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

  Serial.println("In setup");


  accelgyro1.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  accelgyro1.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);

  accelgyro1.setAccelDataRate(LSM6DS_RATE_104_HZ);
  accelgyro1.setGyroDataRate(LSM6DS_RATE_104_HZ);

  mag1.setDataRate(LIS3MDL_DATARATE_80_HZ);
  mag1.setRange(LIS3MDL_RANGE_8_GAUSS);

  Serial.println("collected data");



  //Init BLE device
  BLEDevice::init("");
 
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 100 seconds.




      Serial.println("before while");

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  while(!connected){
      Serial.println("not connected");

    pixels.fill(0xFF0000);
    pixels.show();
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(100);

    if (doConnect == true) {
      if (connectToServer(*pServerAddress)) {
        Serial.println("We are now connected to the BLE Server.");
        //Activate the Notify property of each Characteristic
        serviceCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        connected = true;
      } else {
        Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      }
      doConnect = false;
    }
  }

  //check if value sent
  while(connected){
    
   Serial.println("Connected");
    digitalWrite(LED_BUILTIN, HIGH);

    if (newValue){
      return;
    }   
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