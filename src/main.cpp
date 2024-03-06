#include <Arduino.h>
#include <SwitecX25.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Client Code
#include "BLEDevice.h"

// Global variables
bool connected = false;
bool doScan = false;
BLEAdvertisedDevice* myDevice;
bool doConnect = false;

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

// OLED display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Stepper motor
#define STEPS 945
SwitecX25 motor1(STEPS, D0, D1, D2, D3);

// Button pin
#define BUTTON_PIN D8

// Potentiometer pin
#define POT_PIN A9

// Refresh rate counter
int refreshClk = 0;

// Status flag
enum Status { WAITING, ON, OFF };
Status currentStatus = WAITING;

// Function declarations
void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
bool connectToServer();

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
    doConnect = true; // 添加此行
    Serial.println("Connected to the BLE Server.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("BLE Status: ");
    display.println("Connected");
    display.display();
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Disconnected from the BLE Server.");
    doScan = true; // Restart scanning for other devices
    currentStatus = WAITING;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("BLE Status: ");
    display.println("Disconnected");
    display.display();
  }
};


class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};

void setup(void) {
  // Motor initialization
  motor1.zero();
  motor1.setPosition(0); // Initial position set to 0 (0 degree)
  motor1.update();
  delay(1000);

  // OLED display initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Waiting for order...");
  display.display();

  // Serial port initialization
  Serial.begin(9600);
  Serial.print("Enter a step position from 0 through ");
  Serial.print(STEPS - 1);
  Serial.println(".");

  // BLE Client initialization
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void loop(void) {

  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("Attempting to connect to the BLE Server.");
    } else {
      Serial.println("Failed to connect to the BLE Server.");
    }
    doConnect = false;
  }

  if (connected) {
    // Handle actions when connected to BLE Server

    // Button control to reset position
    bool buttonPressed = digitalRead(BUTTON_PIN) == LOW;
    if (buttonPressed) {
      motor1.setPosition(0); // Reset position when button is pressed
    }

    // Reading potentiometer value
    int potValue = analogRead(POT_PIN);
    potValue = map(potValue, 0, 4095, 0, STEPS - 1); // Map potentiometer value to motor position
    int motorPosition = potValue;

    // Update motor position based on currentStatus
    if (currentStatus == ON) {
      motor1.setPosition(180); // Assume 180 degrees clockwise
    } else if (currentStatus == OFF) {
      motor1.setPosition(-180); // Assume 180 degrees counterclockwise
    }

    // Update motor position
    motor1.update();

    Serial.print("Current motor position: ");
    Serial.println(motorPosition);

    // OLED display update
    if (refreshClk == 10) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("BLE Status: ");
      display.println("Connected");

      switch (currentStatus) {
        case ON:
          display.setCursor(0, 10);
          display.println("Turn on for you !");
          break;
        case OFF:
          display.setCursor(0, 10);
          display.println("Turn off for you !");
          break;
        default:
          display.setCursor(0, 10);
          display.println("Waiting for order...");
      }

      display.display();
      refreshClk = 0;
    }
    refreshClk++;
  } else {
    // Display disconnected status on OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("BLE Status: ");
    display.println("Disconnected");
    display.display();
  }
  delay(10); // Adjust delay time to control motor speed
}

void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  String dataString = "";
  for (size_t i = 0; i < length; i++) {
    dataString += (char)pData[i];
  }

  Serial.print("Received data: ");
  Serial.println(dataString);

  // Update motor position based on received data
  if (dataString.equals("on")) {
    Serial.println("Received: on");
    currentStatus = ON;
  } else if (dataString.equals("off")) {
    Serial.println("Received: off");
    currentStatus = OFF;
  } else {
    Serial.println("Received: Unknown data");
    // Handle other cases
  }

  // Update motor position
  if (currentStatus == ON) {
    motor1.setPosition(180); // Update motor position for 'on' signal (180 degrees clockwise)
  } else if (currentStatus == OFF) {
    motor1.setPosition(-180); // Update motor position for 'off' signal (180 degrees counterclockwise)
  }
}

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  if (pClient->connect(myDevice)) {
    Serial.println(" - Connected to server");
  } else {
    Serial.println(" - Failed to connect to server.");
    return false;
  }

  pClient->setMTU(517);

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  if (pRemoteCharacteristic->canRead()) {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  return true;
}
