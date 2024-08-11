// Paranormal Sensor Station by Two Faces
// M5StickC plus2 version v2_1
#include <M5StickCPlus2.h>
#include <M5Unified.h>
#include "M5UnitENV.h"
#include <Wire.h>
#include "ClosedCube_SHT31D.h"
#include "HX711.h"
#include "EEPROM.h"
#define EEPROM_SIZE 300
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Update.h>
#include "logo.h"
char *baseSSID;
String ssid;
const char *host = "PSS";
AsyncWebServer server(80);
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicData = NULL;
BLECharacteristic *pCharacteristicReceivedData = NULL;
bool deviceConnected = false;
#define LOADCELL_DOUT_PIN 25
#define LOADCELL_SCK_PIN 26
HX711 scale;
ClosedCube_SHT31D sht3xd;
QMP6988 qmp;
SHT31D sht;

#define M5_LED 19

// Rem Duo pins. use pins 32 (26) and 33 (36) for RemDuo on port 2 instead
#define pin26 26
#define pin36 36

volatile int pulseCountPin26 = 2000;
volatile int pulseCountPin36 = 2000;
int lastPulseCountPin26 = 2000;
int lastPulseCountPin36 = 2000;
int maxHzPin26 = 2000;
int maxHzPin36 = 2000;
int targetMaxHzPin26 = 2000;
int targetMaxHzPin36 = 2000;
const long interval = 2000;        // calbration duration to get mode ms
const long transitionTime = 1000;  // calibration correction time ms
long previousMillis = 0;
int historyPin26[3] = { 0 };
int historyPin36[3] = { 0 };
int ynHistory[3] = { 0 };
int historyIndex = 0;
int ynHistoryIndex = 0;
int currentPulseCountPin26;
int currentPulseCountPin36;
int pulsesPerSecondPin26;
int pulsesPerSecondPin36;
int percentagePin36;
int percentagePin26;
int pdn;
int ysn;
int ysnSkin = 30;  /// determines remduo alert percentage based on human touch = 100%
int calibDelay = 45;
int calibRem = 45;

float tcal = 4;  // calibrate thermometer by subtracting this number from the reading

bool state = false;
bool old_state = false;
byte oct = 1;
int id = 10000;
byte bright = 3;
byte nbright = 64;
byte cpu = 24;  // x10 MHz
bool btON = true;
byte alertHold = 10;  // time to display one alert, must do this many cycles to consider a new alert
int sleepr;
int maxSleep = 60;
unsigned long lastActivityTime;
bool batAlert = false;
bool batAlerted = false;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float temp = 0.0F;
float humi = 0.0F;
float pres = 0.0F;
float aX;
float aY;
float aZ;
float at;
float ah;
float ap;
float oldX;
float oldY;
float oldZ;
float oldt;
float oldh;
float oldp;

bool tempOn = false;
bool temp2On = false;
bool scaleOn = false;
bool remOn = false;
bool press = false;

byte event[14];
byte lastEvent;
byte lastTimer;
bool pol;  //last polarity of reading (positive if true, otherwise negative)

float sens = .025;
float tsens = .030;
float ssens = .02;
int rsens = 1;

byte tick;

float weight;
float maxWeight;
float minWeight;
bool scalib = false;
float loadCellReadings[3] = { 0 };  // Array to hold the last three load cell readings
int currentReadingIndex = 0;        // Index to keep track of the current reading

const float alpha = 0.08;          // Smoothing factor, adjust as needed (0 < alpha <= 1)
float emaBatteryPercentage = 100;  // Initial EMA value
byte previousBatteryPercentage = 100;
int upwardTrendCounter = 0;
const int trendThreshold = 2;  // Adjust this value as needed
bool isTrendingUpward = false;
bool findme = false;

void IRAM_ATTR handleInterruptPin26() {
  pulseCountPin26++;
}

void IRAM_ATTR handleInterruptPin36() {
  pulseCountPin36++;
}

int calculateMode(int values[], int length) {
  int number = values[0];
  int mode = number;
  int count = 1;
  int countMode = 1;
  for (int i = 1; i < length; i++) {
    if (values[i] == number) {
      count++;
    } else {
      if (count > countMode) {
        countMode = count;
        mode = number;
      }
      count = 1;
      number = values[i];
    }
  }
  return mode;
}

void reseter() {
  lastEvent = 0;
  for (int c = 0; c < 14; c++) {
    event[c] = 0;
  }
  tick = 0;

  checkPer();
  if (remOn) {
    calibRem = calibDelay;
  } else if (scaleOn) {
    maxWeight = 0;
    minWeight = 0;
    calibScale();
  } else if (!tempOn || !temp2On) {
    delay(2000);
  }
  aX = accX * 3;
  aY = accY * 3;
  aZ = accZ * 3;
  if (tempOn) {
    sht = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_HIGH, SHT3XD_MODE_POLLING, 50);
    temp = sht.t;
    humi = sht.rh;
    at = temp * 3;
    ah = humi * 3;
    ap = pres * 3;
  }
}

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();  // Get the value that was written
    char letter2;
    int vt;
    int number;
    if (rxValue.length() > 0) {
      char letter1 = rxValue[0];
      if (rxValue.length() > 1) {
        letter2 = rxValue[1];  // This could be a number if only one letter was sent
        vt = rxValue[1] - '0';
      }
      if (rxValue.length() > 2) {
        number = atoi(rxValue.substr(2).c_str());
      }

      if ((letter1 == 'r' && letter2 == 'e') || (letter1 == 'r' && letter2 == 'd') || (letter1 == 'a' && letter2 == 'a') || (letter1 == 'a' && !letter2)) {
        btGo();
        wake();
      }
      if ((letter1 == 'b' && letter2 == 'e') || (letter1 == 'b' && letter2 == 'p') || (letter1 == 'b' && !letter2)) {
        findme = true;
      }
      if (letter1 == 'b' && letter2 == 't') {
        batAlert = true;
      }
      if ((letter1 == 'b' && letter2 == 'l') || (letter1 == 'f' && letter2 == 'l') || (letter1 == 'f' && !letter2)) {
        for (int fla = 0; fla < 20; fla++) {
          digitalWrite(M5_LED, HIGH);
          delay(100);
          digitalWrite(M5_LED, LOW);
          delay(100);
        }
      }
      if (letter1 == 'r' && letter2 == 'b') {
        ESP.restart();
      }
      if ((letter1 == 'z' && letter2 == 'z') || (letter1 == 'z' && !letter2)) {
        wake();
        reseter();
      }
      if (letter1 == 'f' && letter2 == 'r') {
        wipeEEPROM();
        ESP.restart();
      }
      if ((letter1 == 'a' && letter2 == 's') && (number < 6)) {
        float sval = number;
        sens = (5 * (sval + 3.000)) / 1000;
        EEPROM.put(9, sens);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(sval) + "   " + String(sens, 3));
      }
      if (letter1 == 'a' && vt >= 0 && vt < 6) {
        float sval = vt;
        sens = (5 * (sval + 3.000)) / 1000;
        EEPROM.put(9, sens);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(sval) + "   " + String(sens, 3));
      }
      if ((letter1 == 't' && letter2 == 's') && (number < 6 && number)) {
        float sval = number;
        tsens = (5 * (sval + 4.000)) / 1000;
        EEPROM.put(17, tsens);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(sval) + "   " + String(tsens, 3));
      }
      if (letter1 == 't' && vt > 0 && vt < 6) {
        float sval = vt;
        tsens = (5 * (sval + 4.000)) / 1000;
        EEPROM.put(17, tsens);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(sval) + "   " + String(tsens, 3));
      }
      if ((letter1 == 's' && letter2 == 's') && (number < 6 && number)) {
        float sval = number;
        ssens = sval / 100.000;
        EEPROM.put(13, ssens);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(sval) + "   " + String(ssens, 3));
      }
      if (letter1 == 's' && vt > 0 && vt < 6) {
        float sval = vt;
        ssens = sval / 100.000;
        EEPROM.put(13, ssens);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(sval) + "   " + String(ssens, 3));
      }
      if ((letter1 == 'r' && letter2 == 's') && (number < 6 && number)) {
        rsens = number;
        EEPROM.put(21, rsens);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(rsens));
      }
      if (letter1 == 'r' && vt > 0 && vt < 6) {
        rsens = vt;
        EEPROM.put(21, rsens);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(rsens));
      }
      if ((letter1 == 'v' && letter2 == 'o') && (number < 6)) {
        oct = number;
        EEPROM.writeByte(7, oct);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(oct));
      }
      if (letter1 == 'v' && vt >= 0 && vt < 6) {
        oct = vt;
        EEPROM.writeByte(7, oct);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(oct));
      }
      if ((letter1 == 'i' && letter2 == 'd') && number < 99999 && number && number > 0) {
        id = number;
        EEPROM.writeByte(2, lowByte(id));
        EEPROM.writeByte(3, highByte(id));
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(id));
      }
      if (((letter1 == 'c' && letter2 == 'p') || (letter1 == 'm' && letter2 == 'h')) && number < 25 && number > 7) {
        byte nnum = number;
        cpu = nnum;
        EEPROM.writeByte(6, cpu);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(cpu));
        setCpuFrequencyMhz(cpu * 10);
      }
      if ((letter1 == 'w' && letter2 == 'i') || (letter1 == 'f' && letter2 == 'w') || (letter1 == 'w' && letter2 == 'f')) {
        startWifi();
      }
      if (letter1 == 'b' && letter2 == 'r' && number < 6) {
        bright = number;
        brightScale();
        EEPROM.writeByte(4, bright);
        EEPROM.writeByte(5, nbright);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(bright) + " " + String(nbright));
        M5.Lcd.setBrightness(nbright);
      }
      if (letter1 == 's' && letter2 == 'l' && number < 301) {
        sleepr = number;
        if (!number) wake();
        EEPROM.put(31, sleepr);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(number));
      }
      if (letter1 == 't' && letter2 == 'c' && number < 2000) {
        tcal = number / 100.00;
        EEPROM.put(35, tcal);
        EEPROM.commit();
        Serial.println(String(letter1) + String(letter2) + String(tcal));
      }
    }
  }
};

void handleRoot(AsyncWebServerRequest *request) {
  String html = "<html><body>";
  html += "<h1>Two Faces PSS Firmware ver 2.1</h1>";
  html += "<form id='uploadForm' method='POST' action='/update' enctype='multipart/form-data'>";
  html += "<input type='file' name='update'><br><br>";
  html += "<input type='submit' id='uploadBtn' value='Upload Firmware' onclick='startUpload()'>";
  html += "</form>";
  html += "<script>";
  html += "function startUpload() {";
  html += "  var btn = document.getElementById('uploadBtn');";
  html += "  btn.value = 'Uploading...';";
  html += "}";
  html += "</script>";
  html += "</body></html>";
  request->send(200, "text/html", html);
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404);
}

void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  static int totalSize = 0;

  if (!index) {
    Serial.println("Starting firmware update");
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Serial.println("Error: Firmware update could not begin");
    }
  }

  if (Update.write(data, len) != len) {
    Serial.println("Error: Writing firmware data failed");
  }

  totalSize += len;

  if (final) {
    if (Update.end(true)) {
      Serial.println("Firmware update completed");
      Serial.print("Total size: ");
      Serial.println(totalSize);
      request->send(200, "text/html", "<br><br>Firmware update <b>completed</b>. Your Two Faces PSS will restart in a few seconds.");
      delay(2000);
      ESP.restart();
    } else {
      Serial.println("Error: Firmware update failed");
      request->send(200, "text/html", "<br><br>Two Faces PSS Firmware <b>update failed</b>. Please try again.<br><i>Be sure to use the file ending in .bin<br>if it ends in .zip double-click it and move the .bin file out of the zip folder first.</i><br><br><a href='/'><button>Try Again</button></a>");
    }
  }
}

void startWifi() {
  baseSSID = "PSS_";
  ssid = String(baseSSID) + String(id);
  WiFi.softAP(ssid.c_str(), "");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);
  server.onFileUpload(handleFileUpload);
  server.on(
    "/update", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(200);
    },
    handleFileUpload);
  server.begin();
}

void wifiDraw() {
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.print(" Wifi\n");
  M5.Lcd.print(" Firmware\n");
  M5.Lcd.print(" Update\n\n");
  M5.Lcd.setCursor(5, 90);
  M5.Lcd.print("Find SSID:\n");
  M5.Lcd.setCursor(5, 110);
  M5.Lcd.print(ssid);
  M5.Lcd.setCursor(5, 140);
  M5.Lcd.print("Go to URL:\n");
  M5.Lcd.setCursor(0, 160);
  M5.Lcd.print("192.168.4.1\n");
}

byte getBatteryLevel(void) {
  byte currentBatteryPercentage = M5.Power.getBatteryLevel();
  emaBatteryPercentage = alpha * currentBatteryPercentage + (1 - alpha) * emaBatteryPercentage;
  byte batteryPercentage = (byte)emaBatteryPercentage;
  if (batteryPercentage > previousBatteryPercentage) {
    upwardTrendCounter++;
    if (upwardTrendCounter >= trendThreshold) {
      isTrendingUpward = true;
    }
  } else if (batteryPercentage < previousBatteryPercentage) {
    upwardTrendCounter = 0;
    isTrendingUpward = false;
  }
  previousBatteryPercentage = batteryPercentage;
  if (batteryPercentage < 10) {
    batAlert = true;
  }
  if (batAlert && !batAlerted) {
    wake();
    if (btON && deviceConnected) {
      String data;
      data += "BAT" + String(id);
      pCharacteristicData->setValue(data.c_str());
      pCharacteristicData->notify();
    }
    if (oct) {
      for (int sad = 80; sad > 10; sad--) {
        M5.Speaker.tone(1310 * (sad / 10), 10);
        delay(10);
      }
    }
    for (int bl = 0; bl < 7; bl++) {
      digitalWrite(M5_LED, HIGH);
      delay(200);
      digitalWrite(M5_LED, LOW);
      delay(200);
    }
    if (tempOn) {  // fix temp fluct during alarm;
      sht = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_HIGH, SHT3XD_MODE_POLLING, 50);
      temp = sht.t;
      at = temp * 3;
    }
    batAlerted = true;
  }
  if (batteryPercentage > 25 && batAlert) {
    batAlert = false;
    batAlerted = false;
    wake();
  }
  return batteryPercentage;
}

void wake(void) {
  lastActivityTime = millis();  // Update the time of the last activity.
  if (batAlert) {
    M5.Lcd.setBrightness(1);
  } else {
    M5.Lcd.setBrightness(nbright);
  }
}

void wipeEEPROM() {
  for (int a = 0; a < EEPROM_SIZE; a++) {
    EEPROM.writeByte(a, 0);
  }
  EEPROM.commit();
}

void save() {
  EEPROM.writeByte(1, 1);
  EEPROM.writeByte(2, lowByte(id));
  EEPROM.writeByte(3, highByte(id));
  EEPROM.writeByte(4, bright);
  EEPROM.writeByte(5, nbright);
  EEPROM.writeByte(6, cpu);
  EEPROM.writeByte(7, oct);
  EEPROM.writeByte(8, btON);
  EEPROM.put(9, sens);
  EEPROM.put(13, ssens);
  EEPROM.put(17, tsens);
  EEPROM.put(21, rsens);
  EEPROM.put(31, sleepr);
  EEPROM.put(35, tcal);
  EEPROM.commit();
}

void load() {
  id = (EEPROM.read(3) << 8) | EEPROM.read(2);
  bright = EEPROM.readByte(4);
  nbright = EEPROM.readByte(5);
  cpu = EEPROM.readByte(6);
  oct = EEPROM.readByte(7);
  btON = EEPROM.readByte(8);
  EEPROM.get(9, sens);
  EEPROM.get(13, ssens);
  EEPROM.get(17, tsens);
  EEPROM.get(21, rsens);
  EEPROM.get(31, sleepr);
  EEPROM.get(35, tcal);
  if (nbright < 0 || nbright > 255) {
    nbright = 64;
    EEPROM.writeByte(5, nbright);
    EEPROM.commit();
  }
  M5.Lcd.setBrightness(nbright);
  if (cpu < 8 || cpu > 24) {
    cpu = 8;
    EEPROM.writeByte(6, cpu);
    EEPROM.commit();
  }
  setCpuFrequencyMhz(cpu * 10);
}

void checkPer() {
  // check peripherals
  pinMode(pin26, INPUT_PULLUP);
  pinMode(pin36, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin26), handleInterruptPin26, FALLING);
  attachInterrupt(digitalPinToInterrupt(pin36), handleInterruptPin36, FALLING);
  delay(2000);                                             // Wait to accumulate some pulses
  if (pulseCountPin26 > 2001 && pulseCountPin36 > 2001) {  // check for rem duo
    remOn = true;
    if (cpu != 24) {
      cpu = 24;
      EEPROM.writeByte(6, cpu);
      EEPROM.commit();
      setCpuFrequencyMhz(cpu * 10);
    }
  } else {  // no rem duo
    detachInterrupt(digitalPinToInterrupt(pin26));
    detachInterrupt(digitalPinToInterrupt(pin36));
    pinMode(25, INPUT);      // return things to non-rem state
    if (!digitalRead(25)) {  // check if strain gauge load cell is connected
                             // no load cell, do env
      Wire.end();
      delay(100);
      Wire.begin(0, 26);
      sht3xd.begin(0x44);
      sht = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_HIGH, SHT3XD_MODE_POLLING, 50);
      if (sht.error == SHT3XD_NO_ERROR) {  // check if env hat is connected
        qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, 0, 26, 3400000U);
        qmp.update();
        pres = qmp.pressure;
        temp = sht.t;
        humi = sht.rh;
        tempOn = true;
      } else {
        tempOn = false;
      }
    } else {  // yes load cell
      pinMode(0, OUTPUT);
      digitalWrite(0, LOW);
      scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
      scale.set_scale(16.4f);
      if (scale.get_units(1)) {
        scaleOn = true;
        splash();
      }
    }
  }
  if (!tempOn) {
    //check port b
    Wire.end();
    delay(100);
    Wire.begin(32, 33);
    sht3xd.begin(0x44);
    sht = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_HIGH, SHT3XD_MODE_POLLING, 50);
    if (sht.error == SHT3XD_NO_ERROR) {
      qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, 32, 33, 3400000U);
      qmp.update();
      pres = qmp.pressure;
      temp = sht.t;
      humi = sht.rh;
      temp2On = true;
      sens = .015;  // turn off accel alerts
    } else {
      temp2On = false;
    }
  }
}  // end perif check

void setup() {
  M5.begin();

  Serial.begin(115200);

  M5.Lcd.setBrightness(64);
  setCpuFrequencyMhz(240);

  M5.update();
  bool ba = M5.BtnA.isPressed();
  bool bb = M5.BtnB.isPressed();
  if (ba && bb) {
    wipeEEPROM();
    ESP.restart();
  }

  EEPROM.begin(EEPROM_SIZE);
  if (EEPROM.readByte(1) == 1) {
    load();
  } else {
    save();
  }

  M5.Lcd.setRotation(0);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);

  M5.Imu.begin();
  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, LOW);

  if (btON) {
    String idString = String(id);
    String deviceName = "PSS_" + idString;
    BLEDevice::init(deviceName.c_str());
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Service
    BLEService *pService = pServer->createService(BLEUUID("180A"));

    // Create BLE Characteristic
    pCharacteristicData = pService->createCharacteristic(
      BLEUUID("2A46"),  // 2A3D device info, 2A46 for Alert
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    pService->addCharacteristic(pCharacteristicData);

    pCharacteristicReceivedData = pService->createCharacteristic(
      BLEUUID("2A3D"),                   // or Device Name 2A00
      BLECharacteristic::PROPERTY_WRITE  // Make it writeable from the client side
    );

    pService->addCharacteristic(pCharacteristicReceivedData);

    pCharacteristicReceivedData->setCallbacks(new MyCallbacks());  // Set the callbacks for this characteristic

    pCharacteristicData->addDescriptor(new BLE2902());
    pCharacteristicReceivedData->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting for connections...");
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);
    //    pAdvertising->setMinPreferred(0xC80);  // Set the advertising interval to 2 seconds
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->start();
  }

  splash();

  checkPer();

  M5.Imu.getAccelData(&accX, &accY, &accZ);

  aX = accX * 3;
  aY = accY * 3;
  aZ = accZ * 3;
  at = temp * 3;
  ah = humi * 3;
  ap = pres * 3;

  delay(1000);
  M5.Lcd.fillScreen(BLACK);
}  // end setup

void splash() {
  M5.Lcd.fillScreen(RED);
  M5.Lcd.setCursor(0, 115);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(BLACK, RED);
  M5.Lcd.print(" Two Faces\n");
  M5.Lcd.print(" Paranormal\n");
  M5.Lcd.print(" Sensor\n");
  M5.Lcd.print(" Station\n");
  M5.Lcd.print(" ver 2.1\n\n");

  M5.Lcd.setTextSize(1);
  M5.Lcd.print(" Left Btn: Reset/Pwr.\n");
  M5.Lcd.print(" Right Btn: Beep/Menu.\n");
  M5.Lcd.print(" Front Btn: SensThresh");

  M5.Lcd.drawXBitmap(17, 5, logo, 100, 100, RED, BLACK);
}

char nth_digit(int val, int n) {
  char buffer[7];  // enough space for large negative int and null terminator
  sprintf(buffer, "%i", val);
  return buffer[0] == '-' ? buffer[n + 1] : buffer[n];
}

void brightScale() {
  switch (bright) {
    case 0:
      nbright = 1;
      break;
    case 1:
      nbright = 8;
      break;
    case 2:
      nbright = 32;
      break;
    case 3:
      nbright = 64;
      break;
    case 4:
      nbright = 128;
      break;
    case 5:
      nbright = 255;
      break;
    default:
      nbright = 8;
      break;
  }
}

void set() {  //////////// SETTINGS screen
  bool resetConfirm = false;
  bool redraw = true;
  byte param = 2;
  int idset;
  digitalWrite(M5_LED, LOW);  // led off
  while (1) {
    M5.update();
    if (M5.BtnB.wasReleased()) {
      if (param == 0 && idset < 4) {
        idset++;
      } else {
        idset = 0;
        param++;
        if (param > 7) param = 0;
      }
      redraw = true;
    }
    if (M5.BtnA.wasReleased()) {
      switch (param) {
        case 0:
          id = id + pow(10, idset);
          if (id > 99999) id = id - 100000;
          redraw = true;
          break;
        case 1:
          if (btON) {
            btON = false;
          } else {
            btON = true;
          }
          redraw = true;
          break;
        case 2:
          bright++;
          if (bright > 5) bright = 0;
          redraw = true;
          brightScale();
          M5.Lcd.setBrightness(nbright);
          break;
        case 3:
          cpu = cpu * 2;
          if (cpu == 32) cpu = 24;
          if (cpu > 24) cpu = 8;
          redraw = true;
          setCpuFrequencyMhz(cpu * 10);
          break;
        case 4:
          sleepr += 5;
          if (sleepr > maxSleep) sleepr = 0;
          redraw = true;
          break;
        case 5:
          startWifi();
          wifiDraw();
          while (1) {
            M5.update();
            if (M5.BtnA.wasReleased()) {
              save();
              ESP.restart();
              M5.Lcd.fillScreen(BLACK);
              return;
            }
          }
          break;
        case 6:
          if (resetConfirm) {
            wipeEEPROM();
            ESP.restart();
          } else {
            resetConfirm = true;
            redraw = true;
          }
          break;
        case 7:
          save();
          ESP.restart();
          return;
          break;
      }
    }
    if (redraw) {
      redraw = false;
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 0);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setTextColor(BLACK, ORANGE);
      M5.Lcd.print(" SETTINGS  \n");
      if (param == 0) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print("   ");
        for (int dg = 0; dg < 5 - idset; dg++) {
          M5.Lcd.print(" ");
        }
        M5.Lcd.print("_");
      }
      M5.Lcd.print("\n");
      if (param == 0) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(">");
      } else {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.print(" ");
      }
      M5.Lcd.printf("Id=%5d\n", id);
      if (param == 1) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(">");
      } else {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.print(" ");
      }
      if (btON) {
        M5.Lcd.printf("Blue2th=ON\n");
      } else {
        M5.Lcd.printf("Blue2th=no\n");
      }
      if (param == 2) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(">");
      } else {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.print(" ");
      }
      M5.Lcd.printf("Bright =%2d\n", bright);
      if (param == 3) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(">");
      } else {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.print(" ");
      }
      M5.Lcd.printf("Mhz  = %2d0\n", cpu);
      if (param == 4) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(">");
      } else {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.print(" ");
      }
      if (sleepr) {
        M5.Lcd.printf("Sleep =%3d\n", sleepr);
      } else {
        M5.Lcd.print("Sleep = no");
      }
      if (param == 5) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(">");
      } else {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.print(" ");
      }
      M5.Lcd.printf("Firmware\n", cpu);
      if (param == 6) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(">");
      } else {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.print(" ");
      }
      if (resetConfirm) {
        M5.Lcd.print("R U Sure?\n");
      } else {
        M5.Lcd.print("Reset All\n");
      }
      if (param == 7) {
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.print(">");
      } else {
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.print(" ");
      }
      M5.Lcd.print("EXIT/SAVE");

      M5.Lcd.setTextSize(1);
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.print("\n\n\n Right btn: setting\n\n");
      M5.Lcd.print(" Front btn: value\n\n");
      M5.Lcd.print(" Lower Bright or Mhz\n");
      M5.Lcd.print(" to Save Battery");
      while (M5.BtnB.isPressed()) {
        M5.update();
      }
    }

  }  // end while
  M5.update();
  delay(300);
}  // end void

void checkButtons() {  //////////////////////////////  BUTTONS
  bool reset;

  M5.update();
  if (M5.BtnA.pressedFor(1000)) {
    press = true;
    scalib = false;
    wake();
    if (remOn || scaleOn) {
      sens += .005;
      if (sens > .04) sens = .015;
      M5.Lcd.setCursor(30, 30);
      M5.Lcd.setTextSize(7);
      M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, ORANGE);
      M5.Lcd.setTextColor(BLACK, ORANGE);
      M5.Lcd.printf("a%1.0f\n", sens * 1000 / 5 - 3);
      M5.Lcd.setTextSize(1);
      M5.Lcd.print(" acel sens thresh");
      EEPROM.put(9, sens);
      EEPROM.commit();
      Serial.println("Accel Sens:" + String(sens, 3));
    }
    if (tempOn || temp2On) {
      tsens += .005;
      if (tsens > .045) tsens = .025;
      M5.Lcd.setCursor(30, 30);
      M5.Lcd.setTextSize(7);
      M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, YELLOW);
      M5.Lcd.setTextColor(BLACK, YELLOW);
      M5.Lcd.printf("t%1.0f\n", tsens * 1000 / 5 - 4);
      M5.Lcd.setTextSize(1);
      M5.Lcd.print(" temp sens thresh");
      EEPROM.put(17, tsens);
      EEPROM.commit();
    }
    delay(300);
    while (M5.BtnA.isPressed()) {
      M5.update();
    }
  } else if (M5.BtnA.wasReleased()) {
    press = true;
    scalib = false;
    if ((sleepr) && (millis() - lastActivityTime >= sleepr * 1000)) {
      wake();
    } else {
      if (remOn) {
        rsens++;
        if (rsens > 5) rsens = 1;
        M5.Lcd.setCursor(30, 30);
        M5.Lcd.setTextSize(7);
        M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, M5.Lcd.color565(0, 255, 128));
        M5.Lcd.setTextColor(BLACK, M5.Lcd.color565(0, 255, 128));
        M5.Lcd.printf("r%1.0d\n", rsens);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print(" rem duo thresh");
        EEPROM.put(21, rsens);
        EEPROM.commit();
      } else if (scaleOn) {
        ssens += .01;
        if (ssens > .07) ssens = .01;
        M5.Lcd.setCursor(30, 30);
        M5.Lcd.setTextSize(7);
        M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, M5.Lcd.color565(0, 255, 128));
        M5.Lcd.setTextColor(BLACK, M5.Lcd.color565(0, 255, 128));
        M5.Lcd.printf("s%1.0f\n", ssens * 100);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print(" bar sens thresh");
        EEPROM.put(13, ssens);
        EEPROM.commit();
      } else {
        sens += .005;
        if (sens > .04) sens = .015;
        M5.Lcd.setCursor(30, 30);
        M5.Lcd.setTextSize(7);
        M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, ORANGE);
        M5.Lcd.setTextColor(BLACK, ORANGE);
        M5.Lcd.printf("a%1.0f\n", sens * 1000 / 5 - 3);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print(" acel sens thresh");
        EEPROM.put(9, sens);
        EEPROM.commit();
        Serial.println("Accel Sens:" + String(sens, 3));
      }
    }
    delay(200);
  }
  if (M5.BtnB.pressedFor(1000)) {
    wake();
    set();
  } else if (M5.BtnB.wasReleased()) {
    oct++;
    if (oct > 5) oct = 0;
    EEPROM.writeByte(7, oct);
    EEPROM.commit();
    wake();
  }
  if (M5.BtnPWR.wasReleased()) {
    // esp_restart();
    wake();
    splash();
    reseter();
    M5.Lcd.fillScreen(BLACK);
  }
}



void doRem() {  ////////// REM DUO
  currentPulseCountPin26 = pulseCountPin26;
  currentPulseCountPin36 = pulseCountPin36;

  pulsesPerSecondPin26 = currentPulseCountPin26 - lastPulseCountPin26;
  pulsesPerSecondPin36 = currentPulseCountPin36 - lastPulseCountPin36;

  // Store the current pulses per second in the history
  historyPin26[historyIndex] = pulsesPerSecondPin26;
  historyPin36[historyIndex] = pulsesPerSecondPin36;
  historyIndex = (historyIndex + 1) % 3;

  // Calculate the mode for the last 3 seconds
  int modePin26 = calculateMode(historyPin26, 3);
  int modePin36 = calculateMode(historyPin36, 3);

  // Check if we should reset the max values
  if (millis() - previousMillis >= interval) {
    targetMaxHzPin26 = modePin26;
    targetMaxHzPin36 = modePin36;
    previousMillis = millis();
  }

  // Smoothly transition to the new max values over 1 second
  unsigned long elapsed = millis() - previousMillis;
  if (elapsed < transitionTime) {
    float alpha = (float)elapsed / transitionTime;
    maxHzPin26 = (1 - alpha) * maxHzPin26 + alpha * targetMaxHzPin26;
    maxHzPin36 = (1 - alpha) * maxHzPin36 + alpha * targetMaxHzPin36;
  }

  // Calculate the percentage of the current pulses per second relative to the max
  percentagePin36 = (pulsesPerSecondPin36 * 100) / maxHzPin36;
  percentagePin26 = (pulsesPerSecondPin26 * 100) / maxHzPin26;

  // new extreme values clean to tame feedback and speed up auto calib
  if (percentagePin36 > 100) percentagePin36 = 100;
  if (percentagePin26 > 100) percentagePin26 = 100;
  if (percentagePin36 < 0) percentagePin36 = 0;
  if (percentagePin26 < 0) percentagePin26 = 0;
  if (pulsesPerSecondPin36 > 9999) pulsesPerSecondPin36 = 5000;
  if (pulsesPerSecondPin26 > 9999) pulsesPerSecondPin26 = 5000;
  if (!pulsesPerSecondPin36) pulsesPerSecondPin36 = 1;
  if (!pulsesPerSecondPin26) pulsesPerSecondPin26 = 1;

  pdn = percentagePin26 - percentagePin36;

  if (pdn > 0) {
    ysn = (pdn * 100) / ysnSkin;
  } else if (pdn < 0) {
    ysn = (abs(pdn) * 100) / ysnSkin;
  }

  ysn = constrain(ysn, 0, 100);

  lastPulseCountPin26 = currentPulseCountPin26;
  lastPulseCountPin36 = currentPulseCountPin36;

  // Serial.println("Rem: y" + String(percentagePin36) + " n" + String(percentagePin26) + "   Y " + pulsesPerSecondPin36 + "   N " + pulsesPerSecondPin26);  ///// temp serial get vals to clean extremes at calib
}

float getMedian(float a, float b, float c) {
  if ((a < b && b < c) || (c < b && b < a)) return b;
  if ((b < a && a < c) || (c < a && a < b)) return a;
  return c;
}
void calibScale() {
  if (scale.get_units(1)) {
    for (int i = 0; i < 3; i++) {
      loadCellReadings[i] = 0;
    }
    currentReadingIndex = 0;  // Reset the current reading index
    scale.tare();
  } else {
    scaleOn = false;
  }
}

void loop() {  ///////////////////  LOOP

  oldX = aX / 3;
  oldY = aY / 3;
  oldZ = aZ / 3;

  if (tempOn) {
    oldt = at / 3;
    oldh = ah / 3;
    oldp = ap / 3;
  }

  if (scaleOn) {
    // get weight median of 3
    float newReading = scale.get_units(1) / 1000;
    loadCellReadings[currentReadingIndex] = newReading;
    currentReadingIndex = (currentReadingIndex + 1) % 3;
    weight = getMedian(loadCellReadings[0], loadCellReadings[1], loadCellReadings[2]);
    if (weight > maxWeight) maxWeight = weight;
    if (weight < minWeight) minWeight = weight;
    if (minWeight < -99.0) minWeight = -99.0;
    if (maxWeight > 99.0) maxWeight = 99.0;
  }
  sht = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_HIGH, SHT3XD_MODE_POLLING, 50);
  if (sht.error == SHT3XD_NO_ERROR) {
    if (!tempOn) {
      qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, 0, 26, 3400000U);
      tempOn = true;
    }
  } else {
    tempOn = false;
  }
  if (tempOn) {
    qmp.update();
    pres = qmp.pressure;
    temp = sht.t;
    humi = sht.rh;
  }

  if (remOn) {
    doRem();
    if (calibRem) {
      calibRem--;
    }
  }

  M5.Imu.getAccelData(&accX, &accY, &accZ);

  if (!lastTimer) {  //////////////////// ALERTS

    if ((sens > .015) && ((accX > oldX + sens) || (accX < oldX - sens))) {
      state = true;
      lastTimer = alertHold;
      event[1]++;
      if (event[1] > 99) event[1] = 0;
      lastEvent = 1;
      if (accX > oldX + sens) {
        pol = true;
      } else {
        pol = false;
      }
    } else if ((sens > .015) && ((accY > oldY + sens) || (accY < oldY - sens))) {
      state = true;
      lastTimer = alertHold;
      event[2]++;
      if (event[2] > 99) event[2] = 0;
      lastEvent = 2;
      if (accY > oldY + sens) {
        pol = true;
      } else {
        pol = false;
      }
    } else if ((sens > .015) && ((accZ > oldZ + sens) || (accZ < oldZ - sens))) {
      state = true;
      lastTimer = alertHold;
      event[3]++;
      if (event[3] > 99) event[3] = 0;
      lastEvent = 3;
      if (accZ > oldZ + sens) {
        pol = true;
      } else {
        pol = false;
      }
    } else if ((tempOn) && (temp > oldt + tsens * 2)) {
      state = true;
      lastTimer = alertHold;
      event[4]++;
      if (event[4] > 99) event[4] = 0;
      lastEvent = 4;
      pol = true;
    } else if ((tempOn) && (temp < oldt - tsens * 2)) {
      state = true;
      lastTimer = alertHold;
      event[7]++;
      if (event[7] > 99) event[7] = 0;
      lastEvent = 7;
      pol = false;
    } else if ((tempOn) && (humi > oldh + tsens * 20)) {
      state = true;
      lastTimer = alertHold;
      event[5]++;
      if (event[5] > 99) event[5] = 0;
      lastEvent = 5;
      pol = true;
    } else if ((tempOn) && (humi < oldh - tsens * 20)) {
      state = true;
      lastTimer = alertHold;
      event[8]++;
      if (event[8] > 99) event[8] = 0;
      lastEvent = 8;
      pol = false;
    } else if ((tempOn) && (pres > oldp + tsens * 200)) {
      state = true;
      lastTimer = alertHold;
      event[6]++;
      if (event[6] > 99) event[6] = 0;
      lastEvent = 6;
      pol = true;
    } else if ((tempOn) && (pres < oldp - tsens * 200)) {
      state = true;
      lastTimer = alertHold;
      event[9]++;
      if (event[9] > 99) event[9] = 0;
      lastEvent = 9;
      pol = false;
    } else if ((scalib) && (scaleOn) && (weight > ssens)) {
      state = true;
      lastTimer = alertHold;
      event[10]++;
      if (event[10] > 99) event[10] = 0;
      lastEvent = 10;
    } else if ((scalib) && (scaleOn) && (weight < ssens - (ssens * 2))) {
      state = true;
      lastTimer = alertHold;
      event[11]++;
      if (event[11] > 99) event[11] = 0;
      lastEvent = 11;
    } else if ((remOn) && (pdn > rsens) && (!calibRem)) {
      state = true;
      lastTimer = alertHold;
      event[12]++;
      if (event[12] > 99) event[12] = 0;
      lastEvent = 12;
      calibRem = alertHold * 2;
    } else if ((remOn) && (pdn < 0 - rsens) && (!calibRem)) {
      state = true;
      lastTimer = alertHold;
      event[13]++;
      if (event[13] > 99) event[13] = 0;
      lastEvent = 13;
      calibRem = alertHold * 2;
    } else {
      state = false;
    }
  }

  aX = aX - oldX + accX;
  aY = aY - oldY + accY;
  aZ = aZ - oldZ + accZ;
  if (tempOn || temp2On) {
    at = at - oldt + temp;
    ah = ah - oldh + humi;
    ap = ap - oldp + pres;
  }
  if (!old_state && state) {
    event[0]++;
    if (event[0] > 99) event[0] = 0;
    digitalWrite(M5_LED, HIGH);  // led on
    if (oct) {
      switch (lastEvent) {
        case 1:
          M5.Speaker.tone((1310 * oct), 100);
          break;
        case 2:
          M5.Speaker.tone((1470 * oct), 100);
          break;
        case 3:
          M5.Speaker.tone((1640 * oct), 100);
          break;

        case 4:
          M5.Speaker.tone((1310 * oct + (lastEvent == 14)), 100);
          delay(100);
          M5.Speaker.tone((1470 * oct + (lastEvent == 14)), 100);
          break;
        case 5:
          M5.Speaker.tone((1470 * oct + (lastEvent == 15)), 100);
          delay(100);
          M5.Speaker.tone((1650 * oct + (lastEvent == 15)), 100);
          break;
        case 6:
          M5.Speaker.tone((1650 * oct + (lastEvent == 16)), 100);
          delay(100);
          M5.Speaker.tone((1960 * oct + (lastEvent == 16)), 100);
          break;
        case 7:
          M5.Speaker.tone((1470 * oct + (lastEvent == 17)), 100);
          delay(100);
          M5.Speaker.tone((1310 * oct + (lastEvent == 17)), 100);
          break;
        case 8:
          M5.Speaker.tone((1650 * oct + (lastEvent == 18)), 100);
          delay(100);
          M5.Speaker.tone((1470 * oct + (lastEvent == 18)), 100);
          break;
        case 9:
          M5.Speaker.tone((1960 * oct + (lastEvent == 19)), 100);
          delay(100);
          M5.Speaker.tone((1650 * oct + (lastEvent == 19)), 100);
          break;

        case 10:
          M5.Speaker.tone((2200 * oct), 100);
          delay(100);
          M5.Speaker.tone((9880 * oct), 100);
          break;
        case 11:
          M5.Speaker.tone((2200 * oct), 100);
          delay(100);
          M5.Speaker.tone((7830 * oct), 100);
          break;

        case 12:
          M5.Speaker.tone((2200 * oct), 100);
          delay(100);
          M5.Speaker.tone((2470 * oct), 100);
          break;
        case 13:
          M5.Speaker.tone((2200 * oct), 100);
          delay(100);
          M5.Speaker.tone((1960 * oct), 100);
          break;
      }
    }
  } else {
    digitalWrite(M5_LED, LOW);  // led off
  }
  old_state = state;

  disp();

  checkButtons();

  if ((sleepr) && (millis() - lastActivityTime >= sleepr * 1000)) {
    M5.Lcd.setBrightness(0);
  }

  if (findme) {
    for (int joy = 10; joy < 80; joy++) {
      M5.Speaker.tone(1310 * (joy / 10), 10);
      delay(10);
    }
    for (int bl = 0; bl < 7; bl++) {
      digitalWrite(M5_LED, HIGH);
      delay(200);
      digitalWrite(M5_LED, LOW);
      delay(200);
    }
    findme = false;
    if (tempOn) {  // fix temp fluct during alarm;
      sht = sht3xd.readTempAndHumidity(SHT3XD_REPEATABILITY_HIGH, SHT3XD_MODE_POLLING, 50);
      temp = sht.t;
      at = temp * 3;
    }
  }
}
void tDisp() {
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.printf("\n t+");
  if (lastEvent == 4) M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("%2d", event[4]);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.printf(" t-");
  if (lastEvent == 7) M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("%2d ", event[7]);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.printf("\n h+");
  if (lastEvent == 5) M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("%2d", event[5]);
  M5.Lcd.setTextColor(CYAN, BLACK);
  M5.Lcd.printf(" h-");
  if (lastEvent == 8) M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("%2d ", event[8]);
  M5.Lcd.setTextColor(MAGENTA, BLACK);
  M5.Lcd.printf("\n p+");
  if (lastEvent == 6) M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("%2d", event[6]);
  M5.Lcd.setTextColor(MAGENTA, BLACK);
  M5.Lcd.printf(" p-");
  if (lastEvent == 9) M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.printf("%2d ", event[9]);
}

void disp() {  //////////////// DISPLAY

  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(BLACK, M5.Lcd.color565(192, 192, 128));
  M5.Lcd.print(id % 10);
  M5.Lcd.setTextColor(BLACK, M5.Lcd.color565(128, 128, 255));
  M5.Lcd.print("v");
  M5.Lcd.print(oct);
  if (scaleOn) {
    M5.Lcd.setTextColor(BLACK, M5.Lcd.color565(0, 255, 128));
    M5.Lcd.printf("s%1.0f", ssens * 100);
  } else if (remOn) {
    M5.Lcd.setTextColor(BLACK, M5.Lcd.color565(0, 255, 128));
    M5.Lcd.printf("r%1.0d", rsens);
  }
  if (tempOn || temp2On) {
    M5.Lcd.setTextColor(BLACK, YELLOW);
    M5.Lcd.printf("t%1.0f", tsens * 1000 / 5 - 4);
  }
  if (!scaleOn && !tempOn && !temp2On && !remOn) {
    M5.Lcd.setTextColor(BLACK, ORANGE);
    M5.Lcd.printf(" ");
  }
  if (!(scaleOn && tempOn) && !(tempOn && remOn)) {
    M5.Lcd.setTextColor(BLACK, ORANGE);
    M5.Lcd.printf("a%1.0f", sens * 1000 / 5 - 3);
  }
  if (!scaleOn && !tempOn && !temp2On && !remOn) M5.Lcd.printf(" ");
  int bat;
  M5.Lcd.setTextColor(BLACK, WHITE);
  bat = getBatteryLevel();
  if (bat == 100) {
    M5.Lcd.setTextColor(BLACK, GREEN);
  } else if (isTrendingUpward) {
    M5.Lcd.setTextColor(BLACK, RED);
  }
  M5.Lcd.printf("b%3d", bat);

  M5.Lcd.drawLine(0, 15, M5.Lcd.width(), 15, BLACK);

  if (lastTimer) {                     ////// ALERTS       // large alert on screen that a parameter exceeding threshold event has occured
    lastTimer--;                       //  timer for how many screen refesh cycles to show the alert
    if (lastTimer == alertHold - 1) {  // draw on first timer cycle
      btGo();
      wake();
      M5.Lcd.setCursor(30, 30);
      M5.Lcd.setTextSize(7);
      switch (lastEvent) {
        case 0:
          lastTimer = 0;
          break;
        case 1:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, RED);
          M5.Lcd.setTextColor(BLACK, RED);
          M5.Lcd.printf("X");
          break;
        case 2:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, GREEN);
          M5.Lcd.setTextColor(BLACK, GREEN);
          M5.Lcd.printf("Y");
          break;
        case 3:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, BLUE);
          M5.Lcd.setTextColor(BLACK, BLUE);
          M5.Lcd.printf("Z");
          break;
        case 4:
        case 7:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, YELLOW);
          M5.Lcd.setTextColor(BLACK, YELLOW);
          M5.Lcd.printf("T");
          break;
        case 5:
        case 8:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, CYAN);
          M5.Lcd.setTextColor(BLACK, CYAN);
          M5.Lcd.printf("H");
          break;
        case 6:
        case 9:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, MAGENTA);
          M5.Lcd.setTextColor(BLACK, MAGENTA);
          M5.Lcd.printf("P");
          break;
        case 10:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, TFT_GREENYELLOW);
          M5.Lcd.setTextColor(BLACK, TFT_GREENYELLOW);
          M5.Lcd.setTextSize(2);
          M5.Lcd.setCursor(30, 5);
          M5.Lcd.printf("\n   %.3f", weight);
          M5.Lcd.setTextSize(7);
          M5.Lcd.setCursor(5, 37);
          M5.Lcd.printf("YES");
          scalib = false;
          break;
        case 11:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, ORANGE);
          M5.Lcd.setTextColor(BLACK, ORANGE);
          M5.Lcd.setTextSize(2);
          M5.Lcd.setCursor(30, 5);
          M5.Lcd.printf("\n   %.3f", abs(weight));
          M5.Lcd.setTextSize(7);
          M5.Lcd.setCursor(30, 37);
          M5.Lcd.printf("NO");
          scalib = false;
          break;
        case 12:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, TFT_GREENYELLOW);
          M5.Lcd.setTextColor(BLACK, TFT_GREENYELLOW);
          M5.Lcd.setTextSize(2);
          M5.Lcd.setCursor(30, 5);
          M5.Lcd.printf("\n   %3d%%", ysn);
          M5.Lcd.setTextSize(7);
          M5.Lcd.setCursor(5, 37);
          M5.Lcd.printf("YES");
          break;
        case 13:
          M5.Lcd.fillRect(0, 16, M5.Lcd.width() - 3, 80, ORANGE);
          M5.Lcd.setTextColor(BLACK, ORANGE);
          M5.Lcd.setTextSize(2);
          M5.Lcd.setCursor(30, 5);
          M5.Lcd.printf("\n   %3d%%", ysn);
          M5.Lcd.setTextSize(7);
          M5.Lcd.setCursor(30, 37);
          M5.Lcd.printf("NO");
          break;
      }
      if ((lastEvent >= 1 && lastEvent < 10) || (lastEvent >= 14 && lastEvent < 20)) {
        if (pol) {
          M5.Lcd.printf("+\n");
        } else {
          M5.Lcd.printf("-\n");
        }
        M5.Lcd.setTextSize(1);
        if (lastEvent >= 1 && lastEvent < 4) {
          M5.Lcd.print(" accelerometer motion");
        } else if ((lastEvent == 4) || (lastEvent == 7) || (lastEvent == 4) || (lastEvent == 7)) {
          M5.Lcd.print(" temperature change");
        } else if ((lastEvent == 5) || (lastEvent == 8) || (lastEvent == 5) || (lastEvent == 8)) {
          M5.Lcd.print(" humidity change");
        } else if ((lastEvent == 6) || (lastEvent == 9) || (lastEvent == 6) || (lastEvent == 9)) {
          M5.Lcd.print(" barometric pressure");
        }
      }
    } else  // still alert hold, but not the first cycle
      if (((lastEvent == 10) || (lastEvent == 11)) && (!press)) {
        if (lastEvent == 10) M5.Lcd.setTextColor(BLACK, TFT_GREENYELLOW);
        if (lastEvent == 11) M5.Lcd.setTextColor(BLACK, ORANGE);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(30, 5);
        M5.Lcd.printf("\n   %.3f", abs(weight));
      }
  } else {
    press = false;
    M5.Lcd.setTextColor(WHITE, BLACK);
    if (!tempOn) {
      M5.Lcd.printf("\n Event:");
      M5.Lcd.printf("%3d ", event[0]);
    } else {
      M5.Lcd.printf("e%2d", event[0]);
    }

    if (remOn) {
      if (calibRem) {
        M5.Lcd.setTextColor(BLACK, RED);
        M5.Lcd.print("\n calibrate ");
      } else {
        M5.Lcd.setTextColor(TFT_GREENYELLOW, BLACK);
        M5.Lcd.printf("\n Y:");
        if (lastEvent == 12) M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.printf("%2d", event[12]);
        M5.Lcd.setTextColor(ORANGE, BLACK);
        M5.Lcd.printf(" N:");
        if (lastEvent == 13) M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.printf("%2d ", event[13]);
      }
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.printf(" %3d%%", percentagePin36);   // yes
      M5.Lcd.printf(" %3d%% ", percentagePin26);  // no
      M5.Lcd.setTextColor(MAGENTA, BLACK);
      M5.Lcd.printf("\n %4d", pulsesPerSecondPin36);
      M5.Lcd.printf(" %4d ", pulsesPerSecondPin26);
    } else if (scaleOn) {
      if (!scalib) {
        M5.Lcd.setTextColor(BLACK, RED);
        M5.Lcd.print("\n calibrate ");
      } else {
        M5.Lcd.setTextColor(M5.Lcd.color565(0, 255, 128), BLACK);
        M5.Lcd.print("\nnow");
        if (weight >= 0) M5.Lcd.print(" ");
        if ((weight >= 10) || (weight <= -10)) {
          M5.Lcd.printf("%.4f", weight);
        } else {
          M5.Lcd.printf("%.5f", weight);
        }
      }
      M5.Lcd.setTextColor(TFT_GREENYELLOW, BLACK);
      M5.Lcd.print("hi");
      if (maxWeight >= 0) M5.Lcd.print(" ");
      if ((maxWeight >= 10) || (maxWeight <= -10)) {
        M5.Lcd.printf("%.1f", maxWeight);
      } else {
        M5.Lcd.printf("%.2f", maxWeight);
      }
      M5.Lcd.print(" Y");
      if (lastEvent == 7) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%2d", event[10]);
      M5.Lcd.setTextColor(ORANGE, BLACK);
      M5.Lcd.print("lo");
      if (minWeight >= 0) M5.Lcd.print(" ");
      if ((minWeight >= 10) || (minWeight <= -10)) {
        M5.Lcd.printf("%.1f", minWeight);
      } else {
        M5.Lcd.printf("%.2f", minWeight);
      }
      M5.Lcd.print(" N");
      if (lastEvent == 8) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%2d", event[11]);
    } else if (tempOn || temp2On) {
      tDisp();
    } else {
      M5.Lcd.print("                                 ");
    }
    if ((scaleOn && temp2On) || (temp2On && remOn)) {
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.printf("\n");
      if (lastEvent == 4) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%1d", event[4]);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.printf("-");
      if (lastEvent == 7) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%1d ", event[7]);
      M5.Lcd.setTextColor(CYAN, BLACK);
      if (lastEvent == 5) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%1d", event[5]);
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.printf("-");
      if (lastEvent == 8) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%1d", event[8]);
      M5.Lcd.setTextColor(MAGENTA, BLACK);
      M5.Lcd.printf(" ");
      if (lastEvent == 6) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%1d", event[6]);
      M5.Lcd.setTextColor(MAGENTA, BLACK);
      M5.Lcd.printf("-");
      if (lastEvent == 9) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%1d", event[9]);
    } else {
      M5.Lcd.setTextColor(RED, BLACK);
      M5.Lcd.printf("\nx");
      if (lastEvent == 1) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%2d", event[1]);
      M5.Lcd.setTextColor(GREEN, BLACK);
      M5.Lcd.printf(" y");
      if (lastEvent == 2) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%2d", event[2]);
      M5.Lcd.setTextColor(BLUE, BLACK);
      M5.Lcd.printf(" z");
      if (lastEvent == 3) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%2d", event[3]);
    }

    if (tempOn || temp2On) {  // tiny temp
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(36, 16);
      M5.Lcd.setTextColor(YELLOW, BLACK);
      M5.Lcd.printf("t:");
      if ((lastEvent == 4) || (lastEvent == 7)) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%.2fC %.2fF ", temp - tcal, ((temp - tcal) * 1.8) + 32);
      M5.Lcd.setCursor(36, 24);
      M5.Lcd.setTextColor(CYAN, BLACK);
      M5.Lcd.printf("h:");
      if ((lastEvent == 5) || (lastEvent == 8)) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%.2f", humi);
      M5.Lcd.print("%");
      M5.Lcd.setTextColor(MAGENTA, BLACK);
      //pres = pres +10000; //test display
      if (pres < 100000) M5.Lcd.print(" ");
      M5.Lcd.printf("p:");
      if ((lastEvent == 6) || (lastEvent == 9)) M5.Lcd.setTextColor(WHITE, BLACK);
      M5.Lcd.printf("%5.0f", pres);
    }
    if ((scaleOn) && (!scalib)) {
      calibScale();
      scalib = true;
    }
  }

  //// HISTOGRAM

  int s = 20;
  int t = 120;

  M5.Lcd.drawLine(tick + 1, t - s - 4, tick + 1, M5.Lcd.height(), WHITE);
  M5.Lcd.drawLine(tick, t - s - 4, tick, M5.Lcd.height(), BLACK);
  t = t - 10;
  if ((remOn) || (scaleOn)) {
    if (tick % 4 == 0) {
      M5.Lcd.drawPixel(tick, t, TFT_LIGHTGREY);
      M5.Lcd.drawPixel(tick, t + (s * 2), TFT_LIGHTGREY);
    }
  }
  if (remOn) {
    if (calibRem) {
      M5.Lcd.drawLine(tick, t + s, tick, t + s - 30 + (percentagePin36 * 30) / 100, TFT_GREENYELLOW);
      M5.Lcd.drawLine(tick, t + s, tick, t + s + 30 - (percentagePin26 * 30) / 100, ORANGE);
    } else {
      M5.Lcd.drawLine(tick, t + s, tick, t + s - 30 + (percentagePin36 * 30) / 100, TFT_LIGHTGREY);
      M5.Lcd.drawLine(tick, t + s, tick, t + s + 30 - (percentagePin26 * 30) / 100, TFT_DARKGREY);
    }
    t = t + 70;
  } else if (scaleOn) {
    int sc = weight * 100;
    if (sc < 0) {
      M5.Lcd.drawLine(tick, t + s, tick, t + s + sc - (sc * 2), ORANGE);
    } else {
      M5.Lcd.drawLine(tick, t + s, tick, t + s + sc - (sc * 2), TFT_GREENYELLOW);
    }
    t = t + 70;
  }
  if (tempOn || temp2On) {
    M5.Lcd.drawLine(tick, t, tick, t + (oldt - temp) * 100, YELLOW);
    M5.Lcd.drawLine(tick, t + s, tick, t + s + (oldh - humi) * 10, CYAN);
    M5.Lcd.drawLine(tick, t + (s * 2), tick, t + (s * 2) + (oldp - pres), MAGENTA);
    t = t + 70;
  }
  if (!(scaleOn && tempOn) && !(tempOn && remOn)) {
    M5.Lcd.drawLine(tick, t, tick, t + (accX - oldX) * 100, RED);
    M5.Lcd.drawLine(tick, t + s, tick, t + s + (oldY - accY) * 100, GREEN);
    M5.Lcd.drawLine(tick, t + (s * 2), tick, t + (s * 2) + (oldZ - accZ) * 100, BLUE);
  }
  //upper right blank area fix
  if (tick >= M5.Lcd.width() - 3) M5.Lcd.drawLine(tick, 0, tick, 96, BLACK);

  tick++;
  if (tick > 134) {
    tick = 0;
    if (scaleOn) scalib = false;
  }
}

void btGo() {
  if (btON && deviceConnected) {
    String data;
    switch (lastEvent) {
      case 1:
        data += "X";
        break;
      case 2:
        data += "Y";
        break;
      case 3:
        data += "Z";
        break;
      case 4:
      case 7:
        data += "T";
        break;
      case 5:
      case 8:
        data += "H";
        break;
      case 6:
      case 9:
        data += "P";
        break;
      case 10:
      case 12:
        data += "Yes";
        break;
      case 11:
      case 13:
        data += "No ";
        break;
    }
    if (lastEvent >= 1 && lastEvent < 10) {
      if (pol) {
        data += "+ ";
      } else {
        data += "- ";
      }
    }
    int sensi = sens * 1000 / 5 - 3;
    data += String(id) + ",e" + String(event[0]);
    if (tempOn) {
      int tsensi = tsens * 1000 / 5 - 4;
      int pr = pres;
      data += ", t+" + String(event[4]) + ",t-" + String(event[7]) + ",t:" + String(temp - tcal) + ", h+" + String(event[5]) + ",h-" + String(event[8]) + ",h:" + String(humi) + ", p+" + String(event[6]) + ",p-" + String(event[9]) + ",p:" + String(pr) + ",ts" + String(tsensi);
    }
    if (scaleOn) {
      int ssensi = ssens * 100;
      data += ", Now" + String(abs(weight)) + ",Y" + String(event[10]) + ",hi" + String(maxWeight) + ",N" + String(event[11]) + ",lo" + String(minWeight) + ",ss" + String(ssensi);
    }
    if (remOn) {
      data += ", Now" + String(ysn) + "%,Y" + String(event[12]) + "," + String(percentagePin36) + "%," + String(pulsesPerSecondPin36) + "Hz,N" + String(event[13]) + "," + String(percentagePin26) + "%," + String(pulsesPerSecondPin26) + "Hz,rs" + String(rsens);
    }

    data += ", x" + String(event[1]) + ",y" + String(event[2]) + ",z" + String(event[3]) + ",as" + String(sensi);
    data += ", br" + String(bright);
    data += ",mh" + String(cpu);
    data += ",sl" + String(sleepr);
    data += ",vo" + String(oct);

    int bat = getBatteryLevel();
    data += ",bat:" + String(bat);

    pCharacteristicData->setValue(data.c_str());
    pCharacteristicData->notify();

    Serial.println(data);
  }
}
