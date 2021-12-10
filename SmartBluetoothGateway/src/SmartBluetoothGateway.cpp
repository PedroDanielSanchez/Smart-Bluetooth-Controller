/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/pedrodanielsanchez/Documents/IoT/Smart-Bluetooth-Controller/SmartBluetoothGateway/src/SmartBluetoothGateway.ino"
/*
 * Project Smart Bluetooth IoT Gateway/Controller 
 * Description: Track Multiple Beacons, notify Smartphones, register
 * *            in a learning mode, publish to Adafruit, report lost beacons
 * Author: Pedro Sanchez
 * Date: Nov 29 2021
 */


#include <math.h>
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_GFX_RK.h"
#include "Adafruit_SSD1306_RK.h"
// #include "JsonParserGeneratorRK.h"
#include "IoT_Timer.h"
#include "TinyGPS++.h"
#include "credentials.h"

void setup();
void loop();
void saveGPSlocation();
void displaySetup();
void updateOLED();
int deviceIsRegistered(const char *arr, int8_t rssi_val);
void processUuid(uint8_t *myData, size_t len, int i);
void buttonHandler();
void MQTT_connect();
void keepAliveMQTT();
#line 21 "/Users/pedrodanielsanchez/Documents/IoT/Smart-Bluetooth-Controller/SmartBluetoothGateway/src/SmartBluetoothGateway.ino"
struct geo
{
  float lat;
  float lon;
};

#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET); // I2C oled display

TCPClient TheClient;

Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Publish feeds
Adafruit_MQTT_Publish ACx = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/acx");
Adafruit_MQTT_Publish mTotalBeacons = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/totalbeacons");
Adafruit_MQTT_Publish mAlarmedBeacons = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alarmedbeacons");
Adafruit_MQTT_Publish mLostBeacons = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lostbeacons");

#define DASHBOARD_TIMING 40000 // publish every 40 seconds
#define TRACK_TIMEOUT_MS 30000 // beacon max timeout alarm is 30 seconds
#define GPS_READ_NTERVAL 120000 // 2 minutes for now... //  600000 for 10 minutes intervals
#define MAX_DEVICES 6

const int OLED_addr = 0x3C;

String DateTime, TimeOnly;

IoT_Timer pingMQQT;
IoT_Timer dashboardTimer;
IoT_Timer gpsReadTimer;

struct devicesBT
{
  char addrDev[20];
  char deviceName[20];
  int8_t deviceRSSI;
  byte deviceRSSIIndex;
  u_long deviceLastTime;
  time_t deviceLastTimeStr;
  byte status;
  byte alarmStatus;
};

void scanResultCallback(const BleScanResult *scanResult, void *context);

devicesBT trackedDevices[MAX_DEVICES]; // Tracking registered devices
IoT_Timer deviceTimers[MAX_DEVICES];   // Tracking if any beacon lost contact

// For logging use LOG as serial is not thread safe
SerialLogHandler logHandler(115200, LOG_LEVEL_ERROR, {{"app", LOG_LEVEL_TRACE}});

// https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf

int totalFound = 0; // Beacons registered
int alarmedBeacons; // Beacons that have changed their RSSI
int lostBeacons;    // Beacons registered but not advertising
String devStatus;   // The payload going to the cloud
bool learningMode;  // Triggered with a button
bool lastLearningMode;

//---GPS variables ---//
TinyGPSPlus gps;
const int UTC_offset = -6;
float lat, lon, alt;
uint8_t hr, mn, se, sat, dy, mth, yr;

SYSTEM_MODE(AUTOMATIC);

void setup()
{

  // GPS  serial module   -> GND,Tx,Rx,Vcc
  // Module to Argon:  Rx => D9  Tx=> D10
  Serial1.begin(9600);

  Time.zone(-6);
  Particle.syncTime();

  learningMode = false;
  lastLearningMode = learningMode;
  delay(5000);
  pinMode(D7, OUTPUT);       // Test blue LED built-in
  pinMode(A0, INPUT_PULLUP); // Button
  attachInterrupt(A0, buttonHandler, FALLING);

  WiFi.connect();

  dashboardTimer.startTimer(DASHBOARD_TIMING);
  pingMQQT.startTimer(100);   // First time quick ping
  gpsReadTimer.startTimer(1); // 1st time read now

  // Initialize devices storage
  for (int i = 0; i < MAX_DEVICES; i++)
  {
    trackedDevices[i].status = 0;
    trackedDevices[i].alarmStatus = 0;
    trackedDevices[i].deviceLastTime = 0;
    trackedDevices[i].addrDev[0] = 0;
    trackedDevices[i].deviceRSSI = 0;
    trackedDevices[i].deviceRSSIIndex = 0;
    trackedDevices[i].deviceLastTimeStr = 0;
    strcpy(trackedDevices[i].deviceName, "");
    strcpy(trackedDevices[i].addrDev, "");
  }

  totalFound = 0;
  alarmedBeacons = 0;
  lostBeacons = 0;

  displaySetup(); // Initialize SSD1306 display
  updateOLED();
  BLE.setScanTimeout(500); // 5 secs as it's in 10ms increments
  Log.trace("Starting SMART BLUETOOTH RSSI Gateway");
}

void loop()
{

  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11, 19);

  if (learningMode != lastLearningMode)
  { // Display operating mode
    lastLearningMode = learningMode;
    updateOLED();
  }

  if (gpsReadTimer.isTimerReady())
  {
    while (Serial1.available() > 0)
    {
      if (gps.encode(Serial1.read()))
      {
        Log.info("READ GPS location");
        saveGPSlocation(); // gps location is displayed later
      }
      else
      {
        Log.info("Cannot read GPS location");
      }
    }
    gpsReadTimer.startTimer(GPS_READ_NTERVAL); // update every 10 minutes
  }

  BLE.scan(scanResultCallback, NULL);

  MQTT_connect();
  keepAliveMQTT();

  //Log.trace("%s",DateTime.c_str());

  delay(2000);
  //BLE.scan(scanResultCallback, NULL);

  //Log.trace("%s", TimeOnly.c_str());

  for (int i = 0; i < totalFound; i++)
  {
    if (deviceTimers[i].isTimerReady())
    {
      // Check for a lost device (non Advertising) !
      if ((millis() - trackedDevices[i].deviceLastTime) > TRACK_TIMEOUT_MS)
      {
        Log.trace("%s [%i-%i] LOST device  #%i : %s  !!", TimeOnly.c_str(), totalFound, strlen(trackedDevices[i].addrDev), i, trackedDevices[i].addrDev);
        if (trackedDevices[i].status == 1)
        { // If it was active report "Not Advertising"
          // TBD Publish device not advertising to update the dashboard
          lostBeacons++;
        }

        devStatus = String::format("{\"beaconNum\":\"%i\",\"address\":\"%s\",\"lastSeen\":\"%s\",\"lastRSSI\":\"%i\",\"status\":\"%s\"}",
                                   i,
                                   trackedDevices[i].addrDev,
                                   Time.format(trackedDevices[i].deviceLastTimeStr, TIME_FORMAT_DEFAULT).c_str(),
                                   trackedDevices[i].deviceRSSI, "NOT ADVERTISING (LAST VALUES)");
        // Publish the RSSI and Device Info
        Log.trace("%s", devStatus.c_str());
        Particle.publish("status", devStatus, PRIVATE, WITH_ACK);
        trackedDevices[i].status = 0; // No advertising received
      }
      deviceTimers[i].startTimer(TRACK_TIMEOUT_MS);
    }
  }

  if (dashboardTimer.isTimerReady())
  {
    // Publish to dashboard
    updateOLED();
    if (mqtt.Update())
    {

      mAlarmedBeacons.publish(alarmedBeacons);
      mTotalBeacons.publish(totalFound);
      mLostBeacons.publish(lostBeacons);

      dashboardTimer.startTimer(DASHBOARD_TIMING); // reset the publish timer
    }
  }
}

void saveGPSlocation()
{

  Log.info("GPS valid: %d", gps.location.isValid());
  Log.info("GPS age  : %lu", gps.location.age());
  lat = gps.location.lat();
  lon = gps.location.lng();
  alt = gps.altitude.meters();
  hr = gps.time.hour();
  mn = gps.time.minute();
  se = gps.time.second();
  sat = gps.satellites.value();
  dy = gps.date.day();
  mth = gps.date.month();
  yr = gps.date.year();
  if (hr > 7)
  {
    hr = hr + UTC_offset;
  }
  else
  {
    hr = hr + 24 + UTC_offset;
  }
  Log.info("Lat: %f, Long: %f, Alt: %f \n", lat, lon, alt);
}

void displaySetup()
{
  display.begin(SSD1306_SWITCHCAPVCC, OLED_addr);
  display.display();
  delay(2000);
  display.clearDisplay();

  display.drawPixel(10, 10, WHITE);
  display.display();
  delay(2000);
  display.clearDisplay();
}

void updateOLED()
{
  display.setCursor(3, 0);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.printf("Smart Bluetooth IoT\n");
  if (!learningMode)
  {
    display.printf("Gateway NORMAL\n");
  }
  else
  {
    display.printf("Gateway LEARNING *\n");
  }
  display.printf("%s\n", TimeOnly.c_str());
  display.printf("Beacons     : %i\n", totalFound);
  display.printf("In Alarm    : %i\n", alarmedBeacons);
  display.printf("Lost Signal : %i\n", alarmedBeacons);
  display.printf("Lat: %.6f\n", lat);
  display.printf("Lon: %.6f\n", lon);
  display.display();
}

int deviceIsRegistered(const char *arr, int8_t rssi_val)
{

  if (memcmp(&arr[0], "AC:23", 4) != 0)
  {            // not equal to beacons PREFIX ?
    return -2; // device qill not be tracked
  }
  // Verify if already registered
  for (int i = 0; i < totalFound; i++)
  {
    if (strcmp(trackedDevices[i].addrDev, arr) == 0)
    { // found
      deviceTimers[i].startTimer(TRACK_TIMEOUT_MS);
      // Calculate if the device is moving based on RSSI ...
      if (abs(abs(trackedDevices[i].deviceRSSI) - abs(rssi_val)) >= 10)
      { // device is moving
        if (trackedDevices[i].alarmStatus == 0)
        {
          trackedDevices[i].alarmStatus = 1;
          alarmedBeacons++;
        }
      }
      else
      {
        if (trackedDevices[i].alarmStatus == 1)
        {
          trackedDevices[i].alarmStatus = 0;
          alarmedBeacons--;
        }
      }
      trackedDevices[i].deviceRSSI = rssi_val;
      trackedDevices[i].deviceLastTime = millis();
      trackedDevices[i].deviceLastTimeStr = Time.now();
      if (trackedDevices[i].status != 1)
      {
        lostBeacons--;
        devStatus = String::format("{\"beaconNum\":\"%i\",\"address\":\"%s\",\"lastSeen\":\"%s\",\"lastRSSI\":\"%i\",\"status\":\"%s\"}",
                                   i,
                                   trackedDevices[i].addrDev,
                                   Time.format(trackedDevices[i].deviceLastTimeStr, TIME_FORMAT_DEFAULT).c_str(),
                                   trackedDevices[i].deviceRSSI, "back in service");
        Log.trace("%s", devStatus.c_str());
        Particle.publish("status", devStatus, PRIVATE, WITH_ACK);

        //Process the publish event immediately
        Particle.process();

        trackedDevices[i].status = 1;
      }
      return i;
    }
  }
  // Check if it is a new one to register
  if (learningMode && (totalFound < MAX_DEVICES))
  {
    strcpy(trackedDevices[totalFound].addrDev, arr);
    trackedDevices[totalFound].status = 1;
    deviceTimers[totalFound].startTimer(TRACK_TIMEOUT_MS);
    trackedDevices[totalFound].deviceRSSI = rssi_val;
    trackedDevices[totalFound].deviceLastTime = millis();
    trackedDevices[totalFound].deviceLastTimeStr = Time.now();

    Log.trace("%s REGISTERED NEW device  #%i : %s  !!", TimeOnly.c_str(), totalFound, trackedDevices[totalFound].addrDev);
    devStatus = String::format("{\"beaconNum\":\"%i\",\"address\":\"%s\",\"lastSeen\":\"%s\",\"lastRSSI\":\"%i\",\"status\":\"%s\"}",
                               totalFound,
                               trackedDevices[totalFound].addrDev,
                               Time.format(trackedDevices[totalFound].deviceLastTimeStr, TIME_FORMAT_DEFAULT).c_str(),
                               trackedDevices[totalFound].deviceRSSI, "new device registered");
    // Publish the RSSI and Device Info
    Log.trace("%s", devStatus.c_str());
    Particle.publish("status", devStatus, PRIVATE, WITH_ACK);
    totalFound++;
    return totalFound - 1; // posicion del timer es 1 menos
  }
  return -1; // ** Array is full   AND/OR    ** Not in learningmode
}

void scanResultCallback(const BleScanResult *scanResult, void *context)
{

  uint8_t buffData[50];
  char tmpData[100];
  size_t dataLen;
  int posi = 0;
  uint8_t devRSSI;
  BleAddress beaconAddress;

  dataLen = scanResult->advertisingData.get(buffData, sizeof(buffData));

  beaconAddress = scanResult->address;
  devRSSI = scanResult->rssi;

  posi = deviceIsRegistered(beaconAddress.toString().c_str(), devRSSI);

  if (posi > -1)
  { // found
    Log.info("%s Target device => %i - %i <= found MAC: %s  RSSI: %ddBm [ %s ]", TimeOnly.c_str(), posi, totalFound, beaconAddress.toString().c_str(), scanResult->rssi,
             scanResult->advertisingData.deviceName().c_str());
    trackedDevices[posi].status = 1;

    memset(tmpData, 0, sizeof(tmpData));
    sprintf(tmpData, " %02x%02x Version Number %02x", buffData[5], buffData[6], buffData[12]);
    Log.info("%s", tmpData);

    if (buffData[5] == 0xe1 && buffData[6] == 0xff &&
        (buffData[12] == 0x03 || buffData[12] == 0x08))
    {
      processUuid(buffData, dataLen, posi);
    }
    memset(tmpData, 0, sizeof(tmpData));

    for (size_t j = 0; j < dataLen; j++)
    {
      sprintf(&tmpData[posi], "%02x", buffData[j]);
      posi += 2;
    }
    Log.info("%s", tmpData);

    BLE.stopScanning();
  }
}

void processUuid(uint8_t *myData, size_t len, int i)
{
  float xAxis;
  float intVal = 0;
  float decVal = 0;
  char devName[50];

  // Check for ACC-Axis Data

  if (myData[12] == 0x03)
  { // Confirm it's Axis data
    // conver hex with decimal in 2's complement to decimal
    intVal = myData[14];
    decVal = myData[15] / 256.0;
    xAxis = intVal > 127 ? ((intVal - 256) + decVal) : (intVal + decVal);
    Log.info("X-Axis = %.2f", xAxis);
  }
  else if (myData[12] == 0x08)
  {
    memset(devName, 0, sizeof(devName));
    memcpy(devName, &myData[20], len - 20);
    strcpy(trackedDevices[i].deviceName, devName);
  }
}

void buttonHandler()
{
  static system_tick_t lastButtonTime = 0;

  if ((millis() - lastButtonTime) > 500)
  {
    learningMode = !learningMode;
    lastButtonTime = millis();
  }
  if (learningMode)
  {
    digitalWrite(D7, HIGH);
    //totalFound = 0;
  }
  else
  {
    digitalWrite(D7, LOW);
  }
}

void MQTT_connect()
{
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected())
  {
    return;
  }

  Log.info("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0)
  { // connect will return 0 for connected
    Log.info("%s\n", (char *)mqtt.connectErrorString(ret));
    Log.info("Retrying MQTT connection in 5 seconds..\n");
    mqtt.disconnect();
    delay(5000); // wait 5 seconds
  }
  Log.info("MQTT Connected!\n");
}

void keepAliveMQTT()
{
  // Ping MQTT Broker every 2 minutes to keep connection alive
  if (pingMQQT.isTimerReady())
  {
    Log.info("Pinging MQTT \n");
    if (!mqtt.ping())
    {
      Log.info("Disconnecting \n");
      mqtt.disconnect();
    }
    pingMQQT.startTimer(120000);
  }
}
