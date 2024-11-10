// Libraries
#include <DHT_U.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <BluetoothSerial.h>
#include <Arduino.h>
#include <Battery18650Stats.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <TimeLib.h>
#include "ArduinoJson.h"
#include <PubSubClient.h>

// SIM card PIN (leave empty, if not defined)
//char simPIN[]  = "";
// APN data
const char GPRS_APN[] = "net2.vodafone.pt";
const char GPRS_USER[] = "vodafone";
const char GPRS_PASSWORD[] = "vodafone";

// Change the variable to your Raspberry Pi IP address, so it connects to your MQTT broker
const char *mqtt_server = "xenomorfo.ddns.net";
const char *mqttUser = "BaByGuArD";
const char *mqttPass = "BaByGuArD";

// Your phone number to send SMS: + (plus sign) and country code, for Portugal +351, followed by phone number
// SMS_TARGET Example for Portugal +351XXXXXXXXX
//#define SMS_TARGET "+351912362046"

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800    // Modem is SIM800
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb

// TTGO T-Call pins
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22

#include <Wire.h>
#include <TinyGsmClient.h>

// Set serial for debug console (to Serial Monitor, default speed 115200)
//#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define IP5306_ADDR 0x75
#define IP5306_REG_SYS_CTL0 0x00

bool setPowerBoostKeepOn(int en) {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) {
    Wire.write(0x37);  // Set bit1: 1 enable 0 disable boost keep on
  } else {
    Wire.write(0x35);  // 0x37 is default reg value
  }
  return Wire.endTransmission() == 0;
}


#define ADC_PIN 35

Battery18650Stats battery(ADC_PIN);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

static const int RXPin = 32, TXPin = 32;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

#define PressurePIN 25  // Digital pin connected to pressure switch
#define ReelSwitch 14   // Magnetic sensor pin
#define DHTPIN 33       // Digital pin connected to the DHT sensor
#define PinBatt 35      // Battery Measurement
#define Car 15          // Car door status

// Uncomment the type of sensor in use:
#define DHTTYPE DHT11  // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

RTC_DATA_ATTR bool isAuthorized = false;

bool isRejected, isUnlocked, isOk, isMsgSent = false;

int count, seat, belt, doors, temperature, humidity, blue = 0;

int old_belt, old_doors, old_temp, old_hum = 0;

double timestamp, old_timestamp, lati, longi = 0.0;

String serial = "A1B2C3D4E5";

String message, simPIN = "";

String contacts[3];

TinyGsmClient client(modem);
PubSubClient mqtt(client);

void setup() {
  //Burnout disable
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // Keep power when running from battery
  isOk = setPowerBoostKeepOn(1);
  //Serials initialization
  Serial.begin(115200);
  ss.begin(GPSBaud);
  // Initialize device.
  dht.begin();

  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay * 2 / 1000;
  //pins definition
  pinMode(ReelSwitch, OUTPUT);
  pinMode(Car, OUTPUT);
  pinMode(PressurePIN, OUTPUT);

  //Serial Begin
  SerialBT.begin("BabyGuard");  //Bluetooth device name
  if (!isAuthorized) Serial.println("The device started, now you can pair it with bluetooth!");

  // MQTT Broker setup
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(mqttCallback);
  
}

void loop() {
 
  if (!isAuthorized) blue_auth();
  else {
    mqtt.loop();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_25, 1);
    seat = digitalRead(PressurePIN);
    doors = analogRead(Car);
    belt = analogRead(ReelSwitch);
    smartDelay(5000);
    satellite();
    temp_and_hum();
    press_seat_doors();
    batteryVolts();
    gprs_connection();
    if (old_belt != belt || old_doors != doors || old_temp != temperature || old_hum != humidity || timestamp - old_timestamp >= 300) mqttPublish(); 
    if(seat && belt && doors && temperature > 35 && contacts[1].length() == 9 && old_timestamp < timestamp - 120){
      modem.gprsDisconnect();
      smartDelay(3000);
      for (int i = 1; i < 4; i++){
        if (lati != 0.1 && longi != 0.1)
          sms_send("ATENCAO!!! CRIANCA NA VIATURA!!!\nCoordenadas - Latitude: "+String(lati,5)+" Longitude: "+String(longi,5),contacts[i]);
        else sms_send("ATENCAO!!! CRIANCA NA VIATURA!!! ATENCAO!!!\nSem Localizacao disponivel - contactar: "+contacts[1],contacts[i]);
      }
      old_timestamp = timestamp;
      gprs_connection();
    }
    if (!seat && isMsgSent) {
      old_timestamp = timestamp - 1000;
      gprs_connection();
      smartDelay(15000);
      isMsgSent = false;
      Serial.println("Vou Dormir!!!");
      SerialBT.print("sleep");
      blue = 0;
      mqttPublish();
      smartDelay(15000);
      esp_deep_sleep_start();
    }
     
  }
}

// MQTT Publish
void mqttPublish() {
  StaticJsonDocument<256> doc;
  char buffer[256];
    if (!mqtt.connected())
      mqttConnect();
    //else {
    doc["temperature"] = temperature;
    doc["humidity"] = humidity;
    doc["seat"] = seat;
    doc["belt"] = belt;
    doc["car"] = doors;
    doc["lat"] = lati;
    doc["long"] = longi;
    doc["blue"] = blue;
    doc["timestamp"] = (timestamp - 3600) * 1000;
    old_timestamp = timestamp;
    serializeJson(doc, buffer);
    mqtt.publish("A1B2C3D4E5/blackbox", buffer);
    if(!seat) isMsgSent = true;
    old_doors = doors;
    old_belt= belt;
    old_temp = temperature;
    old_hum = humidity;
}

//MQTT callback

void mqttCallback(char *topic, byte *message, unsigned int len) {

  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  String messageTemp;
  int count = 0;
  for (int i = 0; i < len; i++) {
    
    if((char)message[i]=='[' or (char)message[i]==']' or (char)message[i]==',' ) {
      contacts[count] = messageTemp;
      messageTemp = "";
      count++;
    }
    else if ((char)message[i] != ' ') messageTemp += (char)message[i];
  }
}

//MQTT connection

boolean mqttConnect() {

  Serial.print("Connecting to ");
  Serial.print(mqtt_server);
  smartDelay(5000);

  // Connect to MQTT Broker without username and password
  //boolean status = mqtt.connect("GsmClientN");

  // Or, if you want to authenticate MQTT:
  boolean status = mqtt.connect(mqtt_server, mqttUser, mqttPass);
  
  if (status == false) {
    Serial.println(" server is down!");
    SerialBT.println("Server is down!");
    ESP.restart();
    return false;
  }
  Serial.println(" success");
  mqtt.subscribe("A1B2C3D4E5/server");
  return mqtt.connected();
}


// GPRS Check
void gprs_connection() {

  bool isConnected;
  isConnected = modem.isGprsConnected();
  if (!isConnected) {
    modem_activate();
    Serial.println("Connecting...");
    modem.gprsConnect(GPRS_APN, GPRS_USER, GPRS_PASSWORD);
  } else {
    delay(5000);
    Serial.print("Network IP ");
    Serial.print(": ");
    Serial.println(modem.localIP());
    setClock();
    timestamp = now();
    Serial.print("timestamp: ");
    Serial.println(timestamp);
    Serial.print("old_timestamp: ");
    Serial.println(old_timestamp);
  }
}

// Modem initialize
void modem_activate() {

  // modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Initializing modem...");
  modem.init();

  // use modem.init() if you don't need the complete restart
  if (modem.getRegistrationStatus() == 2 || modem.getSimStatus() == 3) Serial.println("Modem online!!");
  else Serial.println("Modem offline!!");
}

// Modem deactivate
void modem_deactivate() {

  // modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, LOW);
  digitalWrite(MODEM_POWER_ON, LOW);

  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  Serial.println("Powering off modem...");
  modem.radioOff();
  // use modem.init() if you don't need the complete restart
  if (modem.getRegistrationStatus() == 2 || modem.getSimStatus() == 3) Serial.println("Modem online!!");
  else Serial.println("Modem offline!!");
}

// SMS sender

void sms_send(String smsMessage, String sms_target) {

  if(sms_target.length()==9){
    
    modem.sendSMS(sms_target, smsMessage);
    Serial.print("Number: ");
    Serial.print(sms_target);
    Serial.println(" SMS sent!!");
  }
  else Serial.println("SMS failed to send");
}

// Connection to Bluetooth and messages TX/RX
void blue_auth() {

  while (SerialBT.available()) {
    message += (char)SerialBT.read();
  }
  Serial.println(message);
  if (message.substring(0, 10) == serial) {
    isAuthorized = true;
    blue = 1;
    Serial.println("Autorizado");
    SerialBT.println("Authorized");
    // Keep power when running from battery
    isOk = setPowerBoostKeepOn(1);
    modem_activate();
  }
  if (message.substring(0, 10) != "" && count <= 3 && !isAuthorized) {
    count = count + 1;
    Serial.println("Não Autorizado");
    SerialBT.print("Not Authorized");
    SerialBT.print("Attempt number: ");
    SerialBT.println(count);
    message = "";
  }
  if (count > 3 && !isRejected) {
    isRejected = true;
    Serial.println("Por favor faca reset ao equipamento");
    SerialBT.println("\nPlease reset equipment");
    smartDelay(1000);
    SerialBT.end();
    ESP.restart();
  }
}

// GPS/GNSS register and data
void satellite() {
  gps.encode(ss.read());
  Serial.print("Latitude= ");
  lati = gps.location.lat();
  if (lati == 0) lati = 0.1;
  printFloat(lati, gps.location.isValid(), 11, 6);
  Serial.print("Longitude= ");
  longi = gps.location.lng();
  if (longi == 0) longi = 0.1;
  printFloat(longi, gps.location.isValid(), 12, 6);
  SerialBT.print("Latitude: ");
  SerialBT.print(gps.location.lat());
  SerialBT.print(" Longitude: ");
  SerialBT.println(gps.location.lng());
  delay(delayMS);
  Serial.println();
}

// Temperature and Humidity Sensor
void temp_and_hum() {
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  temperature = event.temperature;
  if (temperature > 60) temperature = 0;
  Serial.print(F("Temperature: "));
  Serial.print(event.temperature);
  Serial.print(F("°C  "));
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  humidity = event.relative_humidity;
  if (humidity > 100) humidity = 0;
  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%   "));

  dht.temperature().getEvent(&event);
  SerialBT.print(F("Temperature: "));
  SerialBT.print(event.temperature);
  SerialBT.print(F("°C  "));

  dht.humidity().getEvent(&event);
  SerialBT.print(F("Humidity: "));
  SerialBT.print(event.relative_humidity);
  SerialBT.println(F("%  "));
}



// Sensors pressure, seat and doors
void press_seat_doors() {
  Serial.print("Magnet: ");
  SerialBT.print("Magnet: ");
  if (belt == 4095) {
    Serial.print("Closed   ");
    SerialBT.print("Closed   ");
    belt = 1;
  } else {
    belt = 0;
    Serial.print("Open   ");
    SerialBT.print("Open   ");
  }
  Serial.print("Pressure: ");
  SerialBT.print("Pressure: ");
  if (seat == 1) {
    Serial.print("Present ");
    SerialBT.print("Present ");
  } else {
    Serial.print("Absent ");
    SerialBT.print("Absent ");
  }
  Serial.print("Car Door: ");
  SerialBT.print("Car Door: ");
  if (doors == 4095) {
    Serial.print("Closed ");
    SerialBT.print("Closed ");
    doors = 1;
  } else {
    doors = 0;
    Serial.print("Open ");
    SerialBT.print("Open ");
  }
  Serial.print("Bluetooth: ");
  SerialBT.print("Bluetooth: ");
  if (blue == 1) {
    Serial.print("Connected ");
    SerialBT.print("Connected ");
  } else {
    Serial.print("Disconnected ");
    SerialBT.print("Disconnected ");
  }
}


// Battery Voltage
void batteryVolts() {
  Serial.print("Volts: ");
  Serial.println(battery.getBatteryVolts());

  Serial.print("Charge level: ");
  Serial.println(battery.getBatteryChargeLevel());

  Serial.print("Charge level (using the reference table): ");
  Serial.println(battery.getBatteryChargeLevel(true));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

// Print GPS values ////
static void printFloat(float val, bool valid, int len, int prec) {


  if (!valid) {
    while (len-- > 1)
      Serial.print("*");
    Serial.print(' ');
  } else {

    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3
                           : vi >= 10  ? 2
                                       : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

// Adjust internal clock
void setClock() {

  TinyGSMDateTimeFormat format;
  String clock = modem.getGSMDateTime(format);
  Serial.println(clock);
  int year = atoi(clock.substring(0, 2).c_str());
  int month = atoi(clock.substring(5, 3).c_str());
  int day = atoi(clock.substring(8, 6).c_str());
  int hour = atoi(clock.substring(11, 9).c_str());
  int min = atoi(clock.substring(14, 12).c_str());
  int sec = atoi(clock.substring(17, 15).c_str());
  // hour, min, sec, day, month, year
  setTime(hour, min, sec, day, month, year);
}
