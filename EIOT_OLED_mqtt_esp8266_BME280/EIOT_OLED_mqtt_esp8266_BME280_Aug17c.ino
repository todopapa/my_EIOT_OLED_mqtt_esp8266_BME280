/* Sketch to collect temp, humidity, air pressure and send via MQTT to EasyIoT Cloud.
 and added display information function to small OLED via ssd1306 IIC interface.
 I've developed this program by combining the following four programs.

１、Temp sensor connected to ESP8266 and upload data using MQTT 
https://www.hackster.io/mtashiro/temp-sensor-connected-to-esp8266-and-upload-data-using-mqtt-5e05c9
（Ｉｔ is for DHT11 sensor, I'vechanged to use BME280 sensor utilizing next program)
2,ESP8266 (ESP-WROOM-02) with Temperature humidity barometric pressure sensor (BME 280) 
  has linked to Ambient IoT Cloud service. by yukitter-san
  http://qiita.com/yukitter/items/de45b8280db0f3435b4d

3, 0.96" small IIC OLED LCD Display with SSD1306. 
   じわじわ進む K.Kurahshi-san  BME280とOLEDをESP-WROOM-02で使ってみた
   https://jiwashin.blogspot.jp/2016/01/bme280-oled-and-esp-wroom-02.html

  SSD1306.h libraly is downloaded from,
   squix78 Daniel Eichhorn-san https://github.com/squix78 esp8266-oled-ssd1306 very versatile libralies and examples.
    https://github.com/squix78/esp8266-oled-ssd1306

4,ESP8266 smart plant irrigation system IOT-PLAYGROUND.COM
  https://iot-playground.com/blog/2-uncategorised/94-esp8266-smart-plant-irrigation-system 
  Caution:MQTT.h libraly shoud be downloaded from URL shown below Github link libraly folder.
  https://github.com/iot-playground/EasyIoT-Cloud

Description: I used ESP8266 (ESP-WROOM-02) pin 4 (GPIO4) as SDA, pin5(GPIO5) as SCK for IIC interface. 
  ・There might be many liabraly with same name (eg.MQTT.h) then you should download the MQTT.h from EasyIot-Cloud.
  ・I strongly addvice you to check libraly function name beforehand.
  ・The function names are correct or not with keyword file in libraly.(eg.getTemperature())
   　Actualy I troubled with old version BME280 libraly.
  ・ I am not English speaker, then some English comments are fuzzy, Please forgive there might be uncertain expression.
    Actualy I used Google translaton for some Japanese comment translation to English comment. (But proofreaded lol.)
 
 Those Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino
  MQTT code Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
*/

#include <ESP8266WiFi.h>
#include <Wire.h>
// #include <Time.h>
#include "BME280_MOD-1022.h"
#include <MQTT.h>
#include <EEPROM.h>
#include "SSD1306.h"

/* BME280 SENSOR SETUP */
#define PERIOD 10          // Display and Publish to EasyIot duration (sec)
#define DEBUG              // Enable #ifdef DEBUG

//  OLED / SSD1306 Initializing
//
SSD1306   display(0x3c, 4, 5);     // If the case indicated on PCB is 78. initialize with the I2C addr 0x3C (for the 128x64)

#define CONFIG_START 0
#define CONFIG_VERSION "v01"

struct StoreStruct {
  // This is for mere detection if they are your settings
  char version[4];
  // The variables of your settings
  uint moduleId;  // module id
} storage = {
  CONFIG_VERSION,
  // The default module 0
  0,
};

#define PARAM_TEMPERATURE  "/Sensor.Parameter1"
#define PARAM_HUMIDITY     "/Sensor.Parameter2"
#define PARAM_PRESSURE     "/Sensor.Parameter3"

/* WIFI SETUP */
#define AP_SSID     "xxxxxxxxxxxx"           // set your wireless router's AP_SSID
#define AP_PASSWORD "xxxxxxxxxxxx"            // set your wireless router's AP_PASSWORD
#define EIOTCLOUD_USERNAME "xxxxxx"            // set your EIOTCLOUD_USERNAME
#define EIOTCLOUD_PASSWORD "xxxxxx"         // set your EIOTCLOUD_PASSWORD

// create MQTT object
#define EIOT_CLOUD_ADDRESS "cloud.iot-playground.com"
#define humidity_topic  mqtt_user + "/sensor/humidity"
#define temperature_c_topic  mqtt_user + "feeds/temperature"
#define barometer_hpa_topic mqtt_user + "/feeds/barometer"

 #define MS_IN_SEC  1000 // 1S  

 MQTT myMqtt("", EIOT_CLOUD_ADDRESS, 1883);

String valueStr("");
String topic("");
boolean result;
bool stepOk = false;
unsigned long startTime;


float temperature = 0.0;
float humidity    = 0.0;
float pressure    = 0.0;
 
//float temp_f;  // Values read from sensor
//String webString="";     // String to display
// Generally, you should use "unsigned long" for variables that hold time
//const long interval = 2300;              // interval at which to read sensor
void printFormattedFloat(float val) {
  char buffer[10];

  dtostrf(val, 4, 2, buffer);
  Serial.print(buffer);
}

void setup() {
  wifi_set_sleep_type(LIGHT_SLEEP_T);

  // Starting I2C communication
  // SDA: GPIO4
  // SCL: GPIO5
  Wire.begin(4, 5);
  Serial.println("I2C start.");

  // startng Serial communication
  Serial.begin(115200);
  Serial.println("Serial start.");

  // Initializing BME280
  BME280.readCompensationParams();

  // Set the number of oversampling
  BME280.writeOversamplingTemperature(os1x);
  BME280.writeOversamplingHumidity(os1x);
  BME280.writeOversamplingPressure(os1x);
  Serial.println("BME280 start.");

  // Initialising the OLED display.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // wifi initializing..
  WiFi.begin(AP_SSID, AP_PASSWORD);
  Serial.println("WiFi start.");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(AP_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  EEPROM.begin(512);
  loadConfig();
  //set client id
  // Generate client name based on MAC address and last 8 bits of microsecond counter
  String clientName;
  //clientName += "esp8266-";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);
  myMqtt.setClientId((char*) clientName.c_str());

  Serial.print("MQTT client id:");
  Serial.println(clientName);

  // setup callbacks
  myMqtt.onConnected(myConnectedCb);
  myMqtt.onDisconnected(myDisconnectedCb);
  myMqtt.onPublished(myPublishedCb);
  //  myMqtt.onData(myDataCb);
  
  //Serial.println("connect mqtt...");
  myMqtt.setUserPwd(EIOTCLOUD_USERNAME, EIOTCLOUD_PASSWORD);  
  myMqtt.connect();

  delay(500);
  
  Serial.print("ModuleId: ");
  Serial.println(storage.moduleId);


  //create module if necessary 
  if (storage.moduleId == 0)
  {
    //create module
    Serial.println("create module: /NewModule");
    storage.moduleId = myMqtt.NewModule();

    if (storage.moduleId == 0)
    {
      Serial.println("Module NOT created. Check module limit");
      while(1)
        delay(100);
    }

   // set module type
    Serial.println("Set module type");    
    myMqtt.SetModuleType(storage.moduleId, "ZMT_BMT280");

    // create Sensor.Parameter1
    // Sensor.Parameter1 - current temperature
    Serial.println("new parameter: /"+String(storage.moduleId)+ PARAM_TEMPERATURE);    
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_TEMPERATURE);


    // set Description
    Serial.println("set description: /"+String(storage.moduleId)+ PARAM_TEMPERATURE);    
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_TEMPERATURE, "garden temperature.");

    // set Unit
    Serial.println("set Unit: /"+String(storage.moduleId)+ PARAM_TEMPERATURE);    
    myMqtt.SetParameterUnit(storage.moduleId, PARAM_TEMPERATURE, "°C");

    // set dbLogging
    Serial.println("set Unit: /"+String(storage.moduleId)+ PARAM_TEMPERATURE);    
    myMqtt.SetParameterDBLogging(storage.moduleId, PARAM_TEMPERATURE, true);

    // create Sensor.Parameter2
    // Sensor.Parameter2 - current garden humidity
    Serial.println("new parameter: /"+String(storage.moduleId)+ PARAM_HUMIDITY);    
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_HUMIDITY);

    // set Description
    Serial.println("set description: /"+String(storage.moduleId)+ PARAM_HUMIDITY);    
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_HUMIDITY, "garden humidity.");

    // set Unit
    Serial.println("set Unit: /"+String(storage.moduleId)+ PARAM_HUMIDITY);    
    myMqtt.SetParameterUnit(storage.moduleId, PARAM_HUMIDITY, "%");

    // set dbLogging
    Serial.println("set Unit: /"+String(storage.moduleId)+ PARAM_HUMIDITY);    
    myMqtt.SetParameterDBLogging(storage.moduleId, PARAM_HUMIDITY, true);

    // create Sensor.Parameter3
    // Sensor.Parameter3 - current grden air pressure
    Serial.println("new parameter: /"+String(storage.moduleId)+ PARAM_PRESSURE);    
    myMqtt.NewModuleParameter(storage.moduleId, PARAM_PRESSURE);

    // set Description
    Serial.println("set description: /"+String(storage.moduleId)+ PARAM_PRESSURE);    
    myMqtt.SetParameterDescription(storage.moduleId, PARAM_PRESSURE, "atmo.");

    // set Unit
    Serial.println("set Unit: /"+String(storage.moduleId)+ PARAM_PRESSURE);    
    myMqtt.SetParameterUnit(storage.moduleId, PARAM_PRESSURE, "hPa");

    // set dbLogging
    Serial.println("set Unit: /"+String(storage.moduleId)+ PARAM_PRESSURE);    
    myMqtt.SetParameterDBLogging(storage.moduleId, PARAM_PRESSURE, true);

    // save new module id
    saveConfig();
  } 
}

void loop() {
  // OLED buffer
    char buffer[80];

  // Set the BME 280 to forced mode to perform measurement once a time and wait until the measurement is completed
  BME280.writeMode(smForced);
  while (BME280.isMeasuring()) {
    delay(1);
  }

  // Read measured data from BME280
  BME280.readMeasurements();
  temperature = BME280.getTemperature();
  humidity = BME280.getHumidity();
  pressure = BME280.getPressure();

  // Serial Print BME280 measured value
  Serial.print("Temperature: ");
  printFormattedFloat(temperature);
  Serial.println("");

  Serial.print("Humidity: ");
  printFormattedFloat(humidity);
  Serial.println("");

  Serial.print("Pressure: ");
  printFormattedFloat(pressure);
  Serial.println("");
  Serial.println("");

  valueStr = temperature;

    topic  = "/"+String(storage.moduleId)+ PARAM_TEMPERATURE;
    result = myMqtt.publish(topic, valueStr, 0, 1);

  valueStr = humidity;

    topic  = "/"+String(storage.moduleId)+ PARAM_HUMIDITY;
    result = myMqtt.publish(topic, valueStr, 0, 1);

  valueStr = pressure;

    topic  = "/"+String(storage.moduleId)+ PARAM_PRESSURE;
    result = myMqtt.publish(topic, valueStr, 0, 1);

        //  Display data on OLED
  display.clear();

  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString( 0, 15, "Tempera");  // changed from 16
  display.drawString( 0, 31, "Humidity");  // changed from 32
  display.drawString( 0, 47, "Pressure");  // changed from 48
  display.setFont(ArialMT_Plain_16);
  display.drawString( 0, 0, "  Whether  Station");
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);

  dtostrf( temperature, 7, 2, buffer);
  display.drawString(127, 14, buffer);  // changed from 12

  dtostrf( humidity, 7, 2, buffer);   
  display.drawString(127, 30, buffer);  // changed from 28

  dtostrf( pressure, 7, 2, buffer);
  display.drawString(127, 46, buffer);  // changed from 44

  display.display();

  delay(PERIOD * 1000);
}

void loadConfig() {
  // To make sure there are settings, and they are YOURS!
  // If nothing is found it will use the default settings.
  if (EEPROM.read(CONFIG_START + 0) == CONFIG_VERSION[0] &&
      EEPROM.read(CONFIG_START + 1) == CONFIG_VERSION[1] &&
      EEPROM.read(CONFIG_START + 2) == CONFIG_VERSION[2])
    for (unsigned int t=0; t<sizeof(storage); t++)
      *((char*)&storage + t) = EEPROM.read(CONFIG_START + t);
}

void saveConfig() {
  for (unsigned int t=0; t<sizeof(storage); t++)
    EEPROM.write(CONFIG_START + t, *((char*)&storage + t));

  EEPROM.commit();
}


String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

void waitOk()
{
  while(!stepOk)
    delay(100);
 
  stepOk = false;
}

boolean IsTimeout()
{
  unsigned long now = millis();
  if (startTime <= now)
  {
    if ( (unsigned long)(now - startTime )  < MS_IN_SEC ) 
      return false;
  }
  else
  {
    if ( (unsigned long)(startTime - now) < MS_IN_SEC ) 
      return false;
  }

  return true;
}


void subscribe()
{
  if (storage.moduleId != 0)
  {
    // Sensor.Parameter1 - humidity treshold value
    myMqtt.subscribe("/"+String(storage.moduleId)+ PARAM_TEMPERATURE);
  
    // Sensor.Parameter2 - manual/auto mode 0 - manual, 1 - auto mode
    myMqtt.subscribe("/"+String(storage.moduleId)+ PARAM_HUMIDITY);
  
    // Sensor.Parameter3 - pump on/ pump off
    myMqtt.subscribe("/"+String(storage.moduleId)+ PARAM_PRESSURE);
  }
}


void myConnectedCb() {
#ifdef DEBUG
  Serial.println("connected to MQTT server");
#endif
  subscribe();
}

void myDisconnectedCb() {
#ifdef DEBUG
  Serial.println("disconnected. try to reconnect...");
#endif
  delay(500);
  myMqtt.connect();
}

void myPublishedCb() {
#ifdef DEBUG  
  Serial.println("published.");
#endif
}


