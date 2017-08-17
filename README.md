# my_EIOT_OLED_mqtt_esp8266_BME280
ESP8266 sketch to collect temp, humidity, air pressure with BME280 and send via MQTT to EasyIoT Cloud.
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
