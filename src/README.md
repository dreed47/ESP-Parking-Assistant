### If you just want to install the firmware on your ESP8266, you do not need to download the source code!
Just download the latest **ParkingAsst_vx.xx_ESP8266.bin** file from the [Releases page](https://github.com/Resinchem/ESP-Parking-Assistant/releases) and follow the installation and instructions that are found in the [wiki](https://github.com/Resinchem/ESP-Parking-Assistant/wiki).  Please try the steps in the wiki before posting questions about how to install the firmware.

## Source Code
This directory contains the source code .ino sketch for the parking assistant.  You can download and modify this to meet your own needs.  The Parking Assistant supports OTA updates from the Arduino IDE (or other appropriate IDEs).  See the wiki section on Performing Updates and Upgrades for more information on OTA updates using an IDE.

You will need to have the ESP8266 boards installed in your environment, plus all the appropriate libraries listed at the top of the sketch file.  

## Building this fork (ESP8266, v0.46 fixes)
This fork patches v0.46 for ESP8266 controllers on unreliable WiFi (non-blocking MQTT reconnect, WiFi watchdog, longer web page send timeout). Build it with:

- **ESP8266 Arduino core 2.7.4** (the official v0.46 release was built on it). Core 3.x builds served the web pages much less reliably on a lossy link.
- Board: LOLIN(WEMOS) D1 R2 & mini, flash layout **4MB (FS:2MB OTA:~1019KB)**. A different filesystem size wipes the saved settings on an OTA update.

```
arduino-cli compile -b esp8266:esp8266:d1_mini:eesz=4M2M src/parking_assistant
```
(the sketch must sit in a folder named `parking_assistant`). Libraries used: WiFiManager 2.0.17, FastLED 3.6.0, ArduinoJson 6.21.5, PubSubClient 2.8, TFMPlus 1.5.0.
