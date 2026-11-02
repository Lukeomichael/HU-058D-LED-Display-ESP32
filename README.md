# HU-058D-LED-Display-ESP32
This project uses a kit available from AliExpress for cheap. It is modified to include an ESP32 Module, turning it into a network clock that is always synced.
The HU-058D LED Display can be used standalone or with HomeAssistant to control enhanced features. The idea is that if you don't have Home Assistant, you can still have a clock that syncs to Network time. 
In addition, by pressing the Top button, you will change to Timer - This is a count up timer that can be started and stopped by pressing the bottom button. Press and hold the bottom button to reset the timer.
If you have Home assistant, you just need to set up  your MQTT Broker and connect to it as part of the initial AP configuration.

This is forked from https://github.com/misterblack1/HU-058_ESPHome.

Hardware modifications include powering the buzzer off 3V3 and using a Wemos D1 ESP32 module to minimise space.
The Access point is triggered if you keep the top button pressed for 10 seconds.
![Config Home](images/AP%20Wifi.jpeg)
![Config Home](images/Config_home.jpeg)

As I am in Australia, I have provided a drop down for Australian Timezones. You will need to make changes if you are in a different country.
![Config Home](images/Config_set.jpeg)

With the LED clock connected to MQTT, HA will discover the device and present you with additional options.
* You can turn the buzzer on and off
* You can set a value with "F" or "C" if needed and show it as required
* You can turn on/off the display LEDs
* You can change the colour of the display LEDs
* You can show a Count Up timer
* You can set the count up Timer value, and a buzzer will sound when it is reached
* You can start and stop the timer
* 
![Config Home](images/HA_MQTT_Device%20info.jpeg)
![Config Home](images/Change%20Color.jpeg)

You can set up HA Automatons to show literature every few seconds, change the brightness at dusk etc.
