The wiring is based on https://github.com/misterblack1/HU-058_ESPHome/blob/main/docs/wiring.md noting that:

* the buzzer power needs to be modified so that it is powered by 3V3 (from the ESP module) not the USB power. To do this you will need to cut the small track as shown
![Config Home](wiring1.jpeg)
  
* I have connected up the two buttons to the ESP - the code sets these as inputs with pullup
![Config Home](wiring2.jpeg)

For programming, use the USB on the ESP32, but once ready, the 5V power comes from the wires soldered from the USB port
