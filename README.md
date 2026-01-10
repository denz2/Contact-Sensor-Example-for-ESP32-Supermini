# Contact-Sensor-for-ESP32-Supermini


This is cpp file for creating matter with two endpoints. I wanted to create a low water level sensor and a high water sensor. How it works when the water is low turn on water and when it fills up the other sensor stops the water.

I have done this using ESP-IDF 5.52 and ESP-Matter 1.4 branch. I am using matter over thread. 

I have put a prerequisities file that should pull up all the requirements for building. 

This is info is from espressif website.

I am using ESP32H2 device.

I will add more main_app files as I need it to be used with a battery. 

To run the prerequisities file you will likely need to do this
chmod + Prerequisities.sh
sed -i 's/\r//' ./Prerequisities.sh

