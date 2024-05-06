# ESP32-Server-Client-IMU
ESP32 MCU server and client protocol to log IMU data to train an ML model

### Project Description
Multiple ESP32 Qt Py microcontrollers are connected together through a BLE server and client protocol. The master MCU waits until the slave MCUs are all connected. Afterwards, a signal is sent to turn on the recording for all the MCUs. 

The slave MCUs are also conencted to an SD Card module and an Adafruit IMU sensor module. This project is used to log IMU sensor data to train a machine learning model to detect the stage in motion from IMU data during ankle and foot gait phase change. This project is used to allow for a more efficient and seamless testing period. This project was the successor of the Nano-Master-Slave-IMU project. I changed the MCU to the ESP32 Qt Py as it was smaller, faster and more efficient. Additionally, the connection with the phone was removed as the connection with the phone was hard to maintain during the testing process. Moreover, in this new version after the test is started all bluetooth connections are ignored to make the process more reliable in case the bluetooth connection accidentally fails.  
