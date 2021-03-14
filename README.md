# TeleMitt Project Code
The TeleMitt software allows the user to connect to the TeleMitt glove and stream the captured sensor data directly to a BLE capable PC. This github contains both the python code for the applicaiton and accompanying python library, but also the embedded systems C code, and necessary libraries needed to run it. 

## TeleMitt software dependencies
The TeleMitt application is dependent on a number of libraries, the pip install commands for which can be found below:

```bash
pip install vpython
pip install bleak
pip install XlsxWriter
```
## TeleMitt embedded systems dependencies
The following URLs contain the download packages for the libraries used in this product, accompanied by a second url that takes the user to the documentation for these extra libraries.

### BLE:
Download: https://learn.adafruit.com/introducing-the-adafruit-bluefruit-le-uart-friend/software

Documentaton: https://github.com/adafruit/Adafruit_BluefruitLE_nRF51

### IMU:
Download:https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary

Documentaton:https://github.com/sparkfun/SparkFun_Qwiic_9DoF_IMU_Breakout
