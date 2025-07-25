# bme280_read
This repository provides a small Python script to read temperature, humidity and barometric pressure over I2C from a connected [Adafruit BME280 module](https://www.adafruit.com/product/2652). It requires the [smbus2](https://pypi.org/project/smbus2) Python module which can be installed from PyPI. 

The script is designed to run on a Raspberry Pi that has the primary I2C bus wired to the BME280.
Use this utility to verify that you have the BME 280 module wired correctly to the Raspberry Pi. If the wiring is correct, you should see output similar to the following when you run `python bme280_read.py`:

```
BME 280 ready...
Temperature: 24.62 C (76.32 F)
Humidity: 47.19%
Pressure: 742.84 mm hg, 29.22 inches hg
```
