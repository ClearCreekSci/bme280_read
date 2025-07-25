# Copyright (c) 2025 Clear Creek Scientific
#
# Based on adafruit's circuit python-bme280 code (pip install adafruit-circuitpython-bme280)
#
# The MIT License (MIT)
#
# Copyright (c) 2017 ladyada for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#         

import struct
import traceback
from time import sleep
from smbus2 import SMBus


REG_DIG_T1       = 0x88
REG_DIG_H1       = 0xa1
REG_DIG_H2       = 0xe1
REG_CHIP_ID      = 0xd0
REG_SOFTRESET    = 0xe0
REG_CTRL_HUM     = 0xf2
REG_STATUS       = 0xf3
REG_CTRL_MEAS    = 0xf4
REG_CONFIG       = 0xf5
REG_PRESS_DATA   = 0xf7
REG_TEMP_DATA    = 0xfa
REG_HUMID_DATA   = 0xfd

EXPECTED_CHIP_ID = 0x60

OVERSCAN_X1      = 0x01
OVERSCAN_X16     = 0x05

MODE_SLEEP       = 0x00
MODE_FORCE       = 0x01
MODE_NORMAL      = 0x03
VALID_MODES      = (MODE_SLEEP,MODE_FORCE,MODE_NORMAL)

# standby time constant values
# TC_X[_Y] where x is milliseconds and y is tenths of milliseconds
STANDBY_TC_125   = 0x02      # 125 milliseconds

# TODO: Figure out if and when this is useful
IIR_FILTER_DISABLE = 0x00

class InvalidIDException(Exception):
    pass

class Bme280(object):

    def __init__(self,addr=0x77):
        self.bus = SMBus(1)
        self.chip_id = self.bus.read_byte_data(addr,REG_CHIP_ID)
        if self.chip_id != EXPECTED_CHIP_ID:
            raise InvalidIDException('Expected id: ' + hex(EXPECTED_CHIP_ID) + ', actual id: ' + hex(self.chip_id))
        self.addr = addr
        self.overscan_humidity = OVERSCAN_X1
        self.overscan_pressure = OVERSCAN_X16
        self.overscan_temperature = OVERSCAN_X1
        self.mode = MODE_SLEEP
        self.standby = STANDBY_TC_125
        self.iir_filter = IIR_FILTER_DISABLE
        self.reset()
        self.read_calibration()
        self.setup_control_registers()
        self.write_config()
        self.last_humidity = 0.0
        self.last_pressure = 0.0
        self.last_temperature = 0.0

    def write_byte(self,reg,val):
        return self.bus.write_byte_data(self.addr,reg,val)

    def read_byte(self,reg):
        return self.bus.read_byte_data(self.addr,reg)

    def read_register(self,reg,size):
        reg = reg & 0xff
        self.write_byte(self.addr,reg)
        rv = bytearray(size)
        return self.bus.read_i2c_block_data(self.addr,reg,size)

    def read_3byte_float(self,reg):
        rv = 0.0
        for b in self.read_register(reg,3):
            rv *= 256.0
            rv += float(b & 0xff)
        return rv

    def set_ctrl_meas(self):
        cm = self.overscan_temperature << 5
        cm += self.overscan_pressure << 2
        cm += self.mode
        self.ctrl_meas = cm

    def set_mode(self,val):
        if not val in VALID_MODES:
            raise ValueError("mode '%s' not supported" % (val))
        self.mode = val
        self.setup_control_registers()

    def set_config(self):
        config = 0
        if self.mode == MODE_NORMAL:
            config += self.standby << 5
        if self.iir_filter:
            config += self.iir_filter << 2
        self.config = config

    def setup_control_registers(self):
        """
        Write the values to the ctrl_hum and ctrl_meas registers in the device.
        ctrl_hum sets the humidity oversampling and must be written first
        ctrl_meas sets the pressure and temperature data acquisition options
        """
        self.write_byte(REG_CTRL_HUM,self.overscan_humidity)
        self.set_ctrl_meas()
        self.write_byte(REG_CTRL_MEAS,self.ctrl_meas)

    def write_config(self):
        normal_flag = False
        if MODE_NORMAL == self.mode:
            # Writes to the config register may be ignored while in Normal mode
            normal_flag = True
            self.set_mode(MODE_SLEEP)
        self.set_config()
        self.write_byte(REG_CONFIG,self.config)
        if normal_flag:
            self.mode = NORMAL_MODE


    def reset(self):
        self.write_byte(REG_SOFTRESET,0x86)
        # Datasheet says to sleep for 2ms. We're on the cautious side
        sleep(0.004)

    def get_status(self):
        return self.read_byte(REG_STATUS)

    def read_calibration(self):
        coeff = self.read_register(REG_DIG_T1,24)
        coeff = list(struct.unpack('<HhhHhhhhhhhh',bytes(coeff)))
        coeff = [float(i) for i in coeff]
        self.temperature_calib = coeff[:3]
        self.pressure_calib = coeff[3:]
        self.humidity_calib = [0] * 6
        self.humidity_calib[0] = self.read_byte(REG_DIG_H1)
        coeff = self.read_register(REG_DIG_H2,7)
        coeff = list(struct.unpack('<hBbBbb',bytes(coeff)))
        self.humidity_calib[1] = float(coeff[0])
        self.humidity_calib[2] = float(coeff[1])
        self.humidity_calib[3] = float((coeff[2] << 4) | (coeff[3] & 0xf))
        self.humidity_calib[4] = float((coeff[4] << 4) | (coeff[3] >> 4))
        self.humidity_calib[5] = float(coeff[5])

    def print_calibration(self):
        print('Temperature Coefficients:')
        print('\t' + str(self.temperature_calib[0]))
        print('\t' + str(self.temperature_calib[1]))
        print('\t' + str(self.temperature_calib[2]))
        print('Pressure Coefficients:')
        print('\t' + str(self.pressure_calib[0]))
        print('\t' + str(self.pressure_calib[1]))
        print('\t' + str(self.pressure_calib[2]))
        print('\t' + str(self.pressure_calib[3]))
        print('\t' + str(self.pressure_calib[4]))
        print('Humidity Coefficients:')
        print('\t' + str(self.humidity_calib[0]))
        print('\t' + str(self.humidity_calib[1]))
        print('\t' + str(self.humidity_calib[2]))
        print('\t' + str(self.humidity_calib[3]))
        print('\t' + str(self.humidity_calib[4]))
        print('\t' + str(self.humidity_calib[5]))

    # Returns ambient temperature in centigrade
    def get_temperature(self):
        self.read_temperature()
        return self.last_temperature / 5120.0

    def read_temperature(self):
        if self.mode != MODE_NORMAL:
            self.set_mode(MODE_FORCE)
            while self.get_status() & 0x08:
                sleep(0.002)
        raw_temp = self.read_3byte_float(REG_TEMP_DATA) / 16  # lowest four bits are dropped
        v1 = (raw_temp / 16384.0 - self.temperature_calib[0] / 1024.0) * self.temperature_calib[1]
        x = raw_temp/131072.0 - self.temperature_calib[0] / 8192.0
        v2 = x * x * self.temperature_calib[2]
        self.last_temperature = int(v1 + v2)

    # Returns ambient relative humidity (%)
    def get_humidity(self):
        self.read_humidity()
        return self.last_humidity
 
    def read_humidity(self):
        self.read_temperature()
        hum = self.read_register(REG_HUMID_DATA,2)
        adc = float(hum[0] << 8 | hum[1])

        # Algorithm from the BME280 driver
        # https://github.com/BoschSensortec/BME280_driver/blob/msater/bme280.c
        v1 = float(self.last_temperature) - 76800.0
        v2 = self.humidity_calib[3] * 64.0 + (self.humidity_calib[4] / 16384.0) * v1
        v3 = adc - v2
        v4 = self.humidity_calib[1] / 65536.0
        v5 = 1.0 + (self.humidity_calib[2] / 67108864.0) * v1
        v6 = 1.0 + (self.humidity_calib[5] / 67108864.0) * v1 * v5
        v6 = v3 * v4 * v5 * v6
        h = v6 * (1.0 - self.humidity_calib[0] * v6 / 524288.0)
        if h > 100.0:
            h = 100.0
        if h < 0.0:
            h = 0.0
        self.last_humidity = h

    # Returns the compensated pressure in hectoPascals
    def get_pressure(self):
        self.read_pressure()
        return self.last_pressure

    def read_pressure(self):
        self.read_temperature()
        
        # Algorithm from the BME280 driver
        # https://github.com/BoschSensortec/BME280_driver/blob/msater/bme280.c
        adc = self.read_3byte_float(REG_PRESS_DATA) / 16     # drop lowest four bits
        v1 = float(self.last_temperature) / 2.0 - 64000.0
        v2 = v1 * v1 * self.pressure_calib[5] / 32768.0
        v2 = v2 + v1 * self.pressure_calib[4] * 2.0
        v2 = v2 / 4.0 + self.pressure_calib[3] * 65536.0
        v3 = self.pressure_calib[2] * v1 * v1 / 524288.0
        v1 = (v3 + self.pressure_calib[1] * v1)  / 524288.0
        v1 = (1.0 + v1 / 32768.0) * self.pressure_calib[0]
        # Avoid divide by zero error...
        if 0.0 == v1:
            raise ArithmeticError('Invalid result possibly related to error while reading the calibration registers')
        p = 1048576.0 - adc
        p = ((p - v2 / 4096.0) * 6250.0) / v1
        v1 = self.pressure_calib[8] * p * p / 2147483648.0
        v2 = p * self.pressure_calib[7] / 32768.0
        p = p + (v1 + v2 + self.pressure_calib[6]) / 16.0
        self.last_pressure = p / 100.0


if '__main__' == __name__:
    try:
        sensor = Bme280()
        print('BME 280 ready...')
        tc = sensor.get_temperature()
        tf = tc * 1.8 + 32 
        print('Temperature: ' + '{:.{}f}'.format(tc,2) + ' C (' + '{:.{}f}'.format(tf,2) + ' F)')

        h = sensor.get_humidity()
        print('Humidity: ' + '{:.{}f}'.format(h,2) + '%')

        hp = sensor.get_pressure()
        mmhg = hp * 0.75006
        inhg = hp * 0.0295
        print('Pressure: ' + '{:.{}f}'.format(mmhg,2) + ' mm hg, ' + '{:.{}f}'.format(inhg,2) + ' inches hg')

    except Exception as ex:
        print('Exception: ' + str(ex))
        print(traceback.format_exc())
