# The MIT License (MIT)
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_fxos8700`
====================================================

CircuitPython module for the NXP FXOS8700 accelerometer and magnetometer.
Based on the driver from:
  https://github.com/adafruit/Adafruit_FXOS8700

See examples/simpletest.py for a demo of the usage.

* Author(s): Tony DiCola
"""
import adafruit_bus_device.i2c_device as i2c_device


# Register addresses and other constants:
_FXOS8700_ADDRESS                  = const(0x1F)   # 0011111
_FXOS8700_ID                       = const(0xC7)   # 1100 0111
_FXOS8700_REGISTER_STATUS          = const(0x00)
_FXOS8700_REGISTER_OUT_X_MSB       = const(0x01)
_FXOS8700_REGISTER_OUT_X_LSB       = const(0x02)
_FXOS8700_REGISTER_OUT_Y_MSB       = const(0x03)
_FXOS8700_REGISTER_OUT_Y_LSB       = const(0x04)
_FXOS8700_REGISTER_OUT_Z_MSB       = const(0x05)
_FXOS8700_REGISTER_OUT_Z_LSB       = const(0x06)
_FXOS8700_REGISTER_WHO_AM_I        = const(0x0D)   # 11000111   r
_FXOS8700_REGISTER_XYZ_DATA_CFG    = const(0x0E)
_FXOS8700_REGISTER_CTRL_REG1       = const(0x2A)   # 00000000   r/w
_FXOS8700_REGISTER_CTRL_REG2       = const(0x2B)   # 00000000   r/w
_FXOS8700_REGISTER_CTRL_REG3       = const(0x2C)   # 00000000   r/w
_FXOS8700_REGISTER_CTRL_REG4       = const(0x2D)   # 00000000   r/w
_FXOS8700_REGISTER_CTRL_REG5       = const(0x2E)   # 00000000   r/w
_FXOS8700_REGISTER_MSTATUS         = const(0x32)
_FXOS8700_REGISTER_MOUT_X_MSB      = const(0x33)
_FXOS8700_REGISTER_MOUT_X_LSB      = const(0x34)
_FXOS8700_REGISTER_MOUT_Y_MSB      = const(0x35)
_FXOS8700_REGISTER_MOUT_Y_LSB      = const(0x36)
_FXOS8700_REGISTER_MOUT_Z_MSB      = const(0x37)
_FXOS8700_REGISTER_MOUT_Z_LSB      = const(0x38)
_FXOS8700_REGISTER_MCTRL_REG1      = const(0x5B)   # 00000000   r/w
_FXOS8700_REGISTER_MCTRL_REG2      = const(0x5C)   # 00000000   r/w
_FXOS8700_REGISTER_MCTRL_REG3      = const(0x5D)   # 00000000   r/w
_ACCEL_MG_LSB_2G          = 0.000244
_ACCEL_MG_LSB_4G          = 0.000488
_ACCEL_MG_LSB_8G          = 0.000976
_MAG_UT_LSB               = 0.1
_SENSORS_GRAVITY_STANDARD = 9.80665

# User-facing constants/module-level globals:
ACCEL_RANGE_2G                    = 0x00
ACCEL_RANGE_4G                    = 0x01
ACCEL_RANGE_8G                    = 0x02


class FXOS8700:

    # Class-level buffer for reading and writing data with the sensor.
    # This reduces memory allocations but means the code is not re-entrant or
    # thread safe!
    _BUFFER = bytearray(13)

    def __init__(self, i2c, address=_FXOS8700_ADDRESS,
                 accel_range=ACCEL_RANGE_2G):
        assert accel_range in (ACCEL_RANGE_2G, ACCEL_RANGE_4G, ACCEL_RANGE_8G)
        self._accel_range = accel_range
        self._device = i2c_device.I2CDevice(i2c, address)
        # Check for chip ID value.
        if self._read_u8(_FXOS8700_REGISTER_WHO_AM_I) != _FXOS8700_ID:
            raise RuntimeError('Failed to find FXOS8700, check wiring!')
        # Set to standby mode (required to make changes to this register)
        self._write_u8(_FXOS8700_REGISTER_CTRL_REG1, 0)
        if accel_range == ACCEL_RANGE_2G:
            self._write_u8(_FXOS8700_REGISTER_XYZ_DATA_CFG, 0x00)
        elif accel_range == ACCEL_RANGE_4G:
            self._write_u8(_FXOS8700_REGISTER_XYZ_DATA_CFG, 0x01)
        elif accel_range == ACCEL_RANGE_8G:
            self._write_u8(_FXOS8700_REGISTER_XYZ_DATA_CFG, 0x02)
        # High resolution
        self._write_u8(_FXOS8700_REGISTER_CTRL_REG2, 0x02)
        # Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode
        self._write_u8(_FXOS8700_REGISTER_CTRL_REG1, 0x15)
        # Configure the magnetometer
        # Hybrid Mode, Over Sampling Rate = 16
        self._write_u8(_FXOS8700_REGISTER_MCTRL_REG1, 0x1F)
        # Jump to reg 0x33 after reading 0x06
        self._write_u8(_FXOS8700_REGISTER_MCTRL_REG2, 0x20)

    def _read_u8(self, address):
        # Read an 8-bit unsigned value from the specified 8-bit address.
        with self._device:
            self._BUFFER[0] = address & 0xFF
            self._device.write(self._BUFFER, end=1)
            self._device.readinto(self._BUFFER, end=1)
        return self._BUFFER[0]

    def _write_u8(self, address, val):
        # Write an 8-bit unsigned value to the specified 8-bit address.
        with self._device:
            self._BUFFER[0] = address & 0xFF
            self._BUFFER[1] = val & 0xFF
            self._device.write(self._BUFFER, end=2)

    def read_raw_accel_mag(self):
        """Read the raw accelerometer and magnetometer readings.  Returns a
        2-tuple of 3-tuples:
          - Accelerometer X, Y, Z axis 14-bit unsigned raw values
          - Magnetometer X, Y, Z axis 16-bit unsigned raw values
        If you want the acceleration or magnetometer values in friendly units
        consider using the accelerometer and magnetometer properties!
        """
        # Read 13 bytes from the sensor.
        with self._device:
            self._BUFFER[0] = _FXOS8700_REGISTER_STATUS | 0x80
            self._device.write(self._BUFFER, end=1)
            self._device.readinto(self._BUFFER)
        # Parse out the accelerometer and magnetometer data.
        status = self._BUFFER[0]
        axhi   = self._BUFFER[1]
        axlo   = self._BUFFER[2]
        ayhi   = self._BUFFER[3]
        aylo   = self._BUFFER[4]
        azhi   = self._BUFFER[5]
        azlo   = self._BUFFER[6]
        mxhi   = self._BUFFER[7]
        mxlo   = self._BUFFER[8]
        myhi   = self._BUFFER[9]
        mylo   = self._BUFFER[10]
        mzhi   = self._BUFFER[11]
        mzlo   = self._BUFFER[12]
        # Shift values to create properly formed integers
        # Note, accel data is 14-bit and left-aligned, so we shift two bit right
        accel_raw_x = (((axhi << 8) | axlo) >> 2) & 0xFFFF
        accel_raw_y = (((ayhi << 8) | aylo) >> 2) & 0xFFFF
        accel_raw_z = (((azhi << 8) | azlo) >> 2) & 0xFFFF
        mag_raw_x = ((mxhi << 8) | mxlo) & 0xFFFF
        mag_raw_y = ((myhi << 8) | mylo) & 0xFFFF
        mag_raw_z = ((mzhi << 8) | mzlo) & 0xFFFF
        return ((accel_raw_x, accel_raw_y, accel_raw_z),
                (mag_raw_x, mag_raw_y, mag_raw_z))

    @property
    def accelerometer(self):
        """Read the acceleration from the accelerometer and return its
        X, Y, Z axis values as a 3-tuple in m/s^2.
        """
        accel_raw, mag_raw = self.read_raw_accel_mag()
        # Convert accel values to m/s^2
        if self._accel_range == ACCEL_RANGE_2G:
            return map(lambda x: x * _ACCEL_MG_LSB_2G * _SENSORS_GRAVITY_STANDARD,
                       accel_raw)
        elif self._accel_range == ACCEL_RANGE_4G:
            return map(lambda x: x * _ACCEL_MG_LSB_4G * _SENSORS_GRAVITY_STANDARD,
                       accel_raw)
        elif self._accel_range == ACCEL_RANGE_8G:
            return map(lambda x: x * _ACCEL_MG_LSB_8G * _SENSORS_GRAVITY_STANDARD,
                       accel_raw)

    @property
    def magnetometer(self):
        """Read the magnetometer values and return its X, Y, Z axis values
        as a 3-tuple in uTeslas.
        """
        accel_raw, mag_raw = self.read_raw_accel_mag()
        # Convert mag values to uTesla
        return map(lambda x: x * _MAG_UT_LSB, mag_raw)
