import smbus
import time

#https://github.com/ControlEverythingCommunity/ITG3200/blob/master/Python/ITG_3200.py

class SensorITG3200(object):
    """ITG3200 digital gyroscope control class.
    Supports data polling at the moment.
    """
    def __init__(self, bus_nr, addr):
      self.bus = smbus.SMBus(1)
      # ITG3200 address, 0x68(104)
      # Select Power management register 0x3E(62)
      #		0x01(01)	Power up, PLL with X-Gyro reference
      self.bus.write_byte_data(0x68, 0x3E, 0x01)
      # ITG3200 address, 0x68(104)
      # Select DLPF register, 0x16(22)
      #		0x18(24)	Gyro FSR of +/- 2000 dps
      self.bus.write_byte_data(0x68, 0x16, 0x18)
      time.sleep(0.5)


    def read_data(self):
      # ITG3200 address, 0x68(104)
      # Read data back from 0x1D(29), 6 bytes
      # X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
      data = self.bus.read_i2c_block_data(0x68, 0x1D, 6)
      # Convert the data
      xGyro = data[0] * 256 + data[1]
      if xGyro > 32767 :
        xGyro -= 65536
      yGyro = data[2] * 256 + data[3]
      if yGyro > 32767 :
        yGyro -= 65536
      zGyro = data[4] * 256 + data[5]
      if zGyro > 32767 :
        zGyro -= 65536
      return xGyro,yGyro,zGyro

if __name__ == '__main__':
    import time
    sensor = SensorITG3200(1, 0x68) # update with your bus number and address
    #sensor.default_init()
    time.sleep(0.1)
    gx, gy, gz = sensor.read_data()
    print gx, gy, gz
