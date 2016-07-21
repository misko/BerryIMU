#!/usr/bin/env python
# vim: set fileencoding=UTF-8 :

# HMC5888L Magnetometer (Digital Compass) wrapper class
# Based on https://bitbucket.org/thinkbowl/i2clibraries/src/14683feb0f96,
# but uses smbus rather than quick2wire and sets some different init
# params.

import smbus
import math
import time
import sys


data=[]
calibration=False

class HMC5883L:

    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0,0)):
        self.bus = smbus.SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
	#print "ts start",bin(val),val,len
        if (val & (1 << len - 1)):
            #print "ts -sub",1<<len-1
            val = val - (1<<len)
	#print "ts final",bin(val),val,len
        return val

    def __convert(self, data, offset):
	#print bin((data[offset] << 8) | data[offset+1])
        #val = self.twos_complement( (data[offset] << 8) | data[offset+1] , 16)
        val = self.twos_complement( ((data[offset] << 8) | data[offset+1]) & 0x0FFF , 12)
	#print "BIN",bin(data[offset]),bin(data[offset+1]),val,(data[offset] << 8 ) + data[offset+1], ((data[offset] << 8 ) + data[offset+1]) & 0x0FFF
	#val = ((data[offset] << 8 ) + data[offset+1]) & 0x0FFF
	#val = ((data[offset] << 8 ) + data[offset+1]) & 0xFFFF
	#if vv & 0x0800 >0 :
	#	vv -= 1<<12 
	#print vv,vv,vv
        #val = self.twos_complement((data[offset] << 8 | data[offset+1]) & 0x0FFF , 16)
        #val = self.twos_complement((data[offset] << 8 | data[offset+1]) & 0x0FFF , 12)
        #val = data[offset] << 8 | data[offset+1]
	#if val > 32767 :
        #	val -= 65536
        if val == -4096: return None
	#print "BIN val",bin(val),val,self.__scale
        return round(val * self.__scale, 4)
        #return round(val *0.25, 4)

    def axes(self):
        d = self.bus.read_i2c_block_data(self.address, 0x03,6)
        #print map(hex, data)
        x = self.__convert(d, 3-3)+242
	#https://github.com/jarzebski/Arduino-HMC5883L/blob/master/HMC5883L_calibrate/HMC5883L_calibrate.ino
        y = self.__convert(d, 7-3)+502
        z = self.__convert(d, 5-3)
	if calibration:
		data.append((x,y,z))
        return (x,y,z)

    def heading(self):
        (x, y, z) = self.axes()
	#print("DO TAN",y,x)
        #headingRad = math.atan2(y, x)
        #headingRad = math.atan2(-x, -y)
        headingRad = math.atan2(-y, -x)
	#print("X",headingRad)
        headingRad += self.__declination
	#print("Y",headingRad)

        # Correct for reversed heading
        if headingRad < 0:
            headingRad += 2 * math.pi

        # Check for wrap and compensate
        elif headingRad > 2 * math.pi:
            headingRad -= 2 * math.pi

        # Convert to degrees from radians
        headingDeg = headingRad * 180 / math.pi
        return headingDeg

    def degrees(self, headingDeg):
        degrees = math.floor(headingDeg)
        minutes = round((headingDeg - degrees) * 60)
        return (degrees, minutes)

    def __str__(self):
        (x, y, z) = self.axes()
        return "Axis X: " + str(x) + "\n" \
               "Axis Y: " + str(y) + "\n" \
               "Axis Z: " + str(z) + "\n" \
               "Declination: " + self.degrees(self.declination()) + "\n" \
               "Heading: " + self.degrees(self.heading()) + "\n"

if __name__ == "__main__":
    # http://magnetic-declination.com/Great%20Britain%20(UK)/Harrogate#
    #compass = HMC5883L(gauss = 4.7, declination = (13,38)) #san francisco
    #compass = HMC5883L(gauss = 4.7, declination = (0,0)) #san francisco
    if len(sys.argv)!=2:
	print "python %s T/F[calibration]" % sys.argv[0]
	sys.exit(1)
    if sys.argv[1]=='T':
	calibration=True
    compass = HMC5883L(gauss = 1.3, declination = (0,0)) #san francisco
    while True:
        print "Heading: " + str(compass.degrees(compass.heading())) 
        #sys.stdout.write("\rHeading: " + str(compass.degrees(compass.heading())) + "     ")
        #sys.stdout.flush()
	if calibration:
		maxes=list(data[0])
		mins=list(data[0])
		for d in data:
			for i in range(3):
				if d[i]>maxes[i]:
					maxes[i]=d[i]
				if d[i]<mins[i]:
					mins[i]=d[i]	
		print "maxes", maxes
		print "mins", mins	
        time.sleep(0.5)

