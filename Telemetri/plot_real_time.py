import imufusion
import numpy
import sys
import serial
import time 
import math

sample_rate = 100  # 100 Hz
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ser = serial.Serial('/dev/ttyACM1', 115200)  # Replace 'COM1' with your serial port name
dataStr = ser.readline().decode().strip()  # Read a line of data from the serial port

data = numpy.fromstring(dataStr, dtype=float, sep=',')

prevtime = data[0]/1000000

ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                   0.5,  # gain
                                   10,  # acceleration rejection
                                   20,  # magnetic rejection
                                   5 * sample_rate)  # rejection timeout = 5 seconds


while True:
    dataStr = ser.readline().decode().strip()  # Read a line of data from the serial port

    data = numpy.fromstring(dataStr, dtype=float, sep=',')


    dt = data[0]/1000000 - prevtime
    prevtime = data[0]/1000000

    gyroscope = offset.update(data[4:7])
    
    ahrs.update(gyroscope, data[1:4], data[7:10], dt)

    euler = ahrs.quaternion.to_euler()


    mag_x = data[7]
    mag_y = data[8]

    # Convert magnetometer data to a compass heading (in radians)
    heading = math.atan2(mag_y, mag_x)

    # Convert heading to degrees
    heading_degrees = heading * 180 / math.pi

    # Adjust heading for magnetic declination (in radians)
    declination = 9.20  # Example magnetic declination for San Francisco
    heading -= declination

    # Make sure heading is in the range [0, 360)
    if heading_degrees < 0:
        heading_degrees += 360


    #print euler
    print("Roll: " + str(euler[0]) + " pitch: " + str(euler[1]) + " yaw: " + str(euler[2]) + " heading: " + str(heading_degrees))

    
    

