# Denne koden leser av og printer verdien av accelerometeren på IMU modulen +-2g 
# Det er bare å kopiere inn i Thonny og kjøre den. Den funker på Pico w, usikker om den kjører på vanlig Pico

import machine
import time
import network
import socket
import urequests

# Define the I2C bus
sdaPIN=machine.Pin(4)
sclPIN=machine.Pin(5)

# Initialize the I2C bus
i2c=machine.I2C(0,sda=sdaPIN, scl=sclPIN, freq=400000)

# Define the I2C address of the IMU module (Always 0x69)
imu_addr = 0x69

# Initialize the module by writing to various configuration registers
# (see the module datasheet for details)

# Select bank 0 
i2c.writeto_mem(imu_addr,127,b'\x00')

# Read from register to get current value (This doesnt do anything, just to show how to read and print register values for debugging)
debugRead = i2c.readfrom_mem(imu_addr, 6,1,addrsize=8)       # Read 1 byte from register "6"
debugValue = int.from_bytes(debugRead, "big")                # Convert the byte to an integer
txt = "Binary: {0:08b}"                                      # Create a string with a placeholder for the binary value
print(txt.format(debugValue))                                # Print the binary value


# Write to register to set value 
# Setting this register to 0x01 puts the module out of sleep mode
i2c.writeto_mem(imu_addr,6,b'\x01')                          # Write 0x01 to register "6"

# Read register to check if set correctly
powerMng = i2c.readfrom_mem(imu_addr, 6,1,addrsize=8)       
value = int.from_bytes(powerMng, "big")
txt = "Binary: {0:08b}"
print(txt.format(value))

# Write to bank 0 register 7. Enables accelerometer and gyro.
i2c.writeto_mem(imu_addr,7,b'\x00')

# Select bank 2
i2c.writeto_mem(imu_addr,127,b'\x10')


# Select bank 0
i2c.writeto_mem(imu_addr,127,b'\x00')


time.sleep(1)

# Continuously read and print accelerometer data
while True:    
    # Read 6 bytes of accelerometer data from the module's data registers
    acc_raw = i2c.readfrom_mem(imu_addr, 0x2d, 6)

    #print(acc_raw)
    
    # Convert the raw data to signed 16-bit values and scale by the appropriate range
    acc_x = (acc_raw[0] << 8) | acc_raw[1]
    if acc_x > 32767:
        acc_x -= 65536
    acc_x = acc_x / 1670.0
    
    acc_y = (acc_raw[2] << 8) | acc_raw[3]
    if acc_y > 32767:
        acc_y -= 65536
    acc_y = acc_y / 1670.0
    
    acc_z = (acc_raw[4] << 8) | acc_raw[5]
    if acc_z > 32767:
        acc_z -= 65536
    acc_z = acc_z / 1670.0
    
    # Print the accelerometer data
    print("Accelerometer: X={:.2f}m/s², Y={:.2f}m/s², Z={:.2f}m/s²".format(acc_x, acc_y, acc_z))
    time.sleep_ms(100)

