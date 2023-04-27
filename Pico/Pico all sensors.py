# Denne koden leser av og printer verdien av accelerometeren, gyroskop og magnetometer på IMU modulen 
# Det er bare å kopiere inn i Thonny og kjøre den. Den funker på Pico w, usikker om den kjører på vanlig Pico

import machine
import time

# Define the I2C bus
sdaPIN=machine.Pin(4)
sclPIN=machine.Pin(5)

# Initialize the I2C bus
i2c=machine.I2C(0,sda=sdaPIN, scl=sclPIN, freq=400000)

# Define the I2C address of the IMU module (Always 0x69)
imu_addr = 0x69
mag_addr = 0x0c

# Initialize accelerometer and gyroscope  module by writing to various configuration registers
# (see the module datasheet for details)
# Select bank 0 
i2c.writeto_mem(imu_addr,127,b'\x00')

# Write to register to set value 
# Setting this register to 0x01 puts the module out of sleep mode
i2c.writeto_mem(imu_addr,6,b'\x01')                          # Write 0x01 to register "6"

# Enable bypass mode for I2C bus, so that the magnetometer can be accessed directly
i2c.writeto_mem(imu_addr,15,b'\x02')

# Write to bank 0 register 7. Enables accelerometer and gyro.
i2c.writeto_mem(imu_addr,7,b'\x00')

# Initialize magnetometer module by writing to various configuration registers
# (see the module datasheet for details)
# Enable magnetometer
i2c.writeto_mem(mag_addr,0x31,b'\x08')

time.sleep_ms(1000)

# Continuously read and print accelerometer data
while True:    
    # Read data from accelerometer and gyroscope
    # Read 6 bytes of accelerometer data from the module's data registers
    acc_raw = i2c.readfrom_mem(imu_addr, 0x2d, 6)
    # Read 6 bytes of gyroscope data from the module's data registers
    gyr_raw = i2c.readfrom_mem(imu_addr, 0x33, 6)
    
    # Read data from magnetometer
    # Read 6 bytes of magnetometer data from the module's data registers
    mag_rar = i2c.readfrom_mem(mag_addr, 0x11, 6)
    # Read 1 byte of magnetometer status from the module's status register
    # It is necessary to read this register AFTER reading the magnetometer data registers, otherwise the data registers will not update. 
    mag_stat = i2c.readfrom_mem(mag_addr,0x18,1)
    
    # Convert the raw accelerometer data to signed 16-bit values and scale by the appropriate range
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
    
    # Convert the raw gyroscope data to signed 16-bit values and scale by the appropriate range
    gyr_x = (gyr_raw[0] << 8) | gyr_raw[1]
    if gyr_x > 32767:
        gyr_x -= 65536
    
    gyr_y = (gyr_raw[2] << 8) | gyr_raw[3]
    if gyr_y > 32767:
        gyr_y -= 65536
    
    gyr_z = (gyr_raw[4] << 8) | gyr_raw[5]
    if gyr_z > 32767:
        gyr_z -= 65536
    
    # Convert the raw magnetometer data to signed 16-bit values and scale by the appropriate range
    mag_x = (mag_rar[1] << 8) | mag_rar[0]
    if mag_x > 32767:
        mag_x -= 65536

    mag_y = (mag_rar[3] << 8) | mag_rar[2]
    if mag_y > 32767:
        mag_y -= 65536
    
    mag_z = (mag_rar[5] << 8) | mag_rar[4]
    if mag_z > 32767:
        mag_z -= 65536

    # Print the accelerometer data
    print("Accelerometer: X={:.2f}m/s², Y={:.2f}m/s², Z={:.2f}m/s²".format(acc_x, acc_y, acc_z))
    
    # Print the gyroscope data
    print("Gyroscope:     X={:.2f}°/s, Y={:.2f}°/s, Z={:.2f}°/s".format(gyr_x, gyr_y, gyr_z))

    # Print the magnetometer data
    print("Magnetometer:  X={:.2f}μT, Y={:.2f}μT, Z={:.2f}μT".format(mag_x, mag_y, mag_z))
    
    time.sleep_ms(100)



