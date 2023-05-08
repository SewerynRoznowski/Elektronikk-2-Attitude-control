import serial

# Open a serial connection
ser = serial.Serial('/dev/ttyACM0', 115200)  # Replace 'COM1' with your serial port name

# Open a file for logging
log_file = open('serial_log.txt', 'w')

# Read and log data
while True:
    data = ser.readline().decode().strip()  # Read a line of data from the serial port
    log_file.write(data + '\n')  # Write the data to the log file
    print(data)  # Print the data to the console (optional)

# Close the serial connection and log file
ser.close()
log_file.close()