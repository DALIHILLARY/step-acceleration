import serial

# Specify the serial port name and baud rate
serial_port = 'COM4'  # Change this to match your Arduino port name
baud_rate = 115200

# Open a serial connection
ser = serial.Serial(serial_port, baud_rate)

# Open a text file for writing received data
filename = 'hix_data-home.txt'
with open(filename, 'w') as file:
    while True:
        # Read a line of data from the serial port
        data = ser.readline().decode().strip()

        # Write the received data to the text file
        file.write(data + '\n')
        file.flush()  # Flush the buffer to ensure data is written immediately

        # Print the received data to the console
        print(data)

