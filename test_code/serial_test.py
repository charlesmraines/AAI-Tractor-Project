import serial
import keyboard  # Requires installation: pip install keyboard

# Set up the serial connection (adjust 'COM3' to your Arduino's port)
serial_port = '/dev/ttyACM0'  # Change this to your Arduino's port
baud_rate = 9600

# Initialize serial connection
ser = serial.Serial(serial_port, baud_rate)

try:
    print("Press any key to send data to the Arduino...")
    while True:
        # Wait for a key press
        event = keyboard.read_event()
        
        # Check if the event is a key press and if it's 'g' or 'b'
        ser.write(event.name.encode())  # Send the command as a byte
        print(f"Data '{event.name}' sent to Arduino.")
        
except KeyboardInterrupt:
    print("Program exited.")
finally:
    ser.close()  # Ensure the serial connection is closed

