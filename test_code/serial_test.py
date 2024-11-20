import serial
import keyboard  # Requires installation: pip install keyboard
import sys, termios, tty, select
import time

# Set up the serial connection (adjust 'COM3' to your Arduino's port)
serial_port = '/dev/ttyACM0'  # Change this to your Arduino's port
baud_rate = 9600
settings = termios.tcgetattr(sys.stdin)

# Function to get keyboard input
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    return key

# Initialize serial connection
# ser = serial.Serial(serial_port, baud_rate)



try:
    print("Press any key to send data to the Arduino...")
    while True:
        # Wait for a key press
        # event = keyboard.read_event()
        key = getKey()
        if key == 'q':
            break
        
        # Check if the event is a key press and if it's 'g' or 'b'
        # ser.write(event.name.encode())  # Send the command as a byte
        print(f"\rData: '{key}' sent to Arduino.")
        sys.stdout.flush()
        time.sleep(0.05)
        
except KeyboardInterrupt:
    print("Program exited.")
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    # ser.close()  # Ensure the serial connection is closed

