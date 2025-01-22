import serial
import time

def fetch_individual_commands():
    """
    Sends SCPI commands `FETC:VOLT?` and `FETC:CURR?` alternately, one at a time, 
    every second to a device connected to COM3 at 9600 baud, and reads back the responses.
    """
    com_port = "COM3"  # Replace with your specific COM port
    baud_rate = 9600

    try:
        # Open the serial port
        with serial.Serial(com_port, baud_rate, timeout=2) as ser:
            print(f"Connected to {com_port} at {baud_rate} baud.")

            while True:
                # Send and receive voltage
                voltage_command = "FETC:VOLT?"
                print(f"Sending command: {voltage_command}")
                ser.write((voltage_command + '\n').encode())
                voltage_response = ser.readline().decode().strip()
                print(f"Response to {voltage_command}: {voltage_response}")

                # Wait for 1 second
                time.sleep(1)

                # Send and receive current
                current_command = "FETC:CURR?"
                print(f"Sending command: {current_command}")
                ser.write((current_command + '\n').encode())
                current_response = ser.readline().decode().strip()
                print(f"Response to {current_command}: {current_response}")

                # Wait for 1 second
                time.sleep(1)

    except serial.SerialException as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    fetch_individual_commands()
