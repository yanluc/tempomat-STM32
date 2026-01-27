import serial
import time
import sys

# Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIMEOUT = 1

def main():
    try:
        # Note: serial.sevenbits is used because the STM32 is configured with UART_WORDLENGTH_7B
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        )
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    try:
        toggle = True
        while True:
            # Alternate between 20 and -20
            setpoint = "20" if toggle else "-20"
            print(f"Setting velocity to: {setpoint} rad/s")
            
            # Send the setpoint followed by newline
            ser.write(f"{setpoint}\n".encode('ascii'))
            
            # Briefly read and print any incoming telemetry
            time.sleep(0.1)
            while ser.in_waiting:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line:
                    print(f"  [STM32] {line}")
            
            toggle = not toggle
            
            # Wait 5 seconds before next switch
            time.sleep(4.9)
            
    except KeyboardInterrupt:
        print("\nScript stopped by user.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
