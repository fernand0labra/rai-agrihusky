import serial
import time
import string

def is_printable(b):
    return chr(b) in string.printable and b != 0x00

probe = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Give Arduino time to reset

probe.write(b'\x00')  # Send PROBE_START

status = None
result = None
buffer = bytearray()

while True:
    if probe.in_waiting > 0:
        byte = probe.read(1)
        b = byte[0]

        if is_printable(b):
            buffer.append(b)

            # Check for end of line
            if b == ord('\n'):
                print("VERBOSE:", buffer.decode(errors='ignore').strip())
                buffer.clear()
        else:
            # Handle binary protocol
            if b == b'\xFF':
                status = b
                result_byte = probe.read(1)
                result = result_byte[0]
                print(f"Status: 0x{status:02X}, Result: 0x{result:02X}")
                break
            else:
                print(f"Unknown binary byte received: 0x{b:02X}")
    time.sleep(0.01)
