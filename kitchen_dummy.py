import serial
import time
import argparse
import re

def main():
    parser = argparse.ArgumentParser(description="Kitchen Robot Dummy (UART to ESP32)")
    parser.add_argument("--port", type=str, default="/dev/ttyS0", help="Serial port to connect to")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    print("Connected. Waiting for Waiter commands...")

    buffer = ""
    while True:
        try:
            if ser.in_waiting > 0:
                char = ser.read().decode('utf-8', errors='ignore')
                buffer += char

                if '>' in buffer:
                    # Extract command inside < >
                    match = re.search(r'<(.*?)>', buffer)
                    if match:
                        cmd = match.group(1)
                        print(f"Received: <{cmd}>")
                        
                        if cmd == "HS":
                            print("Received Handover Start. Preparing...")
                            time.sleep(0.5)
                            print("Sending <HR> (Handover Ready)")
                            ser.write(b"<HR>")
                        
                        elif cmd.startswith("O"):
                            print(f"Received Order: {cmd}. Cooking...")
                            time.sleep(2.0)
                            print("Sending <DR> (Dish Ready)")
                            ser.write(b"<DR>")

                        # Clear buffer up to the matched command
                        buffer = buffer[buffer.find('>')+1:]
                    else:
                        # Clear buffer if malformed
                        if len(buffer) > 100:
                            buffer = ""

            time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            break
            
    ser.close()

if __name__ == "__main__":
    main()
