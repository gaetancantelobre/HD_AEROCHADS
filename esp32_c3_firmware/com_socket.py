import socket
import struct
import time

# --- Configuration ---
# *** IMPORTANT: Change this IP ADDRESS to the one printed by the ESP32! ***
ESP32_IP = "192.168.1.94" 
TCP_PORT = 3333

def send_servo_values(servo_angles):
    """
    Connects to the ESP32 server and sends a 5-byte packet of servo angles.
    
    :param servo_angles: A list or tuple of 5 integers (0-180 degrees).
    """
    if len(servo_angles) != 5:
        print(f"Error: Expected 5 servo values, got {len(servo_angles)}")
        return

    # Check and clamp values for safety (although the ESP32 handles clamping too)
    clamped_angles = [max(0, min(180, a)) for a in servo_angles]

    try:
        # Create a TCP/IP socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print(f"Connecting to {ESP32_IP}:{TCP_PORT}...")
            s.connect((ESP32_IP, TCP_PORT))
            print("Connection successful.")

            # Pack 5 unsigned 8-bit integers (uint8_t) into a byte string.
            # '5B' means 5 unsigned chars (bytes), which perfectly matches the ESP32's expectation.
            data_to_send = struct.pack('5B', *clamped_angles)
            
            s.sendall(data_to_send)
            print(f"Sent angles: {clamped_angles} (Raw bytes: {data_to_send.hex()})")
            
            # Keep the connection open momentarily before letting the context manager close it.
            time.sleep(0.1) 

    except ConnectionRefusedError:
        print(f"Error: Connection refused. Check if the ESP32 is running and the IP address is correct.")
    except TimeoutError:
        print("Error: Connection timed out. Check network connectivity.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == '__main__':
    # --- Example Usage ---
    
    # Example 1: Move all servos to 90 degrees (center)
    angles_center = [90, 90, 90, 90, 90]
    print("\n--- Sending Center (90 degrees) ---")
    send_servo_values(angles_center)
    time.sleep(2) 

    # Example 2: Move servos to a spread pattern
    angles_test = [10, 50, 90, 130, 170]
    print("\n--- Sending Spread Positions ---")
    send_servo_values(angles_test)
    time.sleep(2)

    # Example 3: Move them all back to 0 degrees
    angles_zero = [0, 0, 0, 0, 0]
    print("\n--- Sending Zero Position ---")
    send_servo_values(angles_zero)
    time.sleep(1)