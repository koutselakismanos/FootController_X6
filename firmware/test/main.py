import serial
import json
import time
import sys

midi_config = {
    "config": [
        {
            "id": 1, # BOOT BUTTON
            "toggle": {
                "on_value": 127,
                "off_value": 0
            },
            "layers": [
                {"cc_number": 57, "cc_value": 126},
                {"cc_number": 58, "cc_value": 0}
            ],
            "hold_action": "toggle_layer",
            "target_layer": 1
        },
        {
            "id": 2,
            "layers": [
                {"cc_number": 24, "cc_value": 127}, # patch back
                {"cc_number": 63, "cc_value": 0} # looper record
            ],
            "hold_action": "midi",
            "midi_cc": {"number": 25, "value": 0}
        },
        {
            "id": 3,
            "layers": [
                {"cc_number": 25, "cc_value": 127}, # patch forward
                {"cc_number": 62, "cc_value": 0} # looper on/off
            ],
            "toggle": {
                "on_value": 127,
                "off_value": 0
            },
            "hold_action": "midi",
            "midi_cc": {"number": 12, "value": 0}
        },
        {
            "id": 4,
            "layers": [
                {"cc_number": 13, "cc_value": 127},
                {"cc_number": 48, "cc_value": 0}
            ],
            "hold_action": "midi",
            "midi_cc": {"number": 13, "value": 0}
        },
        {
            "id": 5,
            "layers": [
                {"cc_number": 14, "cc_value": 127},
                {"cc_number": 48, "cc_value": 0}
            ],
            "hold_action": "midi",
            "midi_cc": {"number": 14, "value": 0}
        },
        {
            "id": 6,
            "layers": [
                {"cc_number": 15, "cc_value": 127},
                {"cc_number": 48, "cc_value": 0}
            ],
            "hold_action": "midi",
            "midi_cc": {"number": 15, "value": 0}
        },
        {
            "id": 7,
            "layers": [
                {"cc_number": 57, "cc_value": 0},
                {"cc_number": 48, "cc_value": 0}
            ],
            "toggle": {
                "on_value": 110,
                "off_value": 0
            },
            "hold_action": "toggle_layer",
            "target_layer": 1
        }
    ]
}

# Serialize the dictionary to a JSON string
json_str = json.dumps(midi_config)

# Append the delimiter
json_with_end = json_str + '$'

# Encode the string to bytes
payload = json_with_end.encode()

# Check the payload size
payload_size = len(payload)
print(f"Payload size: {payload_size} bytes")

try:
    # Initialize the serial connection
    ser = serial.Serial('COM4', baudrate=9600, timeout=1)
    print("Serial connection established.")

    # Write the payload to the serial port
    ser.write(payload)
    print("Payload sent.")

    # Read and print responses
    try:
        while True:
            if ser.in_waiting > 0:
                response = ser.readline().decode().strip()
                if response:
                    print("Response:", response)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting...")
        sys.exit(0)
    except Exception as e:
        print(f"An error occurred during communication: {e}")
        sys.exit(1)
    finally:
        ser.close()
        print("Serial connection closed.")

except serial.SerialException as e:
    print(f"Failed to establish serial connection: {e}")
    sys.exit(1)
