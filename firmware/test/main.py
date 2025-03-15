import serial
import json

midi_config = {
    "config": [
        {
            "id": 1,
            "layers": [
                {"cc_number": 49, "cc_value": 127},
                {"cc_number": 48, "cc_value": 0}
            ],
            "hold_action": "midi",
            "target_layer": {"number": 22, "value": 9},
        },
        {
            "id": 2,
            "layers": [
                {"cc_number": 48, "cc_value": 127},
                {"cc_number": 48, "cc_value": 0}
            ],
            "hold_action": "midi",
            "midi_cc": {"number": 66, "value": 50}
        },
        {
            "id": 3,
            "layers": [
                {"cc_number": 48, "cc_value": 127},
                {"cc_number": 48, "cc_value": 0}
            ],
            "hold_action": "midi",
            "midi_cc": {"number": 66, "value": 50}
        },
        {
            "id": 4,
            "layers": [
                {"cc_number": 48, "cc_value": 127},
                {"cc_number": 48, "cc_value": 0}
            ],
            "hold_action": "midi",
            "midi_cc": {"number": 66, "value": 50}
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
print(f"Payload size: {payload_size} bytes: payload: {payload}")

# Initialize the serial connection
ser = serial.Serial('COM4')

# Write the payload to the serial port
ser.write(payload)

# Read and print responses
while True:
    response = ser.readline()
    print("Response:", response.decode())

# Close the serial connection
ser.close()
