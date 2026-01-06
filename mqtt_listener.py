import paho.mqtt.client as mqtt

# ================= CONFIGURATION =================
# Must match the settings in the ESP32 code
BROKER = "localhost" 
PORT = 1883
TOPIC = "shahar_robot/qr_data"
# =================================================

# Callback: Executed when connection is established
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT Broker!")
        print(f" Subscribing to topic: {TOPIC}")
        client.subscribe(TOPIC)
    else:
        print(f" Failed to connect, return code {rc}")

# Callback: Executed when a message is received
def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print("-" * 40)
    print(f" NEW QR CODE RECEIVED!")
    print(f"   Topic:   {msg.topic}")
    print(f"   Content: {payload}")
    print("-" * 40)
    
    # FUTURE: Here you can send 'payload' to AWS or Database

# Main Setup
if __name__ == '__main__':
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    print(f" Connecting to {BROKER}...")
    client.connect(BROKER, PORT, 60)

    # Start the loop to process network traffic
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n Disconnected.")