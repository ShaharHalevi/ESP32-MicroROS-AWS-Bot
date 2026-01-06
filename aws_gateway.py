from awsiot import mqtt5_client_builder
from awscrt import mqtt5
import threading
import time
import json
import paho.mqtt.client as paho

# ==================================================================
#                       USER CONFIGURATION
# ==================================================================

# --- AWS Cloud Settings ---
# 1. Your AWS IoT Endpoint (Found in AWS Console -> Settings)
AWS_ENDPOINT = "<your-iot-endpoint>.amazonaws.com"  

# 2. Paths to your certificates (Relative to this script)
PATH_TO_CERT = "ESP32_CAM.cert.pem"
PATH_TO_KEY  = "ESP32_CAM.private.key"

# 3. The Topic in the Cloud where data will be sent
AWS_TOPIC    = "aws/robot/data" 

# --- Local Robot Settings ---
# 4. Local Broker Address (localhost = this computer)
LOCAL_BROKER = "localhost"      
LOCAL_PORT   = 1883

# 5. The Topic the ESP32 writes to (Must match ESP32 code!)
LOCAL_TOPIC  = "shahar_robot/qr_data" 

# ==================================================================

# Global AWS Client variable
client_aws = None

# --- AWS MQTT5 Callbacks ---

def on_lifecycle_connection_success(lifecycle_connect_success_data):
    print("[AWS] Connected to AWS IoT Core successfully!")

def on_lifecycle_connection_failure(lifecycle_connection_failure):
    print("[AWS] Connection failed:", lifecycle_connection_failure.exception)

def on_lifecycle_disconnection(lifecycle_disconnect_data):
    print("[AWS] Disconnected")

# --- Bridge Logic (Local -> Cloud) ---

def on_local_message(client, userdata, msg):
    """
    Triggered when a message is received from the ESP32 (Local Mosquitto).
    Forwards the data to AWS IoT Core.
    """
    try:
        # 1. Decode message from Robot
        payload = msg.payload.decode()
        print(f"[Local] Received from Robot: {payload}")

        try:
            barcode_data = json.loads(payload)
        except json.JSONDecodeError:
            print("Error: Robot sent invalid JSON, sending as raw text")
            barcode_data = {"raw_content": payload}

        # 2. Prepare JSON payload for Cloud
        cloud_message = {
            "device": "ESP32_Robot",
            "barcode": barcode_data,
            "timestamp": time.time()
        }
        message_json = json.dumps(cloud_message)

        # 3. Publish to AWS
        if client_aws:
            publish_future = client_aws.publish(mqtt5.PublishPacket(
                topic=AWS_TOPIC,
                payload=message_json,
                qos=mqtt5.QoS.AT_LEAST_ONCE
            ))
            print(f"[Bridge] Forwarded to AWS Topic: {AWS_TOPIC}")
        else:
            print("[Bridge] Error: AWS Client not connected")

    except Exception as e:
        print(f"[Bridge] Error processing message: {e}")

# ==================================================================
#                       Main Execution
# ==================================================================

if __name__ == '__main__':
    print("\n--- Starting AWS IoT Gateway Bridge ---\n")

    # 1. Initialize AWS MQTT5 Client
    client_aws = mqtt5_client_builder.mtls_from_path(
        endpoint=AWS_ENDPOINT,
        cert_filepath=PATH_TO_CERT,
        pri_key_filepath=PATH_TO_KEY,
        on_lifecycle_connection_success=on_lifecycle_connection_success,
        on_lifecycle_connection_failure=on_lifecycle_connection_failure,
        on_lifecycle_disconnection=on_lifecycle_disconnection,
        client_id="Python_Gateway_Bridge"
    )

    print("[AWS] Connecting...")
    client_aws.start()

    # Wait a moment for AWS connection
    time.sleep(2)

    # 2. Initialize Local Paho Client (Listener)
    client_local = paho.Client()
    client_local.on_message = on_local_message

    print(f"[Local] Connecting to Mosquitto ({LOCAL_BROKER})...")
    try:
        client_local.connect(LOCAL_BROKER, LOCAL_PORT, 60)
        client_local.subscribe(LOCAL_TOPIC)
        print(f"[Local] Listening for Robot on topic: '{LOCAL_TOPIC}'")
        
        # Keep script running
        client_local.loop_forever()

    except KeyboardInterrupt:
        print("\nStopping bridge...")
        client_aws.stop()
        print("Done.")