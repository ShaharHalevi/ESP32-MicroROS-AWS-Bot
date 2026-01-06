#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32QRCodeReader.h>

// ================= USER CONFIGURATION =================
const char* ssid = "iPhone";
const char* password = "12345678";

// MQTT Broker Settings
// We use a public broker for testing. Later you can use "192.168.x.x" (Mosquitto) or AWS.
const char* mqtt_server = "172.20.10.3"; 
const int mqtt_port = 1883;

// The Topic to publish data to
const char* mqtt_topic = "shahar_robot/qr_data";
// ======================================================

WiFiClient espClient;
PubSubClient client(espClient);
ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);

String lastScannedCode = "";

// -------------------------------------------------------------------------
// Helper: Connect to WiFi and MQTT
// -------------------------------------------------------------------------
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      delay(2000);
    }
  }
}

// -------------------------------------------------------------------------
// Task: QR Code Processing
// -------------------------------------------------------------------------
void onQrCodeTask(void *pvParameters)
{
  struct QRCodeData qrCodeData;

  while (true)
  {
    if (reader.receiveQrCode(&qrCodeData, 100))
    {
      if (qrCodeData.valid)
      {
        String currentCode = (const char *)qrCodeData.payload;

        if (currentCode != lastScannedCode) {
            
            Serial.print("[DEBUG] New QR Found: ");
            Serial.println(currentCode);

            // Ensure MQTT is connected before sending
            if (!client.connected()) {
                reconnect();
            }

            // --- SEND VIA MQTT ---
            // publish(topic, payload)
            if (client.publish(mqtt_topic, currentCode.c_str())) {
                Serial.println("[MQTT] Message published successfully");
            } else {
                Serial.println("[MQTT] Publish failed");
            }
            // ---------------------

            lastScannedCode = currentCode;
            
            // Blink LED
            pinMode(33, OUTPUT);
            digitalWrite(33, LOW); 
            delay(100);             
            digitalWrite(33, HIGH); 
        }
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("\n--- ESP32-CAM MQTT Robot ---");

  // 1. WiFi Connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  // 2. MQTT Setup
  client.setServer(mqtt_server, mqtt_port);

  // 3. QR Reader Setup
  reader.setup();
  reader.beginOnCore(1);
  xTaskCreate(onQrCodeTask, "onQrCode", 4 * 1024, NULL, 4, NULL);
  
  Serial.println("System Ready.");
}

void loop()
{
  // Allow the MQTT client to process incoming/outgoing packets
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  delay(10);
}