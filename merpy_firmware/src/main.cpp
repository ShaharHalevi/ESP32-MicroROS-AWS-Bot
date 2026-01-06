#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32QRCodeReader.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ================= USER CONFIGURATION =================
const char* ssid = "iPhone";
const char* password = "12345678";

// MQTT Broker IP
const char* mqtt_server = "172.20.10.3"; 
const int mqtt_port = 1883;

// Topics
const char* topic_data   = "shahar_robot/qr_data";
const char* topic_status = "shahar_robot/status"; // Heartbeat
const char* topic_log    = "shahar_robot/logs";   // Diagnostic Logs
// ======================================================

WiFiClient espClient;
PubSubClient client(espClient);
ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);

const int LED_PIN = 33; // Active LOW

// Timers for diagnostics
unsigned long lastHeartbeat = 0;
unsigned long lastMsgTime = 0; // Tracks time since last QR sent
unsigned long bootTime = 0;    // Tracks when the robot started

// -------------------------------------------------------------------------
// Helper: Smart LED Signaling
// -------------------------------------------------------------------------
void signalError() {
  // SOS Pattern (3 fast blinks)
  for(int i=0; i<3; i++) { digitalWrite(LED_PIN, LOW); delay(100); digitalWrite(LED_PIN, HIGH); delay(100); }
}

void signalSuccess() {
  // One long smooth blink
  digitalWrite(LED_PIN, LOW); delay(500); digitalWrite(LED_PIN, HIGH);
}

// -------------------------------------------------------------------------
// Helper: Send Diagnostic Log
// -------------------------------------------------------------------------
void sendLog(String msg, bool isError = false) {
  Serial.print(isError ? "[ERROR] " : "[LOG] ");
  Serial.println(msg);

  if (client.connected()) {
    String type = isError ? "ERROR" : "INFO";
    String payload = "{\"type\":\"" + type + "\", \"msg\":\"" + msg + "\", \"uptime\":" + String(millis()/1000) + "}";
    client.publish(topic_log, payload.c_str());
  }
}

// -------------------------------------------------------------------------
// Helper: Reconnect Logic with Error Reporting
// -------------------------------------------------------------------------
void attemptReconnect() {
  if (!client.connected()) {
    Serial.print("MQTT Disconnected... ");
    
    // Generate Random Client ID
    String clientId = "ESP32-Diag-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("Reconnected!");
      sendLog("System Online. Connection restored.");
      digitalWrite(LED_PIN, HIGH); // LED OFF (Normal State)
    } else {
      // REPORT THE EXACT ERROR CODE
      // rc=-2: Connect failed (Check IP/Firewall)
      // rc=-4: Connection timeout
      Serial.print("Failed. RC="); 
      Serial.println(client.state()); 
      signalError(); // Visual alert
    }
  }
}

// -------------------------------------------------------------------------
// Task: QR Code Processing
// -------------------------------------------------------------------------
void onQrCodeTask(void *pvParameters)
{
  struct QRCodeData qrCodeData;
  String lastCode = "";

  while (true)
  {
    // Using default setup for High-Res scanning (Best performance)
    if (reader.receiveQrCode(&qrCodeData, 100))
    {
      if (qrCodeData.valid)
      {
        String currentCode = (const char *)qrCodeData.payload;

        if (currentCode != lastCode) {
            
            // Calculate time since last scan
            unsigned long timeSinceLast = (millis() - lastMsgTime) / 1000;
            sendLog("QR Found: " + currentCode + " (Time since last: " + String(timeSinceLast) + "s)");

            if (client.connected()) {
                if (client.publish(topic_data, currentCode.c_str())) {
                    signalSuccess();
                    lastMsgTime = millis(); // Reset timer
                } else {
                    sendLog("Publish Failed! Packet rejected.", true);
                    signalError();
                }
            } else {
                sendLog("Cannot send - MQTT Offline", true);
            }

            lastCode = currentCode;
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// -------------------------------------------------------------------------
// Setup
// -------------------------------------------------------------------------
void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Stability Fix
  bootTime = millis();

  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED ON while booting

  Serial.println("\n--- ESP32 DIAGNOSTIC SYSTEM START ---");

  // WiFi Setup with Timeout Alert
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    retry++;
    if(retry > 20) {
        Serial.println("\n[FATAL] WiFi Connection Timeout! Check Password.");
        while(1) signalError(); // Infinite loop of error blinks
    }
  }
  Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());

  // MQTT Init
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(2048);
  client.setKeepAlive(10); // Check connectivity often

  // Camera Init
  reader.setup(); 
  reader.beginOnCore(1);
  xTaskCreate(onQrCodeTask, "onQrCode", 4 * 1024, NULL, 4, NULL);

  digitalWrite(LED_PIN, HIGH); // LED OFF (Ready)
  sendLog("System Boot Complete");
}

// -------------------------------------------------------------------------
// Loop (Main Monitoring)
// -------------------------------------------------------------------------
void loop()
{
  // 1. Connection Monitor
  if (!client.connected()) {
    static unsigned long lastReconnect = 0;
    if (millis() - lastReconnect > 5000) {
      lastReconnect = millis();
      attemptReconnect();
    }
  } else {
    client.loop();
  }

  // 2. Advanced Heartbeat (Every 3 seconds)
  // This sends a detailed report of the robot's health
  if (millis() - lastHeartbeat > 3000) {
    lastHeartbeat = millis();
    
    if (client.connected()) {
        long rssi = WiFi.RSSI();
        unsigned long uptime = (millis() - bootTime) / 1000;
        unsigned long idleTime = (millis() - lastMsgTime) / 1000;
        
        // Detailed JSON status
        String status = "{";
        status += "\"robot_id\": \"Robot_Cam_01\",";
        status += "\"status\": \"ONLINE\",";
        status += "\"uptime_sec\": " + String(uptime) + ",";
        status += "\"wifi_signal\": " + String(rssi) + ",";   // -50 is good, -90 is bad
        status += "\"last_scan_ago\": " + String(idleTime) + ","; // Time in seconds since last QR sent
        status += "\"ram_free\": " + String(ESP.getFreeHeap());
        status += "}";

        client.publish(topic_status, status.c_str());
        
        // Alert if signal is weak
        if (rssi < -80) sendLog("Warning: Weak WiFi Signal (" + String(rssi) + "dBm)", true);
    }
  }
  
  delay(10);
}