#include <WiFi.h>
#include <esp_now.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"  // For watchdog resets

#define FIRMWARE_FILE_PATH "/firmware.bin"  // Save firmware in SPIFFS
#define DOWNLOAD_TIMEOUT 30000              // 15 seconds timeout
#define CHUNK_SIZE 240                       // ESP-NOW packet size limit
#define WINDOW_SIZE 4                        // Number of packets sent before waiting for ACK
#define BATCH_SIZE 5

// Acknowledge structure
typedef struct {
    uint8_t slaveAddress[6];
    bool ack = false;
} Acknowledge;

typedef struct {
    char sensorType[10];
    int numValues;
    float values[2];
    char sensorID[20];
} SensorData;

typedef struct{
   char serverversion[10] ; // Buffer to store received OTA version
   char slaveversion[10] ;  //store version of slave
   char firmwareUrl[128];
   uint8_t slaveAddress[6];  //update a microcontroller
} updatemac;

typedef struct {
    uint8_t slaveAddress[6];
    bool ack;
} ackData_t;

ackData_t ackData;
SensorData receivedData;
bool batchReady = false ;  // when batch will send
DynamicJsonDocument doc(4096);
JsonArray batchArray = doc.to<JsonArray>();  // JSON array for batch storage
int batchCount = 0; // Track the number of received packets
bool esp_now=true  ;// by default its true to maitain sattus of es32
unsigned long previousMillis = 0;  // Stores the last execution time
const unsigned long interval = 30000;  // how much later check ota requesrt from server
bool inloop=true ;  // maitain ota state 
updatemac Otarun ;
const char* ssid = "FTTH";
const char* password = "12345678";
const char* serverUrl = "http://172.25.2.129:3001";  
const char* otaUrl = "http://172.25.2.129:3001/firmwareuse";
const char *mongoDBEndpoint = "http://172.25.2.129:3001/sensor-data-batch";
bool sending=false ;
bool pair=false ;
bool slave_version_send=false ;

bool sendMessage(const uint8_t *mac_addr, const uint8_t *message, size_t messageLength) {
    memcpy(ackData.slaveAddress, mac_addr, 6);
    ackData.ack = false;

    if (esp_now_send(mac_addr, message, messageLength) != ESP_OK) {
        Serial.println("Error: Failed to send ESP-NOW message.");
        return false;
    }

    unsigned long startTime = millis();
    while (millis() - startTime < 500) { // Wait up to 500ms for ACK
        if (ackData.ack) return true;
        delay(10);
    }

    return false; // Timeout, return failure
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t sendStatus) {
    if(sendStatus == ESP_NOW_SEND_SUCCESS)
     {
        Serial.println("Packet sent successfully") ;
        slave_version_send=true ;
     }
     else Serial.println("Failed to send packet") ;
}

// ESP-NOW Receive Callback
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
      if(sending)
      {   if(memcmp(ackData.slaveAddress, info->src_addr, 6) == 0)
          { ackData.ack=true ; }
          return ;
      }
      if (len != sizeof(receivedData)&&len!=sizeof(Otarun.slaveversion)) 
      {
        Serial.println("Invalid data size received.");
        return;
      }
      if(len==sizeof(Otarun.slaveversion))
      {
          if(memcmp(Otarun.slaveAddress, info->src_addr, 6) != 0) return ;
          memcpy(Otarun.slaveversion, data, sizeof(Otarun.slaveversion));  // Copy data to slaveversion
          Serial.printf("Received firmware version: %s\n", Otarun.slaveversion);
      }
      else
      {
        memcpy(&receivedData, data, sizeof(receivedData));
        
          char macStr[18];
          snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                  info->src_addr[0], info->src_addr[1], info->src_addr[2],
                  info->src_addr[3], info->src_addr[4], info->src_addr[5]);

          // Display received data on Serial Monitor 📺
          Serial.println("=====================================");
          Serial.printf("📍 Sensor ID: %s\n", receivedData.sensorID);
          Serial.printf("🔍 Sensor Type: %s\n", receivedData.sensorType);
          Serial.printf("🛜 MAC Address: %s\n", macStr);
          Serial.print("📊 Values: ");
          for (int i = 0; i < receivedData.numValues; i++) {
            Serial.printf("Value %d: [%.2f] ", i + 1, receivedData.values[i]);
          }
          Serial.println("\n=====================================");

          // Create new JSON object inside the batch array
          JsonObject sensorData = batchArray.createNestedObject();
          sensorData["sensorId"] = receivedData.sensorID;
          sensorData["sensorType"] = receivedData.sensorType;
          sensorData["macAddress"] = macStr; // Store MAC as a string

          // Add values as an array
          JsonArray valuesArray = sensorData.createNestedArray("values");
          for (int i = 0; i < receivedData.numValues; i++) {
              valuesArray.add(receivedData.values[i]);
          }

          batchCount++; // Increment batch count

          if (batchCount == BATCH_SIZE) {
            batchReady = true; // Set flag when batch is ready
        }
      }
}

bool sendmessage(const uint8_t *mac_addr, const uint8_t *message, size_t messageLength) {
    memcpy(ackData.slaveAddress, mac_addr, 6);  // Store MAC address
    ackData.ack = false;  // Reset acknowledgment status

    // Send the message via ESP-NOW
    if (esp_now_send(mac_addr, message, messageLength) != ESP_OK) {
        Serial.println("Error: Failed to send ESP-NOW message.");
        return false;
    }

    // Wait for acknowledgment
    unsigned long startTime = millis();
    while (millis() - startTime < 500) { // Wait up to 500ms for ACK
        if (ackData.ack) return true;
        delay(10);  // Small delay to avoid CPU overload
    }

    return false; // If timeout, return false
}

bool sendBatchToDatabase() {
    if (!wificonnect()) 
    {
      doc.clear();
      batchArray = doc.to<JsonArray>(); 
      batchCount = 0 ;
      batchReady = false;
      return false ;
    }
    bool ans=false ;
    HTTPClient http;
    http.end() ;
    delay(200) ;
    http.begin(mongoDBEndpoint);
    http.addHeader("Content-Type", "application/json");

    String jsonPayload;
    serializeJson(doc, jsonPayload); // Convert JSON array to string

    Serial.println("Sending JSON batch:");
    Serial.println(jsonPayload);

    int httpResponseCode = http.POST(jsonPayload);
    if (httpResponseCode == HTTP_CODE_OK) {
        ans=true ;
    } else {
        Serial.printf("Batch send failed. HTTP Code: %d\n", httpResponseCode);
    }
    http.end();
    doc.clear();
    batchArray = doc.to<JsonArray>(); 
    batchCount = 0 ;
    batchReady = false;
    return ans ;
}

bool setupespnow() {  
    esp_now=true ;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Initialization Failed!");
        return false;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(onDataRecv);
    return true;
}

bool wificonnect() {
    WiFi.disconnect(true);  // Reset WiFi
    delay(200);
    WiFi.mode(WIFI_STA);    // Station mode
    WiFi.begin(ssid, password);
    
    Serial.print("Connecting to WiFi");
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) { // 10s timeout
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n✅ WiFi Connected! IP: " + WiFi.localIP().toString());
        esp_now=false ;
        return true;
    } else {
        Serial.println("\n❌ WiFi Failed! Status: " + String(WiFi.status()));
        return false;
    }
}

bool checkForOTAUpdate() {
    if (!wificonnect()) return false;

    WiFiClient client;
    HTTPClient http;
    http.end();
    delay(200);
    Serial.println("Checking OTA Request...");

    const char* labNumber = "Lab102";
    String requestUrl = String(serverUrl) + "/getData?lab=" + String(labNumber);
    
    if (!http.begin(client, requestUrl)) {
        Serial.println("Failed to connect to server.");
        client.stop();
        return false;
    }

    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("HTTP Request failed. Code: %d\n", httpCode);
        http.end();
        client.stop();
        return false;
    }

    String payload = http.getString();
    if (payload.length() == 0) {
        Serial.println("Empty response from server!");
        http.end();
        client.stop();
        return false;
    }

    Serial.println("Server Response: " + payload);

    // Increased buffer size for JSON parsing
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.print("JSON Parse Error: ");
        Serial.println(error.c_str());
        http.end();
        client.stop();
        return false;
    }

    if (doc.containsKey("error")) {
        Serial.println("No OTA update available.");
        http.end();
        client.stop();
        return false;
    }

    // Extract version from response
    String serverVersion = doc["version"].as<String>();
    if (serverVersion.length() == 0) {
        Serial.println("Invalid version in response");
        http.end();
        client.stop();
        return false;
    }

    // Extract MAC address
    String macStr = doc["mac"].as<String>();
    if (macStr.length() == 17) {  // MAC addresses are 17 characters (XX:XX:XX:XX:XX:XX)
        int values[6];
        if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x", 
                  &values[0], &values[1], &values[2], 
                  &values[3], &values[4], &values[5]) == 6) {
            for (int i = 0; i < 6; i++) {
                Otarun.slaveAddress[i] = (uint8_t)values[i];
            }
            Serial.print("Received MAC Address: ");
            for (int i = 0; i < 6; i++) {
                Serial.printf("0x%X ", Otarun.slaveAddress[i]);
            }
            Serial.println();
        } else {
            Serial.println("Failed to parse MAC Address!");
            http.end();
            client.stop();
            return false;
        }
    } else {
        Serial.println("Invalid MAC Address length.");
        http.end();
        client.stop();
        return false;
    }

    // Extract firmware URL
    String firmwareUrl = doc["firmwareUrl"].as<String>();
    if (firmwareUrl.length() == 0) {
        Serial.println("No firmware URL in response");
        http.end();
        client.stop();
        return false;
    }

    // Store version and URL
    strncpy(Otarun.serverversion, serverVersion.c_str(), sizeof(Otarun.serverversion) - 1);
    Otarun.serverversion[sizeof(Otarun.serverversion) - 1] = '\0';
    strncpy(Otarun.firmwareUrl, firmwareUrl.c_str(), sizeof(Otarun.firmwareUrl) - 1);
    Otarun.firmwareUrl[sizeof(Otarun.firmwareUrl) - 1] = '\0';
    memset(Otarun.slaveversion, 0, sizeof(Otarun.slaveversion));

    Serial.print("Stored OTA Version: ");
    Serial.println(Otarun.serverversion);
    Serial.print("Firmware URL: ");
    Serial.println(Otarun.firmwareUrl);

    http.end();
    client.stop();
    return true;
}

bool setupESPNowpair(uint8_t PairAddress[]) { 
    esp_now = true ;
    pair=true ;
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(1000);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Initialization Failed!");
        return false;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, PairAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("ESP-NOW Peer added.");
        return true;
    } else {
        Serial.println("Failed to add ESP-NOW peer.");
        return false;
    }
}

bool checkversion() {   
    if (!setupESPNowpair(Otarun.slaveAddress)) return false;

    const char* message = "checkversion";
    slave_version_send = false;  // Reset before sending

    if (esp_now_send(Otarun.slaveAddress, (uint8_t*)message, strlen(message) + 1) != ESP_OK) {
        Serial.println("Failed to send version check message");
        return false;
    }

    // Wait for send to complete (up to 200ms)
    unsigned long sendStart = millis();
    while (!slave_version_send && millis() - sendStart < 200) {
        delay(20);
    }

    if (!slave_version_send) {
        Serial.println("Send callback not triggered in time.");
    }

    // Wait for slave version response
    bool sver = false;
    unsigned long startTime = millis();
    while (millis() - startTime < 2000) {
        delay(100);
        if (Otarun.slaveversion[0] != '\0') {
            sver = true;
            break;
        }
    }

    if (!sver) {
        Serial.println("Slave version not received");
        return false;
    }

    Serial.printf("Master OTA Version: %s\n", Otarun.serverversion);
    Serial.printf("Slave Reported Version: %s\n", Otarun.slaveversion);

    if (strcmp(Otarun.serverversion, Otarun.slaveversion) != 0) {
        Serial.println("Version mismatch: Slave needs update.");
        return true;
    }

    Serial.println("Versions match: No update needed.");
    return false;
}


bool downloadFirmware() {
    Serial.println("Starting firmware download...");
    
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS.");
        return false;
    }

    // Clear old firmware file
    if (SPIFFS.exists(FIRMWARE_FILE_PATH)) {
        Serial.println("Clearing previous firmware file...");
        SPIFFS.remove(FIRMWARE_FILE_PATH);
    }

    HTTPClient http;
    WiFiClient client;
    
    // Construct full URL - ensure it's properly encoded
    String fullUrl = String(serverUrl) + "/firmware/" + Otarun.firmwareUrl;
    Serial.print("Downloading from: ");
    Serial.println(fullUrl);

    // Add retry logic
    int retryCount = 0;
    const int maxRetries = 3;
    
    while (retryCount < maxRetries) {
        http.end();
        delay(200);
        
        if (!http.begin(client, fullUrl)) {
            Serial.println("Failed to begin HTTP connection");
            retryCount++;
            delay(1000);
            continue;
        }

        // Add some headers if needed
        http.addHeader("Accept", "application/octet-stream");
        
        int httpCode = http.GET();
        if (httpCode != HTTP_CODE_OK) {
            Serial.printf("HTTP error: %d (attempt %d/%d)\n", httpCode, retryCount+1, maxRetries);
            http.end();
            retryCount++;
            delay(1000);
            continue;
        }

        // Get file size
        int firmwareSize = http.getSize();
        if (firmwareSize <= 0) {
            Serial.println("Invalid content length");
            http.end();
            retryCount++;
            delay(1000);
            continue;
        }
        Serial.printf("Firmware size: %d bytes\n", firmwareSize);

        // Open file for writing
        File firmwareFile = SPIFFS.open(FIRMWARE_FILE_PATH, FILE_WRITE);
        if (!firmwareFile) {
            Serial.println("Failed to create firmware file");
            http.end();
            return false;
        }

        // Get stream pointer
        WiFiClient *stream = http.getStreamPtr();

        // Download with progress
        size_t bytesRead = 0;
        unsigned long startTime = millis();
        uint8_t buffer[1024]; // Larger buffer for better performance

        while (http.connected() && bytesRead < firmwareSize) {
            if (millis() - startTime > DOWNLOAD_TIMEOUT) {
                Serial.println("Download timeout");
                firmwareFile.close();
                http.end();
                retryCount++;
                delay(1000);
                break;
            }

            size_t available = stream->available();
            if (available > 0) {
                size_t readSize = stream->readBytes(buffer, min(available, sizeof(buffer)));
                size_t written = firmwareFile.write(buffer, readSize);
                bytesRead += written;
                
                // Print progress every 10%
                static int lastPercent = -1;
                int percent = (bytesRead * 100) / firmwareSize;
                if (percent != lastPercent && percent % 10 == 0) {
                    Serial.printf("Downloaded %d%% (%d/%d bytes)\n", percent, bytesRead, firmwareSize);
                    lastPercent = percent;
                }
            }
            delay(1);
        }

        firmwareFile.close();
        http.end();

        if (bytesRead == firmwareSize) {
            Serial.println("Download completed successfully");
            return true;
        } else {
            Serial.printf("Download incomplete: %d/%d bytes\n", bytesRead, firmwareSize);
            SPIFFS.remove(FIRMWARE_FILE_PATH);
            retryCount++;
            delay(1000);
        }
    }

    Serial.println("Max retries reached, download failed");
    return false;
}

bool sendFirmwareToSlave() {
    Serial.println("Starting firmware transfer via ESP-NOW...");

    if (!setupESPNowpair(Otarun.slaveAddress)) return false;

    File firmwareFile = SPIFFS.open(FIRMWARE_FILE_PATH, "r");
    if (!firmwareFile) {
        Serial.println("Error: Failed to open firmware file.");
        return false;
    }

    size_t firmwareSize = firmwareFile.size();
    size_t totalPackets = (firmwareSize + CHUNK_SIZE - 1) / CHUNK_SIZE;
    Serial.printf("Firmware size: %d bytes, Total packets: %d\n", firmwareSize, totalPackets);

    // Send metadata: magic header + firmware size + total packets
    uint8_t metadata[12];
    uint32_t magicHeader = 0xABCD1234;
    memcpy(metadata, &magicHeader, 4);
    memcpy(metadata + 4, &firmwareSize, 4);
    memcpy(metadata + 8, &totalPackets, 4);

    if (!sendMessage(Otarun.slaveAddress, metadata, sizeof(metadata))) {
        Serial.println("Error: Failed to send firmware metadata.");
        firmwareFile.close();
        return false;
    }

    size_t i = 0;
    while (i < totalPackets) {
        esp_task_wdt_reset();  // Reset watchdog if enabled

        uint8_t packet[CHUNK_SIZE + 4];
        memcpy(packet, &i, 4);  // First 4 bytes = packet index

        firmwareFile.seek(i * CHUNK_SIZE);
        size_t readBytes = firmwareFile.read(packet + 4, CHUNK_SIZE);
        if (readBytes == 0) {
            Serial.printf("Error: Read failed at packet %d\n", i);
            break;
        }

        Serial.printf("Sending packet %d, size: %d, Free heap: %d bytes\n", i, readBytes, ESP.getFreeHeap());

        int retryCount = 0;
        bool success = false;

        while (retryCount < 2) {
            if (sendMessage(Otarun.slaveAddress, packet, readBytes + 4)) {
                success = true;
                break;
            } else {
                Serial.printf("Retrying packet %d... (%d)\n", i, retryCount + 1);
                retryCount++;
                delay(100);  // Increased delay on retry
            }
        }

        if (!success) {
            Serial.printf("Error: Failed to send firmware packet %d after retries.\n", i);
            firmwareFile.close();
            return false;
        }

        i++;
        delay(100);  // Increased delay between packets to reduce pressure
    }

    firmwareFile.close();

   
    uint32_t endSignal = 0xFFFFFFFF;
    esp_now_send(Otarun.slaveAddress, (uint8_t*)&endSignal, sizeof(endSignal));
    Serial.println("Firmware transfer complete. Sending termination signal...");
    return true ;

}

void removeAllESPNOWPeers() {
    esp_now_peer_num_t peerNum;
    esp_err_t result = esp_now_get_peer_num(&peerNum);

    if (result != ESP_OK) {
        Serial.println("Failed to get peer count.");
        return;
    }

    int peerCount = peerNum.total_num;  // Get the total number of peers

    if (peerCount == 0) {
        Serial.println("No ESP-NOW peers found.");
        pair=false ;
        return;
    }

    Serial.printf("Removing %d ESP-NOW peers...\n", peerCount);

    esp_now_peer_info_t peerList[ESP_NOW_MAX_TOTAL_PEER_NUM];

    for (int i = 0; i < peerCount; i++) {
        esp_now_del_peer(peerList[i].peer_addr);
    }

    Serial.println("All ESP-NOW peers removed.");
    pair=false ;
    return ;

}

bool sendversionserver() {
    if (!wificonnect()) {
        Serial.println("Failed to connect to WiFi for version update");
        return false;
    }

    WiFiClient client;
    HTTPClient http;
    http.end();
    delay(200);

    // Create JSON payload
    DynamicJsonDocument doc(256);
    doc["mac"] = String(Otarun.slaveAddress[0], HEX) + ":" + 
                 String(Otarun.slaveAddress[1], HEX) + ":" + 
                 String(Otarun.slaveAddress[2], HEX) + ":" + 
                 String(Otarun.slaveAddress[3], HEX) + ":" + 
                 String(Otarun.slaveAddress[4], HEX) + ":" + 
                 String(Otarun.slaveAddress[5], HEX);
    doc["version"] = Otarun.serverversion;
    doc["status"] = "completed";
    doc["lab"] = "Lab102"; // Replace with your lab number or make it dynamic

    String jsonPayload;
    serializeJson(doc, jsonPayload);

    String url = String(serverUrl) + "/update-ota-status";
    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(jsonPayload);
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.println("Server response: " + payload);
        http.end();
        return true;
    } else {
        Serial.printf("Failed to update OTA status. HTTP Code: %d\n", httpCode);
        http.end();
        return false;
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(2, INPUT_PULLUP);
    if (SPIFFS.begin(true)) {
        Serial.println("SPIFFS Initialized.");
    } else {
        Serial.println("SPIFFS Initialization Failed!");
    }

    if (wificonnect()) {
        Serial.println("WiFi Setup Done.");
    } else {
        Serial.println("WiFi Setup Failed!");
    }

    if (setupespnow()) {
        Serial.println("ESP-NOW Setup Done.");
    } else {
        Serial.println("ESP-NOW Setup Failed!");
    }
}

void loop() {
    delay(500);

    if(batchReady&&sendBatchToDatabase())   //send data to server
    {
        Serial.println("Batch sent successfully.");
    }

    delay(500) ;

    if(!esp_now&&setupespnow()) esp_now=true ;  //set all time in espnow mode

    delay(500) ;

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) 
    {
        previousMillis = currentMillis; // Update previousMillis correctly
        inloop = true;
    }
    else 
    {
        inloop = false;
    }

    if (inloop && checkForOTAUpdate()&&checkversion()&&wificonnect()) 
    {
       Serial.println("OTA Update Requested!");
        if (downloadFirmware()) {
            //sendFirmwareToSlave();
            Serial.println("file is downloaded");
            int i=0 ;
            while(i<4)
            {
            sending=true ;
            if(sendFirmwareToSlave())
            {
                delay(15000) ;
                break ;
            }
            delay(100) ;
            sending=false ;
            i++ ;
            }
            delay(500) ;
            if(!checkversion())  sendversionserver() ;
        }
        else {
          Serial.println("its is not updated") ;
        }

        if (SPIFFS.exists(FIRMWARE_FILE_PATH)) 
        {
          if (SPIFFS.remove(FIRMWARE_FILE_PATH))
          {
            Serial.println("Firmware file deleted successfully.");
          } 
          else 
          {
            Serial.println("Error: Failed to delete firmware file.");
          } 
        } 
        else 
        {
         Serial.println("Firmware file does not exist.");
        }
        delay(1000) ;
    }

    delay(500) ;

    if(!esp_now&&setupespnow()) esp_now=true ;  //set all time in espnow mode
    
    delay(500) ;

    if(pair) removeAllESPNOWPeers() ;  // remove pairs


    delay(500) ;
}
