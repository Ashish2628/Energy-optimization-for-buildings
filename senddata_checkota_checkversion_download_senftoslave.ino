#include <WiFi.h>
#include <esp_now.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

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
const char* otaUrl = "http://172.25.2.156:3000/firmwareuse";
const char* serverUrl = "http://172.25.2.156:3000";
const char *mongoDBEndpoint = "http://172.25.2.156:3000/sensor-data-batch";
bool sending=false ;


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
    Serial.println(sendStatus == ESP_NOW_SEND_SUCCESS ? "Packet sent successfully" : "Failed to send packet");
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
    esp_now=false ;
    WiFi.begin(ssid, password);
    int i = 0;
    while (WiFi.status() != WL_CONNECTED && i <= 20) {
        delay(500);
        Serial.print(".");
        i++;
    }
    Serial.println();
    return WiFi.status() == WL_CONNECTED;
}

bool checkForOTAUpdate() {
    if (!wificonnect()) return false;

    WiFiClient client;
    HTTPClient http;
    http.end() ;
    delay(200) ;
    Serial.println("Checking OTA Request...");

    const char* labNumber = "Lab102";
    String requestUrl = String(serverUrl) + "/getData?lab=" + String(labNumber);
    
    if (!http.begin(client, requestUrl)) {
        Serial.println("Failed to connect to server.");
        client.stop();  // Explicitly stop WiFiClient
        return false;
    }

    int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("HTTP Request failed. Code: %d\n", httpCode);
        http.end();
        client.stop();  // Stop WiFiClient before returning
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

    // Increase buffer size for JSON parsing
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.println("JSON Parse Error!");
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

    String serverOtaVersion = doc["ota"].as<String>();
    // Extract MAC address from JSON string
    String macStr = doc["mac"].as<String>();
    if (macStr.length() == 17) {  // MAC addresses are always 17 characters (XX:XX:XX:XX:XX:XX)
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


    strncpy(Otarun.serverversion, serverOtaVersion.c_str(), sizeof(Otarun.serverversion) - 1);
    Otarun.serverversion[sizeof(Otarun.serverversion) - 1] = '\0';

    Serial.print("Stored OTA Version: ");
    Serial.println(Otarun.serverversion);

    http.end();
    client.stop();  // Stop WiFiClient before returning
    return (Otarun.serverversion[0] != '\0');  
}

bool setupESPNowpair(uint8_t PairAddress[]) { 
    esp_now = true ;
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

bool checkversion() 
  {   
    if (!setupESPNowpair(Otarun.slaveAddress)) return false;
    // Send OTA version to slave
    const char* message = "checkversion";
    esp_err_t result = esp_now_send(Otarun.slaveAddress, (uint8_t*)message, strlen(message) + 1);
    if (result != ESP_OK) {
        Serial.println("Error: Failed to send OTA version.");
        return false;
    }

    // Wait for slave response
    bool sver=false ;
      unsigned long startTime = millis();
      while (millis() - startTime < 2000) { // Wait max 2 seconds 
        delay(100);

        // Check if Otarun.slaveversion is updated (e.g., not empty or contains valid data)
        if (Otarun.slaveversion[0] != '\0') {
            sver=true ;
            break;  // Exit the loop if data is received
        }
    }
    
    if(!sver) 
    {  
      Serial.printf("slave version not recived") ;
       return false ;
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

    // ✅ Clear old firmware file
    if (SPIFFS.exists(FIRMWARE_FILE_PATH)) {
        Serial.println("Clearing previous firmware file...");
        SPIFFS.remove(FIRMWARE_FILE_PATH);
    }

    HTTPClient http;
    WiFiClient client;
    http.end();
    delay(200);
    http.begin(client, otaUrl);
    int httpCode = http.GET();

    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("Firmware download failed. HTTP Code: %d\n", httpCode);
        http.end();
        return false;
    }

    int firmwareSize = http.getSize();
    Serial.printf("Firmware size: %d bytes\n", firmwareSize);

    if (firmwareSize <= 0) {
        Serial.println("Error: Invalid firmware size.");
        http.end();
        return false;
    }

    // ✅ Open new file after clearing the old one
    File firmwareFile = SPIFFS.open(FIRMWARE_FILE_PATH, FILE_WRITE);
    if (!firmwareFile) {
        Serial.println("Error opening firmware file.");
        http.end();
        return false;
    }

    WiFiClient *stream = http.getStreamPtr();
    size_t bytesRead = 0;
    unsigned long startTime = millis();

    while (http.connected() && bytesRead < firmwareSize) {
        if (millis() - startTime > DOWNLOAD_TIMEOUT) {
            Serial.println("Error: Download timeout.");
            firmwareFile.close();
            http.end();
            return false;
        }

        if (stream->available()) {
            uint8_t buffer[CHUNK_SIZE];
            size_t len = stream->readBytes(buffer, CHUNK_SIZE);
            firmwareFile.write(buffer, len);
            bytesRead += len;
            Serial.printf("Downloaded %d/%d bytes...\n", bytesRead, firmwareSize);
        }
        delay(10);
    }

    firmwareFile.close();
    Serial.printf("Firmware downloaded successfully (%d bytes)\n", bytesRead);
    http.end();
    return (bytesRead == firmwareSize);
}

void sendFirmwareToSlave() {
    Serial.println("Starting firmware transfer via ESP-NOW...");
     if (!setupESPNowpair(Otarun.slaveAddress)) return ;
    File firmwareFile = SPIFFS.open(FIRMWARE_FILE_PATH, "r");
    if (!firmwareFile) {
        Serial.println("Error: Failed to open firmware file.");
        return;
    }

    size_t firmwareSize = firmwareFile.size();
    size_t totalPackets = (firmwareSize + CHUNK_SIZE - 1) / CHUNK_SIZE;
    Serial.printf("Firmware size: %d bytes, Total packets: %d\n", firmwareSize, totalPackets);

    uint8_t metadata[12];
    uint32_t magicHeader = 0xABCD1234;  // Unique identifier
   memcpy(metadata, &magicHeader, 4);  // Add magic header
   memcpy(metadata + 4, &firmwareSize, 4);  
   memcpy(metadata + 8, &totalPackets, 4);  

    if (!sendMessage(Otarun.slaveAddress, metadata, sizeof(metadata))) {
        Serial.println("Error: Failed to send firmware metadata.");
        firmwareFile.close();
        return;
    }

    size_t i = 0;
    while (i < totalPackets) {
        uint8_t packet[CHUNK_SIZE + 4];
        memcpy(packet, &i, 4);

        firmwareFile.seek(i * CHUNK_SIZE);
        size_t readBytes = firmwareFile.read(packet + 4, CHUNK_SIZE);
        if (readBytes == 0) break;

        Serial.printf("Sending packet %d, size: %d\n", i, readBytes);

        int retryCount = 0;
        bool success = false;

        while (retryCount < 2) {  // Retry up to 2 times
            if (sendMessage(Otarun.slaveAddress, packet, readBytes + 4)) {
                success = true;
                break;
            } else {
                Serial.printf("Retrying packet %d... (%d)\n", i, retryCount + 1);
                retryCount++;
                delay(50);
            }
        }

        if (!success) {
            Serial.printf("Error: Failed to send firmware packet %d after retries.\n", i);
            firmwareFile.close();
            return;
        }

        i++;  
        delay(50);
    }

    firmwareFile.close();

    Serial.println("Firmware transfer complete. Sending termination signal...");
    uint32_t endSignal = 0xFFFFFFFF;
    esp_now_send(Otarun.slaveAddress, (uint8_t*)&endSignal, 4);
}


void setup() {
    Serial.begin(115200);

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
            sending=true ;
            sendFirmwareToSlave();
            sending=false ;
            //checkversion() ;
            //sendversionserver() ;
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
}
