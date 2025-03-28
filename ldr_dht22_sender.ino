#include <ESP8266WiFi.h>
#include <espnow.h>
#include <FS.h>
#include <Updater.h>


#include "DHT.h"
#define FIRMWARE_FILE "/firmware.bin"
#define CHUNK_SIZE 240  // ESP-NOW max payload size
#define DHTPIN D4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
int ldrPin = A0;


size_t firmwareSize = 0; 
size_t totalPackets = 0;
size_t receivedPackets = 0;
bool updateReady = false;
File firmwareFile;
char FIRMWARE_VERSION[10] = "1.0.0";  //current version
uint8_t metadata[12];
uint8_t ackMessage = 1;  // Simple ACK response
uint8_t masterAddress[] = {0x24, 0xDC, 0xC3, 0xAE, 0x8B, 0x2C}; 
// Structure for sending sensor data


typedef struct {
    char sensorType[10];  // Sensor type (e.g., LDR, DHT)
    int numValues;        // Number of values sent
    float values[2];      // Sensor data values
    char sensorID[20];    // Unique sensor ID
} SensorData;

// Sensor data to send
SensorData dataToSend;

// Callback for data sent status
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
    Serial.println(sendStatus == 0 ? "Packet sent successfully" : "Failed to send packet");
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
    if (len < 4) return;
    if (memcmp(mac, masterAddress, 6) != 0) return;

    // 🔹 If master requests firmware version
    if (strncmp((char*)incomingData, "checkversion", len) == 0) {
        Serial.printf("Sending firmware version: '%s' (size: %d bytes)\n", FIRMWARE_VERSION, sizeof(FIRMWARE_VERSION));
        esp_now_send(mac, (uint8_t*)FIRMWARE_VERSION, sizeof(FIRMWARE_VERSION));  
        return;
    }

    // 🔹 If receiving metadata (firmware size + total packets)
    if (len == sizeof(metadata)) {  
        uint32_t receivedMagic;
        memcpy(&receivedMagic, incomingData, 4);

        if (receivedMagic == 0xABCD1234) {  // Validate magic header
            memcpy(&firmwareSize, incomingData + 4, 4);
            memcpy(&totalPackets, incomingData + 8, 4);
            receivedPackets = 0;  // Reset received packets counter

            Serial.println("🚀 START PACKET RECEIVED!");
            Serial.printf("Firmware Size: %u bytes\n", firmwareSize);
            Serial.printf("Total Packets: %u\n", totalPackets); 

            // Open firmware file for writing (overwrite any existing one)
            firmwareFile = SPIFFS.open(FIRMWARE_FILE, "w");
            if (!firmwareFile) {
                Serial.println("❌ ERROR: Cannot open firmware file for writing!");
                return;
            }
            Serial.println("✅ Firmware file opened for writing.");

            esp_now_send(mac, &ackMessage, sizeof(ackMessage));  // Send ACK
            return;
        }
    }

    // 🔹 Read packet index
    uint32_t packetIndex;
    memcpy(&packetIndex, incomingData, 4);
    size_t dataSize = len - 4;

    // 🔹 If end signal is received
    if (packetIndex == 0xFFFFFFFF) {
        Serial.println("✅ Firmware transfer complete. Ready for update.");
        updateReady = true;
        
        if (firmwareFile) {
            firmwareFile.close();
            Serial.println("📁 Firmware file closed.");
        }

        esp_now_send(mac, &ackMessage, sizeof(ackMessage));  // Send ACK
        return;
    }

    // 🔹 If wrong packet sequence, ignore it
    if (packetIndex != receivedPackets) {
        Serial.printf("⚠️ Unexpected packet %d, expected %d. Ignoring...\n", packetIndex, receivedPackets);
        return;
    }

    // 🔹 Write firmware data to SPIFFS
    firmwareFile.write(incomingData + 4, dataSize);
    receivedPackets++;

    Serial.printf("✅ Received packet %d, size: %d\n", packetIndex, dataSize);
    esp_now_send(mac, &ackMessage, sizeof(ackMessage));  // Send ACK
}


bool applyFirmwareUpdate() {
    // 🔹 Check if firmware file exists
    if (!SPIFFS.exists(FIRMWARE_FILE)) {
        Serial.println("❌ ERROR: Firmware file does not exist!");
        return false;
    }

    // 🔹 Open firmware file for reading
    File firmwareFile = SPIFFS.open(FIRMWARE_FILE, "r");
    if (!firmwareFile) {
        Serial.println("❌ ERROR: Failed to open firmware file!");
        return false;
    }

    size_t firmwareSize = firmwareFile.size();
    Serial.printf("🔄 Applying update, size: %d bytes\n", firmwareSize);

    // 🔹 Begin update process
    if (!Update.begin(firmwareSize)) {
        Serial.println("❌ ERROR: Update failed to start!");
        return false;
    }

    size_t written = Update.writeStream(firmwareFile);
    firmwareFile.close();

    // 🔹 Validate firmware write
    if (written != firmwareSize) {
        Serial.println("❌ ERROR: Firmware write failed!");
        return false;
    }

    if (!Update.end()) {
        Serial.println("❌ ERROR: Update error!");
        return false;
    }

    Serial.println("✅ Update successful! Restarting...");
    ESP.restart();
    return true;
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA); // Set Wi-Fi to station mode

    // Initialize DHT sensor
    dht.begin();
     
    pinMode(ldrPin, INPUT);

    if (!SPIFFS.begin()) {
        Serial.println("❌ SPIFFS mount failed! Formatting...");
        SPIFFS.format();
        if (!SPIFFS.begin()) {
            Serial.println("❌ SPIFFS mount failed again! Aborting.");
            return;
        }
    }

    // Initialize ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);  // Registering send callback
    esp_now_add_peer(masterAddress, ESP_NOW_ROLE_CONTROLLER, 1, NULL, 0);

    Serial.println("✅ ESP-NOW Ready!");

}

void loop() {
    // Read and send LDR data
    int ldrValue = analogRead(ldrPin); // Read the LDR value
    strcpy(dataToSend.sensorType, "LDR");
    strcpy(dataToSend.sensorID, "LDR100112202428"); // Unique ID for LDR sensor
    dataToSend.numValues = 1;
    dataToSend.values[0] = ldrValue;

    if (esp_now_send(masterAddress, (uint8_t *)&dataToSend, sizeof(dataToSend)) == 0) {
        Serial.printf("LDR Value Sent: %d\n", ldrValue);
    } else {
        Serial.println("Failed to send LDR data!");
    }

    delay(1000);

    // Read and send DHT data
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT sensor!");
    } else {
        strcpy(dataToSend.sensorType, "DHT");
        strcpy(dataToSend.sensorID, "DHT100112202428"); // Unique ID for DHT sensor
        dataToSend.numValues = 2;
        dataToSend.values[0] = temperature;
        dataToSend.values[1] = humidity;

        if (esp_now_send(masterAddress, (uint8_t *)&dataToSend, sizeof(dataToSend)) == 0) {
            Serial.printf("DHT Values Sent - Temperature: %.2f, Humidity: %.2f\n", temperature, humidity);
        } else {
            Serial.println("Failed to send DHT data!");
        }
    }

    delay(1000) ;
       if (updateReady) {
        applyFirmwareUpdate();
    }

    delay(15000);
}
