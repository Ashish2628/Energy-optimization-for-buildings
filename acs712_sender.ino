#include <ESP8266WiFi.h>
#include <espnow.h>
#include <FS.h>
#include <Updater.h>
#include "ACS712.h"

#define FIRMWARE_FILE "/firmware.bin"
#define CHUNK_SIZE 240

ACS712 ACS(A0, 1.035, 1023, 100);  // Pin A0, Vref, ADC max, sensitivity in mV/A

size_t firmwareSize = 0;
size_t totalPackets = 0;
size_t receivedPackets = 0;
bool updateReady = false;
File firmwareFile;

char FIRMWARE_VERSION[10] = "1.0.0";
uint8_t metadata[12];
uint8_t ackMessage = 1;
uint8_t receiverAddress[] = {0x24, 0xdc, 0xc3, 0xae, 0x8b, 0x2c};

typedef struct {
  char sensorType[10];
  int numValues;
  float values[2];
  char sensorID[20];
} SensorData;

SensorData acsData;

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.println(sendStatus == 0 ? "Packet sent successfully" : "Failed to send packet");
}

void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (len < 4) return;
  if (memcmp(mac, receiverAddress, 6) != 0) return;

  if (strncmp((char*)incomingData, "checkversion", len) == 0) {
    Serial.printf("Sending firmware version: '%s' (size: %d bytes)\n", FIRMWARE_VERSION, sizeof(FIRMWARE_VERSION));
    esp_now_send(mac, (uint8_t*)FIRMWARE_VERSION, sizeof(FIRMWARE_VERSION));
    return;
  }

  if (len == sizeof(metadata)) {
    uint32_t receivedMagic;
    memcpy(&receivedMagic, incomingData, 4);

    if (receivedMagic == 0xABCD1234) {
      memcpy(&firmwareSize, incomingData + 4, 4);
      memcpy(&totalPackets, incomingData + 8, 4);
      receivedPackets = 0;

      Serial.println("🚀 START PACKET RECEIVED!");
      Serial.printf("Firmware Size: %u bytes\n", firmwareSize);
      Serial.printf("Total Packets: %u\n", totalPackets);

      firmwareFile = SPIFFS.open(FIRMWARE_FILE, "w");
      if (!firmwareFile) {
        Serial.println("❌ ERROR: Cannot open firmware file for writing!");
        return;
      }

      Serial.println("✅ Firmware file opened for writing.");
      esp_now_send(mac, &ackMessage, sizeof(ackMessage));
      return;
    }
  }

  uint32_t packetIndex;
  memcpy(&packetIndex, incomingData, 4);
  size_t dataSize = len - 4;

  if (packetIndex == 0xFFFFFFFF) {
    Serial.println("✅ Firmware transfer complete. Ready for update.");
    updateReady = true;

    if (firmwareFile) {
      firmwareFile.close();
      Serial.println("📁 Firmware file closed.");
    }

    esp_now_send(mac, &ackMessage, sizeof(ackMessage));
    return;
  }

  if (packetIndex != receivedPackets) {
    Serial.printf("⚠️ Unexpected packet %d, expected %d. Ignoring...\n", packetIndex, receivedPackets);
    return;
  }

  firmwareFile.write(incomingData + 4, dataSize);
  receivedPackets++;

  Serial.printf("✅ Received packet %d, size: %d\n", packetIndex, dataSize);
  esp_now_send(mac, &ackMessage, sizeof(ackMessage));
}

bool applyFirmwareUpdate() {
  if (!SPIFFS.exists(FIRMWARE_FILE)) {
    Serial.println("❌ ERROR: Firmware file does not exist!");
    return false;
  }

  File firmwareFile = SPIFFS.open(FIRMWARE_FILE, "r");
  if (!firmwareFile) {
    Serial.println("❌ ERROR: Failed to open firmware file!");
    return false;
  }

  size_t firmwareSize = firmwareFile.size();
  Serial.printf("🔄 Applying update, size: %d bytes\n", firmwareSize);

  if (!Update.begin(firmwareSize)) {
    Serial.println("❌ ERROR: Update failed to start!");
    return false;
  }

  size_t written = Update.writeStream(firmwareFile);
  firmwareFile.close();

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
  delay(100);
  Serial.println("ACS712 Calibrated Example");

  WiFi.mode(WIFI_STA);
  if (!SPIFFS.begin()) {
    Serial.println("❌ SPIFFS mount failed! Formatting...");
    SPIFFS.format();
    if (!SPIFFS.begin()) {
      Serial.println("❌ SPIFFS mount failed again! Aborting.");
      return;
    }
  }

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  ACS.autoMidPoint();  // Calibrate zero-current point
  Serial.print("MidPoint: ");
  Serial.println(ACS.getMidPoint());
}

void loop() {
  float current_mA = ACS.mA_AC_sampling();

  if (current_mA < 20) current_mA = 0;  // Noise filtering

  float current_A = current_mA / 1000.0;
  float voltage = 230.0;
  float powerFactor = 0.9;
  float power_W = voltage * current_A * powerFactor;

  strcpy(acsData.sensorType, "ACS712");
  strcpy(acsData.sensorID, "ACS712100112202428");
  acsData.numValues = 2;
  acsData.values[0] = current_A;
  acsData.values[1] = power_W;

  esp_now_send(receiverAddress, (uint8_t*)&acsData, sizeof(acsData));

  Serial.print("Current (mA): ");
  Serial.print(current_mA);
  Serial.print(" | Power (W): ");
  Serial.println(power_W);
  Serial.println("--------------------");

  if (updateReady) {
    applyFirmwareUpdate();
  }

  delay(15000);
}
