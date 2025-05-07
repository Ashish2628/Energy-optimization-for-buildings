#include <ESP8266WiFi.h>
#include <espnow.h>
#include <FS.h>
#include <Updater.h>

// Pin definitions
#define DOOR_SENSOR_PIN D1
#define WINDOW_SENSOR_PIN D3

// Structure for sending sensor data
typedef struct {
    char sensorType[10]; // Type of sensor
    int numValues;       // Number of values sent
    float values[2];     // Array to hold sensor values
    char sensorID[20];   // Unique sensor ID as a char array
} SensorData;

// Replace with the receiver's MAC address
uint8_t receiverAddress[] = {0x24, 0xDC, 0xC3, 0xAE, 0x8B, 0x2C};

// Sensor data to send
SensorData dataToSend;

// Callback for data sent status
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
    Serial.print("Packet send status: ");
    Serial.println(sendStatus == 0 ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA); // Set Wi-Fi to station mode

    pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
    pinMode(WINDOW_SENSOR_PIN, INPUT_PULLUP);

    // Initialize ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(OnDataSent);

    // Add peer device
    if (esp_now_add_peer(receiverAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0) {
        Serial.println("Failed to add peer!");
        return;
    }

    Serial.println("ESP-NOW setup complete!");
}

void loop() {
    // Read door sensor data
    int doorStatus = digitalRead(DOOR_SENSOR_PIN);

    // Prepare door data to send
    strcpy(dataToSend.sensorType, "DOORWIN");
    strcpy(dataToSend.sensorID, "DOORWIN100112202428"); // Unique ID for door sensor
    dataToSend.numValues = 1;
    dataToSend.values[0] = doorStatus; // Door status

    // Send door data via ESP-NOW
    if (esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend)) == 0) {
        Serial.printf("Door Data Sent - Door: %s\n",
                      doorStatus == HIGH ? "Open" : "Closed");
    } else {
        Serial.println("Failed to send Door data!");
    }

    delay(1000); // Delay before sending window data

    // Read window sensor data
    int windowStatus = digitalRead(WINDOW_SENSOR_PIN);

    // Prepare window data to send
    strcpy(dataToSend.sensorType, "DOORWIN");
    strcpy(dataToSend.sensorID, "DOORWIN100212202428"); // Unique ID for window sensor
    dataToSend.numValues = 1;
    dataToSend.values[0] = windowStatus; // Window status

    // Send window data via ESP-NOW
    if (esp_now_send(receiverAddress, (uint8_t *)&dataToSend, sizeof(dataToSend)) == 0) {
        Serial.printf("Window Data Sent - Window: %s\n",
                      windowStatus == HIGH ? "Open" : "Closed");
    } else {
        Serial.println("Failed to send Window data!");
    }

    delay(15000); // Delay before next cycle
}
