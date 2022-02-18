#include <Arduino.h>
#include <BH1750.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <SHTSensor.h>
#include <Wire.h>
#include <esp_sleep.h>

#include <cmath>

#include "config.h"
#include "misc.h"

#define I2C_SDA 4
#define I2C_SCL 3

const BLEUUID kEnvironmentalSensorServiceUUID =
    BLEUUID(static_cast<uint16_t>(0x181A));
const BLEUUID kTemperatureUUID = BLEUUID(static_cast<uint16_t>(0x2A6E));
const BLEUUID kHumidityUUID = BLEUUID(static_cast<uint16_t>(0x2A6F));
const BLEUUID kIlluminanceUUID = BLEUUID(static_cast<uint16_t>(0x2AFB));
const uint16_t kTemperatureDefault = 0x8000;
const uint16_t kHumidityDefault = 0xFFFF;
const uint8_t kIlluminanceDefault[3] = {0xFF, 0xFF, 0xFF};
const uint8_t kSHTAddress = 0x44;
const uint8_t kLightAddress = 0x23;

SHTSensor sht(SHTSensor::SHT3X);
BH1750 light_meter(kLightAddress);
BLECharacteristic *pTemperature = nullptr;
BLECharacteristic *pHumidity = nullptr;
BLECharacteristic *pIlluminance = nullptr;
bool connect = false;
uint32_t status_change = 0;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) {
        Serial.println("A device connected!");
        connect = true;
        status_change = millis();
    }

    void onDisconnect(BLEServer *pServer) {
        Serial.println("A device is disconnected!");
        connect = false;
        status_change = millis();
        Serial.println("Going to sleep now because disconnect.");
        esp_deep_sleep_start();
    }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic,
                esp_ble_gatts_cb_param_t *param) {
        Serial.println("A device read the data.");
    }
    void onStatus(BLECharacteristic *pCharacteristic, Status s, uint32_t code) {
        if (s == BLECharacteristicCallbacks::SUCCESS_INDICATE) {
            Serial.println("A device has received the data.");
        }
    }
};

BLECharacteristic *CharacteristicSetup(BLEService *pService, BLEUUID uuid,
                                       uint16_t default_value) {
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        uuid, BLECharacteristic::PROPERTY_READ |
                  BLECharacteristic::PROPERTY_INDICATE);
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pCharacteristic->setValue(default_value);
    return pCharacteristic;
}

BLECharacteristic *CharacteristicSetup(BLEService *pService, BLEUUID uuid,
                                       uint8_t *pDefaultValue, size_t length) {
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        uuid, BLECharacteristic::PROPERTY_READ |
                  BLECharacteristic::PROPERTY_INDICATE);
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pCharacteristic->setValue(pDefaultValue, length);
    return pCharacteristic;
}

BLECharacteristic *CharacteristicSetup(BLEService *pService, BLEUUID uuid,
                                       const uint8_t *pDefaultValue,
                                       const size_t length) {
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        uuid, BLECharacteristic::PROPERTY_READ |
                  BLECharacteristic::PROPERTY_INDICATE);
    pCharacteristic->addDescriptor(new BLE2902());
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    uint8_t pValue[length] = {0};
    for (int i = 0; i < length; ++i) {
        pValue[i] = pDefaultValue[i];
    }
    pCharacteristic->setValue(pValue, length);
    return pCharacteristic;
}
void BluetoothSetup() {
    Serial.println("Start setup BLE.");
    BLEDevice::init("ESP32BLE");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService =
        pServer->createService(kEnvironmentalSensorServiceUUID);
    pTemperature =
        CharacteristicSetup(pService, kTemperatureUUID, kTemperatureDefault);
    pHumidity = CharacteristicSetup(pService, kHumidityUUID, kHumidityDefault);
    pIlluminance =
        CharacteristicSetup(pService, kIlluminanceUUID, kIlluminanceDefault, 3);
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(
        0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
}

void DeepSleepSetup(uint16_t seconds) {
    std::string message = GetWakeUpReason();
    Serial.println(message.c_str());
    esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
    Serial.printf("Setup ESP32 to wake up after %d seconds\n", seconds);
}

void ScanI2CDevices() {
    int device_number = 0;
    for (int address = 0; address < 127; ++address) {
        Wire.beginTransmission(address);
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            Serial.printf("I2C device found at address 0x%02x", address);
            Serial.println();
            ++device_number;
        } else if (error == 4) {
            Serial.printf("Unknow error at address 0x%02x", address);
            Serial.println();
        }
    }
    if (device_number == 0) {
        Serial.println("No I2C devices found.");
    } else {
        Serial.println("I2C scan done.");
    }
}

void SensorsSetup() {
    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    // ScanI2CDevices();
    sht.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH);
    if (sht.init()) {
        Serial.println("SHT Sensor init success.");
    } else {
        Serial.println("SHT Sensor init fail.");
    }
    if (light_meter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
        Serial.println("BH1750 init success.");
    } else {
        Serial.println("BH1750 init fail.");
    }
}

void setup() {
    Serial.begin(115200);
    SensorsSetup();
    BluetoothSetup();
    DeepSleepSetup(WAKEUP_INTERVAL);
    Serial.println("ESP32BLE setup done!");
}

void UpdateSHT(float &temperature, bool &temperature_valid, float &humidity,
               bool &humidity_valid) {
    if (sht.readSample()) {
        temperature = sht.getTemperature();
        temperature_valid = true;
        humidity = sht.getHumidity();
        humidity_valid = true;
        Serial.printf("Temperature is %.2f °C, humidity is %.2f%", temperature,
                      humidity);
        Serial.println();
    } else {
        temperature_valid = false;
        humidity_valid = false;
    }
}

void UpdateBH1750(float &illuminance, bool &illuminance_valid) {
    light_meter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
    while (!light_meter.measurementReady(true)) {
        delay(120);
    }
    float lux = light_meter.readLightLevel();
    if ((lux == -1) | (lux == -2)) {
        illuminance_valid = false;
    } else {
        illuminance = lux;
        illuminance_valid = true;
        Serial.printf("Illuminance is %.2f lux", illuminance);
        Serial.println();
    }
}

void PublishTemperature(BLECharacteristic *pCharacteristic, float temperature,
                        bool valid = true) {
    uint16_t value = kTemperatureDefault;
    if (valid) {
        value = round(temperature * 100);
    }
    pCharacteristic->setValue(value);
    Serial.print("Temperature is update to ");
    Serial.print(value);
    Serial.println();
    pCharacteristic->indicate();
}

void PublishHumidity(BLECharacteristic *pCharacteristic, float humidity,
                     bool valid = true) {
    uint16_t value = kHumidityDefault;
    if (valid) {
        value = round(humidity * 100);
    }
    pCharacteristic->setValue(value);
    Serial.print("Humidity is update to ");
    Serial.print(value);
    Serial.println();
    pCharacteristic->indicate();
}

void PublishIlluminance(BLECharacteristic *pCharacteristic, float illuminance,
                        bool valid = true) {
    uint8_t pValue[3] = {0};
    for (int i = 0; i < 3; ++i) {
        pValue[i] = kIlluminanceDefault[i];
    }
    if (valid) {
        uint32_t value = round(illuminance * 100);
        pValue[0] = value;
        pValue[1] = value >> 8;
        pValue[2] = value >> 16;
    }
    pCharacteristic->setValue(pValue, 3);
    Serial.printf("Illuminace is update to 0x%x-%x-%x", pValue[0], pValue[1],
                  pValue[2]);
    Serial.println();
    pCharacteristic->indicate();
}

void loop() {
    uint32_t now = millis();
    // connect = true;  // For debug to disable deep sleep
    if ((!connect) &&
        (static_cast<uint32_t>(now - status_change) > IDLE_LIMIT * 1000U)) {
        Serial.println("Going to sleep now because idle");
        esp_deep_sleep_start();
    }
    if (connect) {
        static uint32_t last_update = 0;
        if ((static_cast<uint32_t>(now - last_update) >
             UPDATE_INTERVAL * 1000U) ||
            (last_update == 0)) {
            float temperature, humidity, illuminace = 0;
            bool temperature_valid, humidity_valid, illuminace_valid = false;
            UpdateSHT(temperature, temperature_valid, humidity, humidity_valid);
            UpdateBH1750(illuminace, illuminace_valid);
            PublishTemperature(pTemperature, temperature, temperature_valid);
            PublishHumidity(pHumidity, humidity, humidity_valid);
            PublishIlluminance(pIlluminance, illuminace, illuminace_valid);
            last_update = now;
        }
    }
}