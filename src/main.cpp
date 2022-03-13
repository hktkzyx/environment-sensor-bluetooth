#include <Arduino.h>
#include <BH1750.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <SHTSensor.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>

#include <cmath>

#include "config.h"
#include "misc.h"

#define COUNT_OF(x) \
    ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))
#define I2C_SDA 4
#define I2C_SCL 3

const uint8_t kSHTAddress = 0x44;
const uint8_t kLightAddress = 0x23;

uint32_t advertising_last_on = 0;

void UpdateSHT(float &temperature, float &humidity) {
    SHTSensor sht(SHTSensor::SHT3X);
    sht.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH);
    if (sht.init()) {
        Serial.println("SHT Sensor init success.");
    } else {
        Serial.println("SHT Sensor init fail.");
    }
    if (sht.readSample()) {
        temperature = sht.getTemperature();
        humidity = sht.getHumidity();
        Serial.printf("Temperature is %.2f °C, humidity is %.2f%", temperature,
                      humidity);
        Serial.println();
    } else {
        temperature = -300;
        humidity = -1;
    }
}

void UpdateBH1750(float &illuminance) {
    BH1750 light_meter(kLightAddress);
    if (light_meter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
        Serial.println("BH1750 init success.");
    } else {
        Serial.println("BH1750 init fail.");
    }
    while (!light_meter.measurementReady(true)) {
        delay(120);
    }
    float lux = light_meter.readLightLevel();
    if ((lux == -1) | (lux == -2)) {
        illuminance = -1;
    } else {
        illuminance = lux;
        Serial.printf("Illuminance is %.2f lux", illuminance);
        Serial.println();
    }
}

std::string GenerateServiceData(const float &temperature, const float &humidity,
                                const float &illuminance) {
    uint16_t temperature_data = 0x8000;
    if (temperature <= 327.67 && temperature >= -273.15) {
        temperature_data = round(temperature * 100);
    }
    uint16_t humidity_data = 0xFFFF;
    if (humidity >= 0 && humidity <= 100) {
        humidity_data = round(humidity * 100);
    }
    uint32_t illuminance_data = 0x00FFFFFF;
    if (illuminance >= 0 && illuminance <= 167772.14) {
        illuminance_data = round(illuminance * 100);
    }
    return std::string(reinterpret_cast<char *>(&temperature_data), 2) +
           std::string(reinterpret_cast<char *>(&humidity_data), 2) +
           std::string(reinterpret_cast<char *>(&illuminance_data), 3);
}

std::string GenerateServiceData() { return GenerateServiceData(-300, -1, -1); }

BLEAdvertisementData GenerateAdvertisedData(uint8_t *pManufacturer,
                                            const size_t manufacturer_size,
                                            BLEUUID &service_uuid,
                                            std::string &service_data) {
    BLEAdvertisementData data;
    data.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    std::string manufacturer(reinterpret_cast<char *>(pManufacturer),
                             manufacturer_size);
    data.setManufacturerData(manufacturer);
    data.setServiceData(service_uuid, service_data);
    return data;
}

void Publish(const uint32_t millis_interval) {
    static uint32_t last_publish = -1;
    uint32_t now = millis();
    if (static_cast<uint32_t>(now - last_publish) > millis_interval ||
        last_publish == -1) {
        last_publish = now;
        float temperature = -300, humidity = -1, illuminance = -1;
        UpdateSHT(temperature, humidity);
        UpdateBH1750(illuminance);
        uint8_t manufacturer_id[3] = {
            0xE5, 0x02, 0x66};  // Espressif 0x02E5, self-defined 0x66
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        BLEUUID service_uuid(
            static_cast<uint16_t>(0xFC00));  // Self-defined 0xFC00
        std::string service_data =
            GenerateServiceData(temperature, humidity, illuminance);
        BLEAdvertisementData data = GenerateAdvertisedData(
            manufacturer_id, 3, service_uuid, service_data);
        pAdvertising->setAdvertisementData(data);
        BLEDevice::startAdvertising();
        advertising_last_on = millis();
    }
}

void TurnOffAdvertising(const uint32_t millis_duration,
                        bool deep_sleep = true) {
    if (static_cast<uint32_t>(millis() - advertising_last_on) >
        millis_duration) {
        BLEDevice::stopAdvertising();
        if (deep_sleep) {
            esp_deep_sleep_start();
        }
    }
}

void DeepSleepSetup(const uint32_t millis_deep_sleep_duration) {
    std::string message = GetWakeUpReason();
    Serial.println(message.c_str());
    esp_sleep_enable_timer_wakeup(millis_deep_sleep_duration * 1000ULL);
    Serial.printf("Setup ESP32 to wake up after %d milliseconds\n",
                  millis_deep_sleep_duration);
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

void setup() {
    esp_task_wdt_init(1000 * WATCHDOG_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    // ScanI2CDevices();
    DeepSleepSetup(1000 * DEEP_SLEEP_DURATION);
    BLEDevice::init(DEVICE_NAME);
    Serial.println("Setup done!");
}

void WatchdogReset(const uint32_t interval) {
    static uint32_t last_reset = 0;
    uint32_t now = millis();
    if (static_cast<uint32_t>(now - last_reset) >= interval) {
        esp_task_wdt_reset();
        Serial.println("Reset watchdog done");
        last_reset = now;
    }
}

void loop() {
    WatchdogReset(1000 * WATCHDOG_RESET_INTERVAL);
    Publish(1000 * PUBLISH_INTERVAL);
    TurnOffAdvertising(1000 * PUBLISH_DURATION);
}
