#include "misc.h"

#include <esp_sleep.h>
std::string GetWakeUpReason() {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    std::string message;
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            message = "Wakeup caused by external signal using RTC_IO";
            break;
        case ESP_SLEEP_WAKEUP_EXT1:
            message = "Wakeup caused by external signal using RTC_CNTL";
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            message = "Wakeup caused by timer";
            break;
        case ESP_SLEEP_WAKEUP_TOUCHPAD:
            message = "Wakeup caused by touchpad";
            break;
        case ESP_SLEEP_WAKEUP_ULP:
            message = "Wakeup caused by ULP program";
            break;
        default:
            message = "Wakeup was not caused by deep sleep";
            break;
    }
    return message;
}
