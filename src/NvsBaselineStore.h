#pragma once

#include <PersonalBaseline.h>
#include <Preferences.h>
#include <nvs.h>

class NvsBaselineStore : public personal::Store
{
public:
    bool begin() { opened_ = preferences_.begin("sutra-baseline", false); return opened_; }

    personal::ReadResult read(unsigned slot, personal::Record &record) override
    {
        if (!opened_ || slot > 1) return personal::ReadResult::ERROR;
        // Read through NVS directly to distinguish a missing key from an I/O
        // error. Preferences' isKey() reports false for both cases.
        nvs_handle_t handle;
        if (nvs_open("sutra-baseline", NVS_READONLY, &handle) != ESP_OK)
            return personal::ReadResult::ERROR;
        size_t size = sizeof(record);
        esp_err_t result = nvs_get_blob(handle, key(slot), &record, &size);
        nvs_close(handle);
        if (result == ESP_ERR_NVS_NOT_FOUND) return personal::ReadResult::MISSING;
        return result == ESP_OK && size == sizeof(record)
            ? personal::ReadResult::FOUND : personal::ReadResult::ERROR;
    }

    bool write(unsigned slot, const personal::Record &record) override
    {
        // putBytes commits the whole record, once per completed session.
        return opened_ && slot < 2 &&
            preferences_.putBytes(key(slot), &record, sizeof(record)) == sizeof(record);
    }
private:
    Preferences preferences_;
    bool opened_ = false;
    static const char *key(unsigned slot) { return slot == 0 ? "history0" : "history1"; }
};
