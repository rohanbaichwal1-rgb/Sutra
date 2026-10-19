#include "BleTelemetry.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <atomic>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#ifndef SUTRA_BLE_ENABLED
#define SUTRA_BLE_ENABLED 1
#endif

namespace telemetry {
namespace {

constexpr const char *SERVICE_UUID =
    "7d2a0000-8f5a-4b10-9b6d-6f7574726100";

// Phone -> ESP32
constexpr const char *WRITE_UUID =
    "7d2a01fe-8f5a-4b10-9b6d-6f7574726100";

// ESP32 -> Phone
constexpr const char *STREAM_UUID =
    "7d2a01ff-8f5a-4b10-9b6d-6f7574726100";

constexpr unsigned VALUE_COUNT =
    static_cast<unsigned>(Group::COUNT);

constexpr unsigned SELECTED_READ_GROUP =
    static_cast<unsigned>(Group::LMS);

QueueHandle_t snapshots = nullptr;

std::atomic<bool> ready{false};
std::atomic<bool> connected{false};
std::atomic<bool> restartAdvertising{false};
std::atomic<bool> failed{false};

BLECharacteristic *values[VALUE_COUNT] = {};

BLECharacteristic *stream = nullptr;
BLECharacteristic *terminalWrite = nullptr;

BLE2902 *subscription = nullptr;

std::atomic<uint16_t> peerMtu{23};

// BLE deliberately publishes a compact selection. The full group formatter in
// TelemetryReport remains the Serial Monitor formatter.
class BleLine {
public:
    BleLine(char *buffer, size_t capacity) : buffer_(buffer), capacity_(capacity) {
        if (capacity_) buffer_[0] = 0;
    }

    void append(const char *format, ...) {
        if (!ok_ || used_ >= capacity_) { ok_ = false; return; }
        va_list args;
        va_start(args, format);
        const int count = vsnprintf(buffer_ + used_, capacity_ - used_, format, args);
        va_end(args);
        if (count < 0 || static_cast<size_t>(count) >= capacity_ - used_) {
            ok_ = false;
            return;
        }
        used_ += static_cast<size_t>(count);
    }

    void value(float number, const char *unit, unsigned decimals = 1) {
        if (isfinite(number)) append("%.*f%s", static_cast<int>(decimals), number, unit);
        else append("--");
    }

    bool finish() {
        append("\r\n");
        if (!ok_ && capacity_) buffer_[0] = 0;
        return ok_;
    }

private:
    char *buffer_;
    size_t capacity_;
    size_t used_ = 0;
    bool ok_ = true;
};

void p1Counter(BleLine &line, uint32_t elapsedMs, float reference) {
    if (isfinite(reference))
        line.append("%lu/%lus", static_cast<unsigned long>(elapsedMs / 1000UL),
                    static_cast<unsigned long>(rmssd::P1_HOLD_MS / 1000UL));
    else
        line.append("--");
}

bool formatBleSelection(const Snapshot &snapshot, char *output, size_t capacity) {
    const float sessionDev =
    (isfinite(snapshot.rmssd) &&
     isfinite(snapshot.session) &&
     snapshot.session > 0.0f)
        ? rmssd::Monitor::deviation(snapshot.rmssd, snapshot.session)
        : NAN;

const float shortDev =
    (isfinite(snapshot.rmssd) &&
     isfinite(snapshot.shortReference) &&
     snapshot.shortReference > 0.0f)
        ? rmssd::Monitor::deviation(snapshot.rmssd, snapshot.shortReference)
        : NAN;
    const auto &pdr = snapshot.respiration;

    BleLine line(output, capacity);
    // Five terminal lines per snapshot; full diagnostics remain on Serial.
    line.append("Time:%lus", static_cast<unsigned long>(snapshot.uptimeSeconds));
    line.append(" | LMS_HR:"); line.value(snapshot.hr, "bpm");
    line.append(" | LMS_IBI:"); line.value(snapshot.ibi, "ms", 0);
    const char *motionText = snapshot.motion == 0 ? "LOW" :
                             snapshot.motion == 1 ? "MODERATE" :
                             snapshot.motion == 2 ? "HIGH" : "UNKNOWN";
    line.append(" | Motion:%s", motionText);

    line.append("\r\nSessionDev:"); line.value(sessionDev, "%");
    line.append(" | ShortDev:"); line.value(shortDev, "%");

    line.append("\r\nCal100:%lu/%lus",
                static_cast<unsigned long>(snapshot.sessionValidMs / 1000UL),
                static_cast<unsigned long>(snapshot.sessionTargetMs / 1000UL));
    line.append(" | Session:"); p1Counter(line, snapshot.p1Ms[0], snapshot.session);
    line.append(" | Short:"); p1Counter(line, snapshot.p1Ms[1], snapshot.shortReference);
    line.append(" | P1:%s", snapshot.p1Triggered ? "YES" : "NO");

    line.append("\r\nPDR:"); line.value(pdr.valid ? pdr.rate : NAN, "brpm");
    line.append(" | PDRStatus:%s", pdr::statusName(pdr.status));

    line.append("\r\nPersonalBaseline:"); line.value(snapshot.personal, "ms");
    return line.finish();
}


// ============================================================
// CONNECTION CALLBACKS
// ============================================================

class ConnectionCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        peerMtu.store(23);
        connected.store(true);

        Serial.println("[BLE] Phone connected");
    }

    void onDisconnect(BLEServer *) override {
        connected.store(false);
        peerMtu.store(23);

        if (subscription)
            subscription->setNotifications(false);

        restartAdvertising.store(true);

        Serial.println("[BLE] Phone disconnected");
    }

    void onMtuChanged(
        BLEServer *,
        esp_ble_gatts_cb_param_t *param) override {

        peerMtu.store(param->mtu.mtu);

        Serial.print("[BLE] MTU = ");
        Serial.println(param->mtu.mtu);
    }
};

ConnectionCallbacks callbacks;


// ============================================================
// TERMINAL WRITE CALLBACK
//
// This is required mainly so Bluetooth Serial Terminal sees
// a normal bidirectional serial-style BLE service.
//
// You do NOT need to send anything from the phone for the
// telemetry stream to work.
// ============================================================

class TerminalWriteCallbacks : public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic *characteristic) override {

        std::string data = characteristic->getValue();

        if (data.empty())
            return;

        Serial.print("[BLE RX] ");

        for (char c : data)
            Serial.print(c);

        Serial.println();
    }
};

TerminalWriteCallbacks terminalWriteCallbacks;


// ============================================================
// SEND ONE TEXT LINE OVER BLE
// ============================================================

void sendLine(const char *text) {

    if (!text)
        return;

    size_t remaining = strlen(text);

    while (remaining &&
           connected.load() &&
           subscription &&
           subscription->getNotifications()) {

        const size_t count =
            notificationChunkSize(
                remaining,
                peerMtu.load());

        stream->setValue(
            reinterpret_cast<uint8_t *>(
                const_cast<char *>(text)),
            count);

        stream->notify();

        text += count;
        remaining -= count;

        // Small delay prevents flooding the BLE stack.
        // This task runs on core 0, not the sensor loop.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// ============================================================
// BLE WORKER
// ============================================================

void worker(void *) {

    BLEDevice::init("Sutra");

    if (!BLEDevice::getInitialized()) {
        failed.store(true);
        vTaskDelete(nullptr);
        return;
    }

    // Request large MTU.
    // Actual value depends on phone negotiation.
    BLEDevice::setMTU(517);

    BLEServer *server =
        BLEDevice::createServer();

    server->setCallbacks(&callbacks);

    BLEService *service =
        server->createService(
            BLEUUID(SERVICE_UUID),
            48);


// ============================================================
// CREATE INDIVIDUAL READABLE TELEMETRY CHARACTERISTICS
//
// 0101
// 0102
// ...
// 0109
// ============================================================

    for (unsigned i = 0;
         i < VALUE_COUNT;
         ++i) {

        char uuid[37];

        snprintf(
            uuid,
            sizeof(uuid),
            "7d2a%04x-8f5a-4b10-9b6d-6f7574726100",
            // Keep existing UUIDs stable; retired slot 0105 remains unused.
            0x101 + i + (i >= static_cast<unsigned>(Group::SESSION) ? 1 : 0));

        values[i] =
            service->createCharacteristic(
                uuid,
                BLECharacteristic::PROPERTY_READ);

        auto *description =
            new BLEDescriptor(
                BLEUUID((uint16_t)0x2901));

        description->setAccessPermissions(
            ESP_GATT_PERM_READ);

        description->setValue(
            i == SELECTED_READ_GROUP
                ? "Selected Sutra telemetry"
                : "Reserved - use selected telemetry");

        values[i]->addDescriptor(description);

        values[i]->setValue("--");
    }


// ============================================================
// PHONE -> SUTRA WRITE CHARACTERISTIC
//
// UUID 01FE
//
// Bluetooth Serial Terminal needs a WRITE characteristic
// in the same service.
// ============================================================

    terminalWrite =
        service->createCharacteristic(
            WRITE_UUID,
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR);

    terminalWrite->setCallbacks(
        &terminalWriteCallbacks);

    {
        auto *description =
            new BLEDescriptor(
                BLEUUID((uint16_t)0x2901));

        description->setAccessPermissions(
            ESP_GATT_PERM_READ);

        description->setValue(
            "Bluetooth terminal input");

        terminalWrite->addDescriptor(
            description);
    }


// ============================================================
// SUTRA -> PHONE LIVE TEXT STREAM
//
// UUID 01FF
//
// READ added in addition to NOTIFY so terminal apps that
// explicitly search for a readable characteristic also
// recognize this characteristic.
// ============================================================

    stream =
        service->createCharacteristic(
            STREAM_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);

    stream->setValue(
        "Sutra telemetry ready\r\n");

    subscription =
        new BLE2902();

    stream->addDescriptor(subscription);

    {
        auto *description =
            new BLEDescriptor(
                BLEUUID((uint16_t)0x2901));

        description->setAccessPermissions(
            ESP_GATT_PERM_READ);

        description->setValue(
            "All groups - newline text stream");

        stream->addDescriptor(
            description);
    }


// ============================================================
// START SERVICE + ADVERTISING
// ============================================================

    service->start();

    BLEAdvertising *advertising =
        BLEDevice::getAdvertising();

    advertising->addServiceUUID(
        SERVICE_UUID);

    advertising->setScanResponse(true);

    advertising->start();

    ready.store(true);

    Serial.println(
        "[BLE] Sutra telemetry ready");


// ============================================================
// WORKER LOOP
// ============================================================

    for (;;) {

        if (restartAdvertising.exchange(false)) {

            // Avoid restarting from BLE callback.
            vTaskDelay(
                pdMS_TO_TICKS(200));

            advertising->start();

            Serial.println(
                "[BLE] Advertising restarted");
        }


        Snapshot snapshot;

        if (xQueueReceive(
                snapshots,
                &snapshot,
                pdMS_TO_TICKS(250))
            != pdTRUE) {

            continue;
        }


        char line[LINE_CAPACITY];

        if (!formatBleSelection(snapshot, line, sizeof(line)))
            continue;


// ------------------------------------------------------------
// Update BLE READ characteristics
//
// Keep the existing UUIDs, but only the selected-telemetry characteristic
// exposes live values. The legacy full-group BLE formatter is intentionally
// retained below as a comment; Serial still uses it in main.cpp.
// ------------------------------------------------------------
        for (unsigned i = 0;
             i < VALUE_COUNT;
             ++i) {
            values[i]->setValue(i == SELECTED_READ_GROUP ? std::string(line) : "--");

            // Legacy full BLE group output, retained for future restoration:
            // if (formatLine(static_cast<Group>(i), snapshot, line, sizeof(line)))
            //     values[i]->setValue(std::string(line));
        }


// ------------------------------------------------------------
// Send selected telemetry through 01FF
// ------------------------------------------------------------
        if (connected.load() && subscription && subscription->getNotifications())
            sendLine(line);

        // Legacy full BLE notification loop, retained for future restoration:
        // for (unsigned i = 0; i < VALUE_COUNT; ++i)
        //     if (formatLine(static_cast<Group>(i), snapshot, line, sizeof(line)))
        //         sendLine(line);


// ------------------------------------------------------------
// Blank line between each one-second selected telemetry update
// ------------------------------------------------------------

        sendLine("\r\n");
    }
}

} // namespace


// ============================================================
// PUBLIC API
// ============================================================

bool begin() {

#if !SUTRA_BLE_ENABLED

    return true;

#else

    snapshots =
        xQueueCreate(
            1,
            sizeof(Snapshot));

    if (!snapshots) {

        failed.store(true);

        return false;
    }


    // Sensor loop runs on core 1.
    // BLE runs independently on core 0.
    if (xTaskCreatePinnedToCore(
            worker,
            "SutraBLE",
            8192,
            nullptr,
            1,
            nullptr,
            0)
        != pdPASS) {

        vQueueDelete(snapshots);

        snapshots = nullptr;

        failed.store(true);

        return false;
    }

    return true;

#endif
}


void publish(
    const Snapshot &snapshot) {

    if (snapshots)
        xQueueOverwrite(
            snapshots,
            &snapshot);
}


const char *state() {

#if !SUTRA_BLE_ENABLED

    return "OFF";

#else

    if (failed.load())
        return "ERROR";

    if (!ready.load())
        return "STARTING";

    return connected.load()
               ? "CONNECTED"
               : "ADVERTISING";

#endif
}

} // namespace telemetry
