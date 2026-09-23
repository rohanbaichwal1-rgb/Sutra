#include "BleTelemetry.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <atomic>
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
            0x101 + i);

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
            groupName(
                static_cast<Group>(i)));

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


// ------------------------------------------------------------
// Update normal READ characteristics
// ------------------------------------------------------------

        for (unsigned i = 0;
             i < VALUE_COUNT;
             ++i) {

            if (formatLine(
                    static_cast<Group>(i),
                    snapshot,
                    line,
                    sizeof(line))) {

                values[i]->setValue(
                    std::string(line));
            }
        }


// ------------------------------------------------------------
// Send complete telemetry continuously through 01FF
// ------------------------------------------------------------

        for (unsigned i = 0;
             i < VALUE_COUNT;
             ++i) {

            if (!connected.load())
                break;

            if (!subscription ||
                !subscription->getNotifications())
                break;

            if (formatLine(
                    static_cast<Group>(i),
                    snapshot,
                    line,
                    sizeof(line))) {

                sendLine(line);
            }
        }


// ------------------------------------------------------------
// Blank line between each one-second telemetry block
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