#include "BleTelemetry.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLE2902.h>
#include <atomic>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#ifndef SUTRA_BLE_ENABLED
#define SUTRA_BLE_ENABLED 1
#endif

namespace telemetry {
namespace {
constexpr const char *SERVICE_UUID = "7d2a0000-8f5a-4b10-9b6d-6f7574726100";
constexpr unsigned VALUE_COUNT = 8;
// Preserve existing UUIDs for status and timing after removing battery telemetry.
constexpr unsigned VALUE_IDS[VALUE_COUNT] = {1, 2, 3, 4, 5, 6, 8, 9};
QueueHandle_t snapshots = nullptr;
std::atomic<bool> ready{false}, connected{false}, restartAdvertising{false};
std::atomic<bool> failed{false};
BLECharacteristic *values[VALUE_COUNT] = {};

class ConnectionCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *) override { connected.store(true); }
    void onDisconnect(BLEServer *) override {
        connected.store(false);
        restartAdvertising.store(true);
    }
};
ConnectionCallbacks callbacks;

void send(unsigned index, const char *text) {
    values[index]->setValue(std::string(text));
    if (connected.load()) values[index]->notify(); // no indication/ack wait
}

void number(unsigned index, float value) {
    char text[21];
    if (isfinite(value) && value >= 0 && value < 100000)
        snprintf(text, sizeof(text), "%.1f", value);
    else snprintf(text, sizeof(text), "--");
    send(index, text);
}

void worker(void *) {
    BLEDevice::init("Sutra");
    if (!BLEDevice::getInitialized()) {
        failed.store(true);
        vTaskDelete(nullptr);
        return;
    }
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(&callbacks);
    // Eight characteristics, each with value, declaration, CCCD and description.
    BLEService *service = server->createService(BLEUUID(SERVICE_UUID), 48);
    const char *names[VALUE_COUNT] = {
        "LMS HR (bpm)", "LMS median IBI (ms)", "RMSSD (ms)",
        "Session baseline (ms)", "Live short reference (ms)",
        "7-session baseline (ms)",
        "Finger,motion,converged,stable,fresh,saved,episode",
        "Uptime(s),max loop gap(us)"
    };
    for (unsigned i = 0; i < VALUE_COUNT; ++i) {
        char uuid[37];
        snprintf(uuid, sizeof(uuid), "7d2a%04x-8f5a-4b10-9b6d-6f7574726100", VALUE_IDS[i]);
        values[i] = service->createCharacteristic(uuid,
            BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
        values[i]->addDescriptor(new BLE2902());
        auto *description = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
        description->setAccessPermissions(ESP_GATT_PERM_READ);
        description->setValue(names[i]);
        values[i]->addDescriptor(description);
        values[i]->setValue("--");
    }
    service->start();
    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->start();
    ready.store(true);
    for (;;) {
        if (restartAdvertising.exchange(false)) {
            // Avoid restarting inside a Bluetooth callback.
            vTaskDelay(pdMS_TO_TICKS(200));
            advertising->start();
        }
        Snapshot snapshot;
        if (xQueueReceive(snapshots, &snapshot, pdMS_TO_TICKS(250)) != pdTRUE)
            continue;
        number(0, snapshot.hr);
        number(1, snapshot.ibi);
        number(2, snapshot.rmssd);
        number(3, snapshot.session);
        number(4, snapshot.shortReference);
        number(5, snapshot.personal);
        char text[21]; // every notification fits the default 23-byte ATT MTU
        snprintf(text, sizeof(text), "%u,%u,%u,%u,%u,%u,%u",
            snapshot.finger, snapshot.motion, snapshot.converged, snapshot.stable,
            snapshot.fresh, snapshot.savedSessions, snapshot.episode);
        send(6, text);
        snprintf(text, sizeof(text), "%lu,%lu", (unsigned long)snapshot.uptimeSeconds,
            (unsigned long)snapshot.maxLoopGapUs);
        send(7, text);
    }
}
} // namespace

bool begin() {
#if !SUTRA_BLE_ENABLED
    return true;
#else
    snapshots = xQueueCreate(1, sizeof(Snapshot));
    if (!snapshots) { failed.store(true); return false; }
    // Arduino sensor loop is core 1. BLE formatting/notifications use core 0.
    if (xTaskCreatePinnedToCore(worker, "SutraBLE", 8192, nullptr, 1, nullptr, 0) != pdPASS) {
        vQueueDelete(snapshots);
        snapshots = nullptr;
        failed.store(true);
        return false;
    }
    return true;
#endif
}

void publish(const Snapshot &snapshot) {
    if (snapshots) xQueueOverwrite(snapshots, &snapshot);
}

const char *state() {
#if !SUTRA_BLE_ENABLED
    return "OFF";
#else
    if (failed.load()) return "ERROR";
    if (!ready.load()) return "STARTING";
    return connected.load() ? "CONNECTED" : "ADVERTISING";
#endif
}
} // namespace telemetry
