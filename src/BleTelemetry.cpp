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
constexpr const char *SERVICE_UUID = "7d2a0000-8f5a-4b10-9b6d-6f7574726100";
constexpr unsigned VALUE_COUNT = static_cast<unsigned>(Group::COUNT);
QueueHandle_t snapshots = nullptr;
std::atomic<bool> ready{false}, connected{false}, restartAdvertising{false};
std::atomic<bool> failed{false};
BLECharacteristic *values[VALUE_COUNT] = {};
BLECharacteristic *stream = nullptr;
BLE2902 *subscription = nullptr;
std::atomic<uint16_t> peerMtu{23};

class ConnectionCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *) override {
        peerMtu.store(23);
        connected.store(true);
    }
    void onDisconnect(BLEServer *) override {
        connected.store(false);
        peerMtu.store(23);
        subscription->setNotifications(false);
        restartAdvertising.store(true);
    }
    void onMtuChanged(BLEServer *, esp_ble_gatts_cb_param_t *param) override {
        peerMtu.store(param->mtu.mtu);
    }
};
ConnectionCallbacks callbacks;

void sendLine(const char *text) {
    size_t remaining = strlen(text);
    while (remaining && connected.load() && subscription->getNotifications()) {
        const size_t count = notificationChunkSize(remaining, peerMtu.load());
        stream->setValue(reinterpret_cast<uint8_t *>(const_cast<char *>(text)), count);
        stream->notify(); // no indication/ack wait; only this worker sends
        text += count;
        remaining -= count;
        // Pace traffic on core 0, never in the sensor loop.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void worker(void *) {
    BLEDevice::init("Sutra");
    if (!BLEDevice::getInitialized()) {
        failed.store(true);
        vTaskDelete(nullptr);
        return;
    }
    BLEDevice::setMTU(517); // Phone still negotiates the actual ATT MTU.
    BLEServer *server = BLEDevice::createServer();
    server->setCallbacks(&callbacks);
    BLEService *service = server->createService(BLEUUID(SERVICE_UUID), 48);
    for (unsigned i = 0; i < VALUE_COUNT; ++i) {
        char uuid[37];
        snprintf(uuid, sizeof(uuid), "7d2a%04x-8f5a-4b10-9b6d-6f7574726100", 0x101 + i);
        // Full lines stay readable even when stream notifications are fragmented.
        values[i] = service->createCharacteristic(uuid, BLECharacteristic::PROPERTY_READ);
        auto *description = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
        description->setAccessPermissions(ESP_GATT_PERM_READ);
        description->setValue(groupName(static_cast<Group>(i)));
        values[i]->addDescriptor(description);
        values[i]->setValue("--");
    }
    stream = service->createCharacteristic("7d2a01ff-8f5a-4b10-9b6d-6f7574726100",
                                           BLECharacteristic::PROPERTY_NOTIFY);
    subscription = new BLE2902();
    stream->addDescriptor(subscription);
    auto *description = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    description->setAccessPermissions(ESP_GATT_PERM_READ);
    description->setValue("All groups - newline text stream");
    stream->addDescriptor(description);
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
        char line[LINE_CAPACITY];
        for (unsigned i = 0; i < VALUE_COUNT; ++i) {
            if (formatLine(static_cast<Group>(i), snapshot, line, sizeof(line)))
                values[i]->setValue(std::string(line));
        }
        for (unsigned i = 0; i < VALUE_COUNT; ++i) {
            if (!connected.load() || !subscription->getNotifications()) break;
            if (formatLine(static_cast<Group>(i), snapshot, line, sizeof(line))) sendLine(line);
        }
        sendLine("\n");
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
