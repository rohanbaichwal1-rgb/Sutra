#include "PdrService.h"
#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace pdrService {
namespace {
enum class Kind { BEAT, CONTACT, CONTEXT };
struct Event {
    Kind kind;
    uint32_t time;
    float amplitude = 0, ibi = 0;
    pdr::Context context = {};
};
QueueHandle_t inputs = nullptr, outputs = nullptr;
std::atomic<bool> lost{false};
pdr::Observer observer; // owned exclusively by worker
bool haveContext = false;
uint32_t lastContextMs = 0;
unsigned lastContextMask = 0;
void post(const Event &event) {
    if (inputs && xQueueSend(inputs, &event, 0) != pdTRUE) lost.store(true);
}
void worker(void *) {
    Event event;
    for (;;) {
        if (xQueueReceive(inputs, &event, portMAX_DELAY) != pdTRUE) continue;
        if (lost.exchange(false)) {
            // Missing beats/context make the entire respiratory window suspect.
            xQueueReset(inputs);
            observer.dataLost(event.time, pdr::ResetReason::QUEUE_OVERFLOW);
            pdr::Result result = observer.result();
            xQueueOverwrite(outputs, &result);
            continue;
        }
        switch (event.kind) {
        case Kind::BEAT: observer.addBeat(event.time, event.amplitude, event.ibi); break;
        case Kind::CONTACT: observer.contactChanged(event.time); break;
        case Kind::CONTEXT: observer.update(event.time, event.context); break;
        }
        const pdr::Result result = observer.result();
        xQueueOverwrite(outputs, &result);
    }
}
}
bool begin() {
    inputs = xQueueCreate(64, sizeof(Event));
    outputs = xQueueCreate(1, sizeof(pdr::Result));
    if (inputs && outputs &&
        xTaskCreatePinnedToCore(worker, "SutraPDR", 8192, nullptr, 1, nullptr, 0) == pdPASS)
        return true;
    if (inputs) vQueueDelete(inputs);
    if (outputs) vQueueDelete(outputs);
    inputs = outputs = nullptr;
    return false;
}
void beat(uint32_t time, float amplitude, float ibi) {
    Event event; event.kind = Kind::BEAT; event.time = time;
    event.amplitude = amplitude; event.ibi = ibi;
    post(event);
}
void contactChanged(uint32_t now) {
    Event event; event.kind = Kind::CONTACT; event.time = now; post(event);
}
void context(uint32_t now, const pdr::Context &context) {
    const unsigned mask = context.signalGood | (context.lowMotion << 1);
    if (haveContext && mask == lastContextMask && now-lastContextMs < 100) return;
    haveContext = true; lastContextMs = now; lastContextMask = mask;
    Event event; event.kind = Kind::CONTEXT; event.time = now; event.context = context;
    post(event);
}
pdr::Result read(uint32_t now) {
    pdr::Result result;
    if (!outputs || xQueuePeek(outputs, &result, 0) != pdTRUE ||
        now-result.time > pdr::MAX_GAP_MS) {
        result = pdr::Result{};
        result.status = pdr::Status::STALE;
    }
    return result;
}
}
