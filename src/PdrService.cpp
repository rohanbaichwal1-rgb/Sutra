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
std::atomic<uint32_t> lostTime{0};
pdr::Observer observer; // owned exclusively by worker
bool haveContext = false;
uint32_t lastContextMs = 0;
unsigned lastContextMask = 0;
bool post(const Event &event) {
    if (!inputs)
        return false;

    if (xQueueSend(inputs, &event, 0) == pdTRUE)
        return true;

    // Losing a beat or contact event compromises
    // continuity, so the observer must be informed.
    if (event.kind != Kind::CONTEXT)
        lost.store(true);

    return false;
}
void worker(void *) {
    Event event;
    bool lossPending = false;
    uint32_t pendingLossTime = 0;
    for (;;) {
        if (xQueueReceive(inputs, &event, portMAX_DELAY) != pdTRUE) continue;

        if (lost.exchange(false) && !lossPending) {
            lossPending = true;
            pendingLossTime = lostTime.load();
        }

        if (lossPending && (uint32_t)(event.time - pendingLossTime) < 0x80000000UL) {
            observer.dataLost(pendingLossTime, pdr::ResetReason::QUEUE_OVERFLOW);
            pdr::Result result = observer.result();
            xQueueOverwrite(outputs, &result);
            lossPending = false;
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
    const unsigned mask =
        (context.signalGood ? 1u : 0u) |
        (context.lowMotion ? 2u : 0u);

    const bool changed =
        !haveContext || mask != lastContextMask;

    if (!changed &&
        now - lastContextMs < 100)
        return;

    Event event;
    event.kind = Kind::CONTEXT;
    event.time = now;
    event.context = context;

    if (post(event)) {
        haveContext = true;
        lastContextMs = now;
        lastContextMask = mask;
    } else if (changed) {
        // Important state transition could not be
        // delivered. Do not silently lose it.
        lost.store(true);
    }
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
