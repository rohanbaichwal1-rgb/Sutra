#pragma once
#include <PdrObserver.h>

namespace pdrService {
bool begin();
void beat(uint32_t time, float amplitude, float ibi);
void contactChanged(uint32_t now);
void context(uint32_t now, const pdr::Context &context);
void beginP2(uint32_t now);
void endP2(uint32_t now, bool completed);
pdr::Result read(uint32_t now);
}
