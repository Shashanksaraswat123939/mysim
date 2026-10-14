#pragma once
#include <stdint.h>
#include "track.h"

struct MapInfo {
  const char* name;
  const char* summary;
  const Segment* segments;
  uint8_t count;
};

constexpr uint8_t kMapCount = 6;

const MapInfo& mapInfo(uint8_t index);
