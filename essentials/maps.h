#pragma once
// The map catalogue. Each map is a route the robot must follow from the start
// to the finish. Add a map by writing a new Segment list in maps.cpp.
#include <stdint.h>
#include "track.h"

struct MapInfo {
  const char* name;
  const char* summary;
  const Segment* segments;
  uint8_t count;
};

constexpr uint8_t kMapCount = 6;

// Index is clamped to the catalogue.
const MapInfo& mapInfo(uint8_t index);
