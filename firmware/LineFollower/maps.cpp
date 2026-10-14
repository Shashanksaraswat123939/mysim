#include "maps.h"

static const Segment kWarmUp[] = {
    Straight(0.60f), LeftArc(90, 0.30f), Straight(0.60f), LeftArc(90, 0.30f),
    Straight(0.60f), LeftArc(90, 0.30f), Straight(0.60f), LeftArc(90, 0.30f),
    Straight(0.30f),
};

static const Segment kChicane[] = {
    Straight(0.40f), RightArc(45, 0.15f), LeftArc(45, 0.15f),  Straight(0.30f),
    LeftArc(45, 0.15f), RightArc(45, 0.15f), Straight(0.40f), RightArc(90, 0.20f),
    Straight(0.40f),
};

static const Segment kHairpins[] = {
    Straight(0.50f), LeftArc(180, 0.10f), Straight(0.50f), RightArc(180, 0.10f),
    Straight(0.50f), LeftArc(180, 0.10f), Straight(0.40f),
};

static const Segment kDashed[] = {
    Straight(0.40f), Gap(0.06f), Straight(0.40f), LeftArc(90, 0.25f),
    Gap(0.10f), Straight(0.30f), RightArc(90, 0.25f), Gap(0.06f),
    Straight(0.50f),
};

static const Segment kZigZag[] = {
    Straight(0.30f), RightArc(90, 0.08f), Straight(0.25f), LeftArc(90, 0.08f),
    Straight(0.25f), RightArc(90, 0.08f), Straight(0.25f), LeftArc(90, 0.08f),
    Straight(0.30f),
};

static const Segment kSweepers[] = {
    Straight(1.00f), LeftArc(60, 0.60f), Straight(0.50f), RightArc(120, 0.50f),
    Straight(0.40f), LeftArc(60, 0.60f), Straight(0.80f),
};

static const MapInfo kMaps[kMapCount] = {
    {"Warm-Up Oval", "Four gentle 90 degree corners on long straights", kWarmUp,
     sizeof(kWarmUp) / sizeof(kWarmUp[0])},
    {"Chicane", "Alternating S-bends with 15 cm radius", kChicane,
     sizeof(kChicane) / sizeof(kChicane[0])},
    {"Hairpin Alley", "Three 180 degree turns at 10 cm radius", kHairpins,
     sizeof(kHairpins) / sizeof(kHairpins[0])},
    {"Dashed Line", "Painted line with gaps the robot must coast over", kDashed,
     sizeof(kDashed) / sizeof(kDashed[0])},
    {"Zig-Zag", "Tight 90 degree corners at 8 cm radius", kZigZag,
     sizeof(kZigZag) / sizeof(kZigZag[0])},
    {"Sweepers", "Fast, wide bends where the racing line matters most", kSweepers,
     sizeof(kSweepers) / sizeof(kSweepers[0])},
};

static_assert(sizeof(kMaps) / sizeof(kMaps[0]) == kMapCount, "kMapCount must match kMaps");

const MapInfo& mapInfo(uint8_t index) {
  return kMaps[index < kMapCount ? index : kMapCount - 1];
}
