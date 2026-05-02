#pragma once

#include "wled.h"

// ---------------------------------------------------------------------------
// Stub usermod for adding custom effects to WLED via strip.addEffect().
//
// Add new effects by:
//   1. Writing a function `uint16_t mode_my_effect(void)` that draws one frame
//      using SEGMENT.setPixelColor(...) and returns the desired delay (ms)
//      until the next call. Use FRAMETIME for "as fast as possible".
//   2. Defining a metadata string for the UI sliders/colors.
//   3. Calling strip.addEffect(...) for it inside CustomFxUsermod::setup().
//
// The mode-id (first arg to addEffect) only needs to be unique within this
// build. Pick a small integer between 200 and 254 that no other usermod uses;
// id 255 is reserved.
// ---------------------------------------------------------------------------

// ----- Effect: Sync Wave --------------------------------------------------
// Travelling sine pulse coloured from the active palette. Demonstrates the
// minimal set of segment APIs you'll touch in any custom effect.
//   speed slider     -> wave travel speed
//   intensity slider -> wave width (smaller = sharper pulse)
//   palette          -> colour mapping
static uint16_t mode_sync_wave(void) {
  const uint16_t len = SEGMENT.virtualLength();
  if (len == 0) return FRAMETIME;

  // Advance the per-segment phase. SEGENV.step is a free 32-bit slot WLED
  // reserves per segment, perfect for timing state across frames.
  SEGENV.step += 1 + (SEGMENT.speed >> 4);

  // 1..255 -> ~32..2 (smaller intensity -> wider wave)
  const uint8_t sharpness = 1 + (SEGMENT.intensity >> 3);

  for (uint16_t i = 0; i < len; i++) {
    // Position along the strip in [0, 255].
    const uint8_t pos = (i * 255) / (len - 1 ? len - 1 : 1);
    // Sine pulse that travels along the strip.
    const uint8_t s = sin8((pos * sharpness) - (SEGENV.step & 0xFF));
    // Use s both as palette index and brightness so the wave fades on its
    // edges instead of cutting off hard.
    uint32_t col = SEGMENT.color_from_palette(s, false, true, 0);
    SEGMENT.setPixelColor(i, color_fade(col, s));
  }

  return FRAMETIME;
}
static const char _data_FX_MODE_SYNC_WAVE[] PROGMEM = "Sync Wave@!,Width;;!";

// ----- Usermod plumbing ---------------------------------------------------
class CustomFxUsermod : public Usermod {
  public:
    void setup() override {
      // Effect IDs 200-254 are typically free. Bump if you add more.
      strip.addEffect(200, &mode_sync_wave, _data_FX_MODE_SYNC_WAVE);
    }
    void loop() override {}

    uint16_t getId() override { return USERMOD_ID_CUSTOM_FX; }
};
