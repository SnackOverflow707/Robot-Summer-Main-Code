#pragma once

// Shared calibration constants describing where the solar panel sits
// relative to the side-tape crossing that marks it.

// TODO measure
static constexpr float SOLAR_PANEL_FROM_SIDE_TAPES_DX = 0.0f;
static constexpr float SOLAR_PANEL_FROM_SIDE_TAPES_DY = 0.0f;

// Used by SlowTapeFollowing::haveSolarPanelsPassed() as a margin so the
// transition out of tape-following fires slightly before the exact
// calibrated point, rather than requiring pixel-perfect arrival.
static constexpr float SEARCH_THRESHOLD_X = 0.03f;  // TODO tune
static constexpr float SEARCH_THRESHOLD_Y = 0.03f;  // TODO tune