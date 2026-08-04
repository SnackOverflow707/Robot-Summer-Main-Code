#pragma once

// Shared calibration constants describing where the solar panel sits
// relative to the side-tape crossing that marks it.

// TODO measure
// Pose data (UART::PoseData) is in millimeters -- see
// Sensor_ESP_Arduino/src/main.cpp ("Position is just dx/dy integrated
// straight from the sensor, in mm.") -- so these must be in mm too.

//placeholder values for testing. 
static constexpr float SOLAR_PANEL_CHECKPOINT_DX = -57.00f;
static constexpr float SOLAR_PANEL_CHECKPOINT_DY = 400.000f;

// Used by SlowTapeFollowing::haveSolarPanelsPassed() as a margin so the
// transition out of tape-following fires slightly before the exact
// calibrated point, rather than requiring pixel-perfect arrival.
static constexpr float SEARCH_THRESHOLD_X_MM = 15.0f;  // TODO tune
static constexpr float SEARCH_THRESHOLD_Y_MM = 15.0f;  // TODO tune