#pragma once

namespace SolarPanelRipper
{

void begin();
void start();

// Runs the fallback grab sequence (solarPanelFallbackSequence_1KHz)
// instead of the normal one. Use this when the robot never managed
// to align precisely with the panel (manual strafe + IR close-range
// search both failed), since it ends up in a different position.
void startFallback();

void update();
void stop();

bool isFinished();
bool hasFailed();
bool isDone();

} // namespace SolarPanelRipper