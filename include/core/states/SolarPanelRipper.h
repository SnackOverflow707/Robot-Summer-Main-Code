#pragma once

namespace SolarPanelRipper
{

void begin();
void start();
void update();
void stop();

bool isFinished();
bool hasFailed();
bool isDone();

} // namespace SolarPanelRipper