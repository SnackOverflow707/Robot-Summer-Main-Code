#pragma once
#include "robotArm/armSequences/rock.h"

namespace RockGrabber
{

void begin();
void start(int rockIndex);
void stop();
void update();

bool isFinished();
bool hasFailed();
bool isDone();

} // namespace RockGrabber
