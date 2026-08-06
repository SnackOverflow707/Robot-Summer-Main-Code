#pragma once
#include "robotArm/armSequences/rock.h"

namespace RockGrabber
{

void begin();

// FIX: isLastRock added -- start() used to always block through the full
// lift/place/retract/return-to-tape sequence. Now, for every rock except
// the last, the "place rock in bin + rehome" portion runs on a background
// FreeRTOS task while the drive strafes back to tape, so the caller can
// move on to TAPE_FOLLOW_TO_TOWER without waiting for the arm to finish.
// The last rock still runs everything inline/blocking, same as before.
void start(int rockIndex, bool isLastRock);

void stop();
void update();

bool isFinished();
bool hasFailed();
bool isDone();

// True while the background "place rock + retract" task (spawned for
// every rock except the last) is still running. The arm is not safe to
// use for anything else -- TOWER_RAM, another RockGrabber::start(), etc.
// -- until this returns false. Callers that use the arm after ROCK_GRAB
// should check this before touching it.
bool isArmTaskBusy();

} // namespace RockGrabber