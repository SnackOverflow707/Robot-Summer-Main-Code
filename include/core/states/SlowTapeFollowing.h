#ifndef SLOW_TAPE_FOLLOWING_H
#define SLOW_TAPE_FOLLOWING_H

#include <Arduino.h>

namespace SlowTapeFollowing
{

void begin();
void start();
void update();
void stop();

bool isIRDetected(
    uint16_t mag1,
    uint16_t mag2
);

} // namespace SlowTapeFollowing

#endif // SLOW_TAPE_FOLLOWING_H