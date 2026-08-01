#ifndef SLOW_TAPE_FOLLOWING_H
#define SLOW_TAPE_FOLLOWING_H

#include <Arduino.h>

#define SOLAR_PANEL_FROM_SIDE_TAPES_DX 0.732 //placeholder
#define SOLAR_PANEL_FROM_SIDE_TAPES_DY -0.132 //placeholder
#define SEARCH_THRESHOLD_X 0.05 //placeholder 
#define SEARCH_THRESHOLD_Y 0.05 //placeholder 

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

bool haveSideTapesPassed(); 

bool havePanelsPassed(float currentX, float currentY); 

} // namespace SlowTapeFollowing

#endif // SLOW_TAPE_FOLLOWING_H