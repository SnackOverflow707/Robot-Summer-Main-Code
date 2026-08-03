#pragma once

// Shared time budget for deciding whether there's still time left to go
// for another rock before the tower/solar-panel portion of the course
// needs to start.

// Total time allotted for the whole course/heat.
static constexpr unsigned long TOT_TIME_FOR_COURSE = 120000UL; // 2 minutes

// TODO measure: worst-case time to complete the tower build + solar
// panel rip once rocks are done.
static constexpr unsigned long TOT_TIME_TOWER_AND_PANELS = 0UL;

// Safety margin subtracted from the remaining rock-time budget.
static constexpr unsigned long TIME_BUFFER = 0UL;

// Latest course-elapsed time (ms) at which it's still worth starting
// another rock attempt.
static constexpr unsigned long ROCK_TIME_DEADLINE_MS =
    TOT_TIME_FOR_COURSE - TOT_TIME_TOWER_AND_PANELS - TIME_BUFFER;

// TODO measure: worst-case time for one RockApproach / RockGrabber attempt.
static constexpr unsigned long ROCK_APPROACH_WORST_CASE_MS = 0UL;
static constexpr unsigned long ROCK_GRAB_WORST_CASE_MS = 0UL;
