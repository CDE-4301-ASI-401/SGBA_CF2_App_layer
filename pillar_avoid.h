#ifndef CRAZYFLIE_PILLAR_AVOIDANCE_H
#define CRAZYFLIE_PILLAR_AVOIDANCE_H

#include <stdbool.h>

#define TAKEOFF_HEIGHT 0.5f
#define FORWARD_SPEED 0.2f
#define AVOID_SPEED 0.1f
#define SAFE_DISTANCE 0.3f
#define FORWARD_DURATION 5  // Move forward for 5 seconds

bool isObstacleAhead(void);
void fly_forward_then_avoid(void);
void appMain(void);

#endif // CRAZYFLIE_PILLAR_AVOIDANCE_H