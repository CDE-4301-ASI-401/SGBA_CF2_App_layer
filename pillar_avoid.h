#ifndef CRAZYFLIE_PILLAR_AVOIDANCE_H
#define CRAZYFLIE_PILLAR_AVOIDANCE_H

#include <stdbool.h>

#define TAKEOFF_HEIGHT 0.8f
#define FORWARD_SPEED 0.2f
#define AVOID_SPEED 0.1f
#define SAFE_DISTANCE 0.3f
#define FORWARD_DURATION 3  // Move forward for 3 seconds
#define CRITICAL_DISTANCE 0.1f // 10cm threshold for turning

bool isObstacleAhead(void);
void fly_forward_then_avoid(void);
void appMain(void);
static void land(setpoint_t *sp, float velocity);
static void shut_off_engines(setpoint_t *sp);

#endif // CRAZYFLIE_PILLAR_AVOIDANCE_H