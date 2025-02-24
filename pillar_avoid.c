#include <stdio.h>
#include <stdbool.h>
#include "stabilizer.h"
#include "controller.h"
#include "commander.h"
#include "param.h"
#include "log.h"
#include "deck.h"
#include "range.h"
#include "pillar_avoid.h"
#include <time.h>
#include <unistd.h>
#include "FreeRTOS.h"

#include <string.h>
#include <errno.h>
#define __USE_MISC
#include <math.h>

#include "task.h"
#include "semphr.h"
#include "log.h"
#include "param.h"
#include "system.h"

#include "commander.h"
#include "sensors.h"
#include "stabilizer_types.h"

#include "estimator_kalman.h"
#include "stabilizer.h"

#include "wallfollowing_multiranger_onboard.h"
#include "wallfollowing_with_avoid.h"
#include "SGBA.h"
#include "usec_time.h"

#include "range.h"
#include "radiolink.h"
#include "median_filter.h"
#include "configblock.h"
#include "debug.h"

#define vTaskDelay(ms) usleep((ms) * 1000) // Sleep for `ms` milliseconds
#define xTaskGetTickCount() ((uint32_t)clock() * 1000 / CLOCKS_PER_SEC) // Get time in milliseconds

void p2pcallbackHandler(P2PPacket *p);

static void take_off(setpoint_t *sp, float velocity)
{
    sp->mode.x = modeVelocity;
    sp->mode.y = modeVelocity;
    sp->mode.z = modeVelocity;
    sp->velocity.x = 0.0;
    sp->velocity.y = 0.0;
    sp->velocity.z = velocity;
    sp->mode.yaw = modeVelocity;
    sp->attitudeRate.yaw = 0.0;
}

static void land(setpoint_t *sp, float velocity)
{
    sp->mode.x = modeVelocity;
    sp->mode.y = modeVelocity;
    sp->mode.z = modeVelocity;
    sp->velocity.x = 0.0;
    sp->velocity.y = 0.0;
    sp->velocity.z = -velocity;
    sp->mode.yaw = modeVelocity;
    sp->attitudeRate.yaw = 0.0;
}

static void shut_off_engines(setpoint_t *sp)
{
    sp->mode.x = modeDisable;
    sp->mode.y = modeDisable;
    sp->mode.z = modeDisable;
    sp->mode.yaw = modeDisable;
}

void fly_forward_then_avoid(void) {
    setpoint_t sp = {0};
    bool turnRight = true; // Alternate turning direction

    // Takeoff
    take_off(&sp, 0.5f);  // Ascend at 0.5m/s
    commanderSetSetpoint(&sp, 1);
    vTaskDelay(M2T(2000)); // Wait for takeoff

    // Ensure drone reaches the target height before proceeding
    while ((float)rangeGet(rangeDown) > TAKEOFF_HEIGHT * 1.2f || (float)rangeGet(rangeDown) < TAKEOFF_HEIGHT * 0.8f) {
        vTaskDelay(M2T(100));
    }

    uint32_t startTime = xTaskGetTickCount();
    while ((xTaskGetTickCount() - startTime) < M2T(FORWARD_DURATION * 1000)) {
        sp.velocity.x = FORWARD_SPEED;
        commanderSetSetpoint(&sp, 2);
        vTaskDelay(M2T(100));

        // Check for hand above drone
        if ((float)rangeGet(rangeUp) < 0.15f) { 
            break; // Exit loop and land
        }
    }
    
    // Start obstacle avoidance
    while (true) {
        if ((float)rangeGet(rangeUp) < 0.15f) {  // If hand is detected above
            break;  // Exit loop to land
        }

        if ((float)rangeGet(rangeFront) < CRITICAL_DISTANCE) {
            sp.velocity.x = 0.0f; // Stop
            sp.velocity.y = turnRight ? -AVOID_SPEED : AVOID_SPEED; // Alternate turn
            turnRight = !turnRight;
        } else {
            sp.velocity.x = FORWARD_SPEED;
            sp.velocity.y = 0.0f;
        }
        commanderSetSetpoint(&sp, 3);
        vTaskDelay(M2T(100));
    }

    // Land if hand is detected
    land(&sp, 0.1f);
    commanderSetSetpoint(&sp, 4);
    vTaskDelay(3000); // Wait for landing

    // Shut off motors
    shut_off_engines(&sp);
    commanderSetSetpoint(&sp, 5);
}

void appMain(void) {
    fly_forward_then_avoid();
}
