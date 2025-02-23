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

#define TAKEOFF_HEIGHT 0.5f
#define FORWARD_SPEED 0.2f
#define AVOID_SPEED 0.1f
#define SAFE_DISTANCE 0.3f
#define CRITICAL_DISTANCE 0.1f // 10cm threshold for turning
#define FORWARD_DURATION 5  // Move forward for 5 seconds

// Ensure range sensor functions are correctly declared
extern float rangeFront(void);
extern float rangeLeft(void);
extern float rangeRight(void);

static bool isObstacleAhead(void) {
    return rangeFront() < SAFE_DISTANCE;
}

void fly_forward_then_avoid(void) {
    setpoint_t sp = {0};
    bool turnRight = true; // Alternate turning direction
    
    sp.position.z = TAKEOFF_HEIGHT;  // Takeoff
    commanderSetSetpoint(&sp, 1);
    vTaskDelay(M2T(2000));  // Wait 2 seconds

    uint32_t startTime = xTaskGetTickCount();
    while ((xTaskGetTickCount() - startTime) < M2T(FORWARD_DURATION * 1000)) {
        sp.velocity.x = FORWARD_SPEED;
        commanderSetSetpoint(&sp, 2);
        vTaskDelay(M2T(100));
    }
    
    // Start obstacle avoidance
    while (true) {
        if (rangeFront() < CRITICAL_DISTANCE) {
            sp.velocity.x = 0.0f; // Stop
            sp.velocity.y = 0.0f; // Stop sideways movement
            
            // Alternate turning direction
            if (turnRight) {
                sp.velocity.y = -AVOID_SPEED; // Turn right
            } else {
                sp.velocity.y = AVOID_SPEED; // Turn left
            }
            turnRight = !turnRight;
        } else {
            sp.velocity.x = FORWARD_SPEED;
            sp.velocity.y = 0.0f;
        }
        commanderSetSetpoint(&sp, 3);
        vTaskDelay(M2T(100));
    }
}

void appMain(void) {
    fly_forward_then_avoid();
}
