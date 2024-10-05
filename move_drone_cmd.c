// source: crazyfile-firmware/examples/demos/app_wall_following_demo/src/wallfollowing_multiranger_onboard.c
#include "move_drone_cmd.h"
#include "wallfollowing_multiranger_onboard copy_demo_code.h"
#include <math.h>

#define __USE_MISC

// variables
// static float ref_distance_from_wall;
// static float max_speed;
// static float max_rate;
// static float direction;
// static float first_run;
static int state;
static float state_start_time;
static float x_position;
static float y_position;
static float height;

static float refDistanceFromWall = 0.0f;
static float maxForwardSpeed = 0.2f;
static float maxTurnRate = 0.5f;
static float direction = 1.0f;
static float firstRun = false;
static float prevHeading = 0.0f;
static float wallAngle = 0.0f;
static bool aroundCornerBackTrack = false;
static float stateStartTime;
static float rangerValueBuffer = 0.2f;
static float angleValueBuffer = 0.1f;
static float rangerThresholdLost = 0.3f;
static const float inCornerAngle = 0.8f;
static const float waitForMeasurementSeconds = 1.0f;

static StateWF stateWF = forward;
float timeNow = 0.0f;


static StateWF stateWF = forward;
float timeNow = 0.0f;

void take_off_hover_land(float *cmdVelX, float *cmdVelY, float *cmdAngW, float timeOuter, float hoverDuration, float takeOffHeight) {
    float cmdVelXTemp = 0.0f;
    float cmdVelYTemp = 0.0f;
    float cmdAngWTemp = 0.0f;

    timeNow = timeOuter;

    switch (stateWF) {
        case take_off:
            // Take-off logic: increase height until the target takeOffHeight is reached
            if (height < takeOffHeight) {
                commandTakeOff(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp, &height);
            } else {
                // Once the target height is reached, transition to hover state
                stateWF = transition(hover);
            }
            break;

        case hover:
            // Hover command
            commandHover(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp);

            // Check if hover duration is complete, then land
            if (timeNow - stateStartTime >= hoverDuration) {
                stateWF = transition(land);
            }
            break;

        case land:
            // Landing logic: decrease height until 0
            if (height > 0.0f) {
                commandLand(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp, &height);
            } else {
                // Landed, stop all velocities
                *cmdVelX = 0.0f;
                *cmdVelY = 0.0f;
                *cmdAngW = 0.0f;
            }
            break;

        default:
            // Start with take-off state
            stateWF = transition(take_off);
            break;
    }

    // Update command velocities
    *cmdVelX = cmdVelXTemp;
    *cmdVelY = cmdVelYTemp;
    *cmdAngW = cmdAngWTemp;
}

static void commandHover(float *cmdVelX, float *cmdVelY, float *cmdAngW)
{
    *cmdVelX = 0.0f;
    *cmdVelY = 0.0f;
    *cmdAngW = 0.0f;
}

static void commandLand(float *cmdVelX, float *cmdVelY, float *cmdAngW, float *height)
{
    // Decrease height gradually (adjust rate as needed)
    *height -= 0.1f;
    *cmdVelX = 0.0f;
    *cmdVelY = 0.0f;
    *cmdAngW = 0.0f;
}

static void commandTakeOff(float *cmdVelX, float *cmdVelY, float *cmdAngW, float *height)
{
    // Increase height gradually
    *height += 0.1f;
    *cmdVelX = 0.0f;
    *cmdVelY = 0.0f;
    *cmdAngW = 0.0f;
}

static StateWF transition(StateWF newState)
{
    stateStartTime = timeNow;
    return newState;
}

void take_off_hover_rotate_land (float height, float rotational_speed) {
    // take off
    // hover at elevation h from the ground
    // rotate on the spot by 360 degrees at rotational speed of while hovering
    // land
    commandHover(&cmdVelXTemp, &cmdVelYTemp, &cmdAngWTemp);
    
}





// to take off run take.off_py
// to land run land_all.py
// 1. (Shu Hui) Take off ® hover for duration of t at elevation h from the ground ® land (you can try different values for t and h)
// 2. (Shu Hui) Take off ® hover at elevation h from the ground ® rotate on the spot by 360 degrees at rotational speed of while hovering ® land (you can try different values for h and )