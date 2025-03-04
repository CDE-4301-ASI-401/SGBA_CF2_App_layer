#include <math.h>
#include "stdbool.h"
//#include "debug.h"

// Variables
static float distance_from_pillar = 0.5f;
static float max_speed = 0.5f;
static float max_turn_speed = 0.5f;

// Converts degrees to radians.
#define deg2rad(angleDegrees) (angleDegrees * (float)M_PI / 180.0f)

// Converts radians to degrees.
#define rad2deg(angleRadians) (angleRadians * 180.0f / (float)M_PI)

static int transition(int new_state)
{
  //float t =  usecTimestamp() / 1e6;
  //state_start_time = t;
  //DEBUG_PRINT("Transitioning to state %d\n", new_state);
  return new_state;
}

// Static helper functions
// Check if yaw of drone is close to target value to a margin
static bool logicIsCloseTo(float real_value, float checked_value, float margin)
{
  if (real_value > checked_value - margin && real_value < checked_value + margin) {
    return true;
  } else {
    return false;
  }
}

// To determine direction when turning towards goal
static float wraptopi(float number)
{
  if (number > (float)M_PI) {
    return (number - (float)(2 * M_PI));
  } else if (number < (float)(-1 * M_PI)) {
    return (number + (float)(2 * M_PI));
  } else {
    return (number);
  }

}

// Command functions
static void commandTurn(float *vel_w, float max_rate)
{
  *vel_w = max_rate;
}

/***********************************************************
 * State definitions
 ***********************************************************/
// 1 = forward
// 2 = rotate away from obstacle
// 3 = wall following
// 4 = rotate back to forward direction

int pillar_controller(float *vel_x, float *vel_y, float *vel_w,
                            float frontRange, float leftRange, float rightRange,
                            float currentHeading, float currentPosX, float currentPosY)
{
    // Initialize static variables
    static int state = 1;
    static int turnDirection = 1; // 1 = left, -1 = right
    static float forwardHeading = 0; // heading of forward direction
    static float targetHeading = 0; // target heading when turn required

    /***********************************************************
     * Handle state transitions
     ***********************************************************/
    if (state == 1) {     //FORWARD
        if (frontRange < distance_from_pillar) {
            state = transition(2); //rotate away from obstacle
            targetHeading = wraptopi(currentHeading + deg2rad(90 * turnDirection));
        }
    } else if (state == 2) { //ROTATE AWAY FROM OBSTACLE
        if (logicIsCloseTo(wraptopi(currentHeading - targetHeading), 0, 0.1f)) {
            state = transition(3); //wall following
        }
    } else if (state == 3) { //WALL FOLLOWING
        if (turnDirection == 1) {
            if (rightRange > distance_from_pillar) {
                targetHeading = wraptopi(currentHeading + deg2rad(90 * -turnDirection));
                state = transition(4); //rotate back to forward direction
            }
        } else {
            if (leftRange > distance_from_pillar) {
                targetHeading = wraptopi(currentHeading + deg2rad(90 * -turnDirection));
                state = transition(4); //rotate back to forward direction
            }
        }
    } else if (state == 4) { //ROTATE BACK TO FORWARD DIRECTION
        if (logicIsCloseTo(wraptopi(currentHeading - targetHeading), 0, 0.1f)) {
            state = transition(1); // forward
            turnDirection *= -1;
        }
    }


    /***********************************************************
     * Handle state actions
     ***********************************************************/
    float temp_vel_x = 0;
    float temp_vel_y = 0;
    float temp_vel_w = 0;

    if (state == 1) {        //FORWARD
        temp_vel_x = max_speed;
        forwardHeading = currentHeading;
    } else if (state == 2) { // ROTATE AWAY FROM OBSTACLE
        commandTurn(&temp_vel_w, max_turn_speed * turnDirection);
    } else if (state == 3) { // WALL FOLLOWING
        temp_vel_x = max_speed;
    } else if (state == 4) { // ROTATE BACK TO FORWARD DIRECTION
        commandTurn(&temp_vel_w, max_turn_speed * -turnDirection);
    }

    *vel_x = temp_vel_x;
    *vel_y = temp_vel_y;
    *vel_w = temp_vel_w;

    return state;
}