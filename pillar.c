#include <math.h>
#include "stdbool.h"
//#include "debug.h"

// Variables
static float distance_from_pillar = 0.60f;
static float max_speed = 0.25f;
static float max_turn_speed = 0.5f;
static bool zigzag = false;
static float zigzagRate = 0.5f;
#define maxZigzagOffset 10
#define CRITICAL_DISTANCE 0.2f

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

// 11 = move away from wall (too close)
// 5 = reverse

int pillar_controller(float *vel_x, float *vel_y, float *vel_w,
                            float frontRange, float leftRange, float rightRange,
                            float currentHeading, float currentPosX, float currentPosY, 
                            bool commandReverse)
{
    // Initialize static variables
    static bool firstRun = true;
    static int state = 1;
    static int turnDirection = 1; // 1 = left, -1 = right
    static float forwardHeading = 0; // heading of forward direction
    static float targetHeading = 0; // target heading when turn required
    static float zigzagOffset = 0; // offset for zigzagging
    static int zigzagDirection = 1; // 1 = left, -1 = right

    if (firstRun) {
        forwardHeading = currentHeading;
        firstRun = false;
    }

    /***********************************************************
     * Handle state transitions
     ***********************************************************/
    if (state == 1) {     //FORWARD
        if (frontRange < distance_from_pillar) {
            state = transition(2); //rotate away from obstacle
            targetHeading = wraptopi(forwardHeading + deg2rad(90 * turnDirection));
        } else if (leftRange < CRITICAL_DISTANCE || rightRange < CRITICAL_DISTANCE) {
            state = transition(11); // move away from wall
        }
    } else if (state == 11) { // MOVE AWAY FROM WALL
        if (leftRange > CRITICAL_DISTANCE && rightRange > CRITICAL_DISTANCE) {
            state = transition(1); // forward
        }
    } else if (state == 2) { //ROTATE AWAY FROM OBSTACLE
        if (logicIsCloseTo(wraptopi(currentHeading - targetHeading), 0, 0.1f)) {
            state = transition(3); //wall following
        }
    } else if (state == 3) { //WALL FOLLOWING
        if (turnDirection == 1) {
            if (rightRange > distance_from_pillar) {
                vTaskDelay(1000);
                targetHeading = wraptopi(currentHeading + deg2rad(90 * -turnDirection));
                state = transition(4); //rotate back to forward direction
            }
        } else {
            if (leftRange > distance_from_pillar) {
                vTaskDelay(1000);
                targetHeading = wraptopi(currentHeading + deg2rad(90 * -turnDirection));
                state = transition(4); //rotate back to forward direction
            }
        }
    } else if (state == 4) { //ROTATE BACK TO FORWARD DIRECTION
        if (logicIsCloseTo(wraptopi(currentHeading - targetHeading), 0, 0.1f)) {
            state = transition(1); // forward
            forwardHeading = currentHeading;
            zigzagOffset = 0;
            zigzagDirection = -1;
            turnDirection *= -1;
        }
    } else if (state == 5) { // REVERSE
        if (logicIsCloseTo(wraptopi(currentHeading - targetHeading), 0, 0.1f)) {
            state = transition(1); // forward
            forwardHeading = currentHeading;
        }
    }

    if (commandReverse) {
        state = transition(5); // reverse
        targetHeading = wraptopi(forwardHeading + deg2rad(180));
    }


    /***********************************************************
     * Handle state actions
     ***********************************************************/
    float temp_vel_x = 0;
    float temp_vel_y = 0;
    float temp_vel_w = 0;

    if (state == 1) {        //FORWARD
      temp_vel_x = max_speed;
      if (zigzag) {
        commandTurn(&temp_vel_w, zigzagRate * zigzagDirection);
        if (zigzagDirection == 1 && currentHeading > forwardHeading + deg2rad(maxZigzagOffset)) { // reverse direction if further than offset
          zigzagDirection = -1;
        } else if (zigzagDirection == -1 && currentHeading < forwardHeading - deg2rad(maxZigzagOffset)) {
          zigzagDirection = 1;
        }
      }
    } else if (state == 11) {
      if (rightRange < CRITICAL_DISTANCE) {
        temp_vel_y = max_speed;
      } else if (leftRange < CRITICAL_DISTANCE) {
        temp_vel_y = -max_speed;
      }
    } else if (state == 2) { // ROTATE AWAY FROM OBSTACLE
        commandTurn(&temp_vel_w, max_turn_speed * turnDirection);
    } else if (state == 3) { // WALL FOLLOWING
        temp_vel_x = max_speed;
    } else if (state == 4) { // ROTATE BACK TO FORWARD DIRECTION
        commandTurn(&temp_vel_w, max_turn_speed * -turnDirection);
    } else if (state == 5) { // REVERSE
        commandTurn(&temp_vel_w, max_turn_speed);
    }

    *vel_x = temp_vel_x;
    *vel_y = temp_vel_y;
    *vel_w = temp_vel_w;

    return state;
}