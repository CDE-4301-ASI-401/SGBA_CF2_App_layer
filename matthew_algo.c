#include "matthew_algo.h"
#define __USE_MISC
#include <math.h>
#include "usec_time.h"
#include "debug.h"

// variables
static float ref_distance_from_goal = 0;
static float max_speed = 0.5;
static float max_rate = 0.5;
static float direction = 1;
static bool first_run = true;
static int state = 1;
static float state_start_time;
static float H[GRID_SIZE][GRID_SIZE];  // Heuristic map, assuming grid-based environment
static float G[GRID_SIZE][GRID_SIZE];  // Cost map for the path
static int grid_x = 0, grid_y = 0;     // Current grid coordinates
static int goal_x = GRID_GOAL_X, goal_y = GRID_GOAL_Y; // Goal coordinates

void lrta_init(float new_ref_distance_from_goal, float max_speed_ref, int init_state)
{
    ref_distance_from_goal = new_ref_distance_from_goal;
    max_speed = max_speed_ref;
    first_run = true;
    state = init_state;

    // Initialize heuristic and cost arrays
    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            H[i][j] = heuristic_estimate(i, j, goal_x, goal_y);  // Estimate heuristic to the goal
            G[i][j] = INFINITY;  // Initialize the cost as infinite
        }
    }

    G[grid_x][grid_y] = 0;  // Starting point cost is 0
}

// Heuristic function: Example with Euclidean distance
float heuristic_estimate(int x, int y, int goal_x, int goal_y) {
    return sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
}

// Static helper function for action cost computation
float action_cost(float current_cost, float distance_traveled) {
    return current_cost + distance_traveled;
}

// State transition based on LRTA* learning
int lrta_transition(int new_state, float front_range, float side_range) {
    float t = usecTimestamp() / 1e6;
    state_start_time = t;

    // Update heuristic for current state
    float min_cost = INFINITY;
    for (int i = 0; i < 4; i++) {  // Check 4 possible directions (front, back, left, right)
        int next_x = grid_x + dx[i];  // Change this with your grid directions
        int next_y = grid_y + dy[i];
        float cost = action_cost(G[grid_x][grid_y], calculate_distance(next_x, next_y));

        if (cost < min_cost) {
            min_cost = cost;
        }
    }

    // Update heuristic to the minimum found
    H[grid_x][grid_y] = min_cost;

    // Transition to new state based on sensor readings and heuristic
    if (front_range < ref_distance_from_goal) {
        return 3; // Turn to find goal
    } else if (side_range > ref_distance_from_goal) {
        return 5; // Move forward along the path
    }

    return new_state; // Continue in the current state
}

int lrta_drone_control(float *vel_x, float *vel_y, float *vel_w, float front_range, float side_range, float current_heading, int direction_turn) {
    direction = direction_turn;

    if (first_run) {
        first_run = false;
    }

    /***********************************************************
     * State definitions
     ***********************************************************/
    // 1 = explore
    // 2 = hover
    // 3 = turn_to_find_goal
    // 4 = move_toward_goal

    /***********************************************************
     * Handle state transitions based on heuristic
     ***********************************************************/
    state = lrta_transition(state, front_range, side_range);

    /***********************************************************
     * Handle state actions
     ***********************************************************/
    float temp_vel_x = 0;
    float temp_vel_y = 0;
    float temp_vel_w = 0;

    if (state == 1) {  // Explore
        DEBUG_PRINT("EXPLORING | ");
        temp_vel_x = max_speed;
        temp_vel_w = direction * max_rate;
    } else if (state == 2) {  // Hover
        DEBUG_PRINT("HOVERING | ");
        commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
    } else if (state == 3) {  // Turn to find goal
        DEBUG_PRINT("TURNING TO FIND GOAL | ");
        commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
    } else if (state == 4) {  // Move toward goal
        DEBUG_PRINT("MOVING TOWARD GOAL | ");
        temp_vel_x = max_speed;
        temp_vel_w = 0;
    }

    *vel_x = temp_vel_x;
    *vel_y = temp_vel_y;
    *vel_w = temp_vel_w;

    return state;
}
