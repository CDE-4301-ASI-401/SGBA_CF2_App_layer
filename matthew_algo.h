#ifndef MATTHEW_ALGO_H
#define MATTHEW_ALGO_H

// Define constants for grid size and goal position
#define GRID_SIZE 100
#define GRID_GOAL_X 99
#define GRID_GOAL_Y 99

// Include necessary libraries
#include <stdbool.h>

// Function prototypes

/**
 * @brief Initialize the LRTA* algorithm.
 *
 * @param new_ref_distance_from_goal Distance threshold for decision making.
 * @param max_speed_ref Maximum speed for the drone.
 * @param init_state Initial state of the drone.
 */
void lrta_init(float new_ref_distance_from_goal, float max_speed_ref, int init_state);

/**
 * @brief Heuristic function to estimate the cost to the goal.
 *
 * @param x Current grid x-coordinate.
 * @param y Current grid y-coordinate.
 * @param goal_x Goal's x-coordinate.
 * @param goal_y Goal's y-coordinate.
 * @return Estimated heuristic distance to the goal.
 */
float heuristic_estimate(int x, int y, int goal_x, int goal_y);

/**
 * @brief Calculate the cost of an action based on distance traveled.
 *
 * @param current_cost Current cost in the G map.
 * @param distance_traveled Distance traveled by the drone.
 * @return Updated action cost.
 */
float action_cost(float current_cost, float distance_traveled);

/**
 * @brief Transition the drone to a new state based on LRTA* learning.
 *
 * @param new_state The new state to transition to.
 * @param front_range The front sensor's range.
 * @param side_range The side sensor's range.
 * @return Updated state after transition.
 */
int lrta_transition(int new_state, float front_range, float side_range);

/**
 * @brief Control function for the drone's movement based on LRTA*.
 *
 * @param vel_x Pointer to velocity in the x direction.
 * @param vel_y Pointer to velocity in the y direction.
 * @param vel_w Pointer to rotational velocity.
 * @param front_range The front sensor's range.
 * @param side_range The side sensor's range.
 * @param current_heading The current heading direction of the drone.
 * @param direction_turn The direction in which to turn.
 * @return Updated state after control action.
 */
int lrta_drone_control(float *vel_x, float *vel_y, float *vel_w, float front_range, float side_range, float current_heading, int direction_turn);

/**
 * @brief Utility function to calculate distance between two grid points.
 *
 * @param x1 First point x-coordinate.
 * @param y1 First point y-coordinate.
 * @param x2 Second point x-coordinate.
 * @param y2 Second point y-coordinate.
 * @return Distance between the two points.
 */
float calculate_distance(int x1, int y1, int x2, int y2);

/**
 * @brief Command the drone to hover in place.
 *
 * @param vel_x Pointer to velocity in the x direction.
 * @param vel_y Pointer to velocity in the y direction.
 * @param vel_w Pointer to rotational velocity.
 */
void commandHover(float *vel_x, float *vel_y, float *vel_w);

/**
 * @brief Command the drone to perform a turn.
 *
 * @param vel_x Pointer to velocity in the x direction.
 * @param vel_w Pointer to rotational velocity.
 * @param max_rate Maximum turning rate.
 */
void commandTurn(float *vel_x, float *vel_w, float max_rate);

// Directional vectors for grid navigation
extern int dx[4]; // Direction x offsets (for example, {-1, 1, 0, 0})
extern int dy[4]; // Direction y offsets (for example, {0, 0, -1, 1})

#endif // MATTHEW_ALGO_H
