/*
 * wallfollowing_multirange_onboard.h
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#ifndef SRC_MATTHEW_H_
#define SRC_MATTHEW_H_
#include <stdint.h>
#include <stdbool.h>
#include "drone_variables.h"

int wall_follower3(float *vel_x, float *vel_y, float *vel_w, float front_range, float side_range, float current_heading,
                  int direction_turn);

void adjustDistanceWall3(float distance_wall_new);

void wall_follower3_init(float new_ref_distance_from_wall, float max_speed_ref, int init_state);
#endif /* SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_H_ */
