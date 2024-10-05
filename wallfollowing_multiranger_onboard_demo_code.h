/*
 * wallfollowing_multiranger_onboard copy_demo_code.h
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#ifndef SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_DEMO_CODE_H_
#define SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_DEMO_CODE_H_
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    forward,
    hover,
    turnToFindWall,
    turnToAlignToWall,
    forwardAlongWall,
    rotateAroundWall,
    rotateInCorner,
    findCorner
    take_off,
    land
} StateWF;

StateWF wallFollower(float *cmdVelX, float *cmdVelY, float *cmdAngW, float frontRange, float sideRange, float currentHeading,
                     int directionTurn, float timeOuter);

void adjustDistanceWall(float distanceWallNew);

void wallFollowerInit(float refDistanceFromWallNew, float maxForwardSpeed_ref, StateWF initState);
#endif /* SRC_WALLFOLLOWING_MULTIRANGER_ONBOARD_DEMO_CODE_H_ */
