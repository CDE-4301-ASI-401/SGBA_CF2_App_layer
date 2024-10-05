#ifndef MOVE_DRONE_CMD_H_
#define MOVE_DRONE_CMD_H_
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

void take_off_hover_land(float *cmdVelX, float *cmdVelY, float *cmdAngW, float timeOuter, float hoverDuration, float takeOffHeight);


#endif /* MOVE_DRONE_CMD_H_ */