#include "wallfollowing_multiranger_onboard.h"
#define __USE_MISC
#include <math.h>

// variables
static float ref_distance_from_wall;
static float max_speed;
static float max_rate;
static float direction;
static float first_run;
static int state;
static float state_start_time;
static float x_position;
static float y_position;
static float height;