#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "radiolink.h"
#include "crtp.h"
#include "commander.h"
#include "range.h"

#define vTaskDelay(ms) usleep((ms) * 1000)
#define xTaskGetTickCount() ((uint32_t)clock() * 1000 / CLOCKS_PER_SEC)

#define TAKEOFF_COMMAND 0x63
#define TAKEOFF_HEIGHT 1.5f
#define FORWARD_SPEED 0.5f
#define FORWARD_DURATION 5.0f
#define CRITICAL_DISTANCE 0.5f

static void take_off(setpoint_t *sp, float velocity) {
    sp->mode.z = modeVelocity;
    sp->velocity.z = velocity;
    commanderSetSetpoint(sp, 1);
}

static void land(setpoint_t *sp, float velocity) {
    sp->mode.z = modeVelocity;
    sp->velocity.z = -velocity;
    commanderSetSetpoint(sp, 4);
    vTaskDelay(3000);
}

static void fly_forward_then_avoid(void) {
    setpoint_t sp = {0};
    take_off(&sp, 0.5f);
    vTaskDelay(2000);

    while ((float)rangeGet(rangeDown) < TAKEOFF_HEIGHT * 0.8f || 
           (float)rangeGet(rangeDown) > TAKEOFF_HEIGHT * 1.2f) {
        vTaskDelay(100);
    }

    uint32_t startTime = xTaskGetTickCount();
    while ((xTaskGetTickCount() - startTime) < (FORWARD_DURATION * 1000)) {
        sp.velocity.x = FORWARD_SPEED;
        commanderSetSetpoint(&sp, 2);
        vTaskDelay(100);
        if ((float)rangeGet(rangeUp) < 0.15f) break;
    }
    land(&sp, 0.1f);
}

static void processCommand(uint8_t *data, uint8_t len) {
    if (len >= 5 && data[0] == 0xff && data[1] == 0x80 && data[2] == TAKEOFF_COMMAND) {
        printf("Takeoff command received!\n");
        fly_forward_then_avoid();
    }
}

static void radioTask(void *param) {
    CRTPPacket packet;
    while (true) {
        if (radiolinkReceivePacket(&packet) > 0) {
            processCommand(packet.data, packet.size);
        }
        vTaskDelay(10);
    }
}

void appMain(void) {
    xTaskCreate(radioTask, "RadioListener", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    fly_forward_then_avoid();
}
