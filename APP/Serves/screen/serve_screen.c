#include "serve_screen.h"

#include <stddef.h>

#include "SCREEN.h"

Serve_Screen_Status_t Serve_Screen_GetCommand(Serve_Screen_Command_t *command)
{
    SCREEN_Command_t screen_command;
    SCREEN_Status_t status;

    if (command == NULL) {
        return SERVE_SCREEN_INVALID_ARGUMENT;
    }

    status = SCREEN_GetCommand(&screen_command);
    if (status == SCREEN_EMPTY) {
        return SERVE_SCREEN_EMPTY;
    }
    if (status == SCREEN_NOT_INITIALIZED) {
        return SERVE_SCREEN_NOT_INITIALIZED;
    }
    if (status != SCREEN_OK) {
        return SERVE_SCREEN_ERROR;
    }

    switch (screen_command) {
    case SCREEN_CMD_HOME:
        *command = SERVE_SCREEN_CMD_HOME;
        break;
    case SCREEN_CMD_PAUSE:
        *command = SERVE_SCREEN_CMD_PAUSE;
        break;
    case SCREEN_CMD_START:
        *command = SERVE_SCREEN_CMD_START;
        break;
    case SCREEN_CMD_SCREEN_EDGE:
        *command = SERVE_SCREEN_CMD_SCREEN_EDGE;
        break;
    case SCREEN_CMD_A4_TARGET:
        *command = SERVE_SCREEN_CMD_A4_TARGET;
        break;
    case SCREEN_CMD_TRACK_START:
        *command = SERVE_SCREEN_CMD_TRACK_START;
        break;
    default:
        return SERVE_SCREEN_ERROR;
    }

    return SERVE_SCREEN_OK;
}

Serve_Screen_Status_t Serve_Screen_UpdateAngles(float horizontal_angle,
                                                 float vertical_angle,
                                                 uint32_t timeout_ms)
{
    SCREEN_Status_t status;

    status = SCREEN_SendAngles(horizontal_angle,
                               vertical_angle,
                               timeout_ms);

    switch (status) {
    case SCREEN_OK:
        return SERVE_SCREEN_OK;
    case SCREEN_BUSY:
        return SERVE_SCREEN_BUSY;
    case SCREEN_TIMEOUT:
        return SERVE_SCREEN_TIMEOUT;
    case SCREEN_INVALID_ARGUMENT:
        return SERVE_SCREEN_INVALID_ARGUMENT;
    case SCREEN_NOT_INITIALIZED:
        return SERVE_SCREEN_NOT_INITIALIZED;
    default:
        return SERVE_SCREEN_ERROR;
    }
}
