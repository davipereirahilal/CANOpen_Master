/*
 * serial_command.h
 *
 * Declares a FreeRTOS task for handling user commands over the
 * serial interface.  Commands are sent from a host computer via
 * USB/UART and interpreted on the ESP32 to control the CANopen
 * master.  The actual command parsing logic can be extended as
 * needed.
 */

#ifndef SERIAL_COMMAND_H
#define SERIAL_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif
/**
 * FreeRTOS task that listens on Serial for newline‑terminated
 * commands and dispatches them to appropriate handlers.  Should be
 * started once during setup().
 *
 * Each command is processed in the context of this task and may
 * interact with other tasks or modules (e.g., motor_control).
 */
void serialCommandTask(void* arg);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_COMMAND_H */