/*
 * can_messages.h
 *
 * Defines helper structures for passing TWAI messages through FreeRTOS
 * queues.  Both the transmit and receive queues use the same
 * structure, wrapping a single twai_message_t.  Defining these in a
 * header ensures consistent use across modules.
 */

#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

#include "driver/twai.h"

/**
 * Wrapper for TWAI messages used in the CAN_TX_queue and CAN_RX_queue.
 */
typedef struct CANMessages {
    twai_message_t message;
} TX_Messages, RX_Messages;

#endif /* CAN_MESSAGES_H */