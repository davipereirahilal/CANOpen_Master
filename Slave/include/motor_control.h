/*
 * motor_control.h
 *
 * High-level helper routines to discover nodes on a CANopen bus and
 * configure basic motion parameters for maxon EPOS/MCP drives.  These
 * functions rely on the lower-level SDO and heartbeat helpers to
 * perform the actual communication.  They may be invoked once the
 * CANopen stack has been initialised and the CAN bus is in normal
 * mode.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <vector>

#include "sdo_helpers.h"
#include "heartbeat_helpers.h"
#include "profile_velocity_mode.h"
#include "can_messages.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Discover nodes on the CANopen bus by sending Node Guard remote frames
 * (COB‑IDs 0x700+node) and waiting for heartbeat responses.  This
 * function uses the global CAN_TX_queue and CAN_RX_queue defined in
 * main.cpp to send and receive TWAI messages.  Only standard 11‑bit
 * identifiers are used.  The function scans node IDs 1..127 and
 * collects those that respond within the given timeout.
 *
 * @param timeout_ms  Amount of time in milliseconds to wait for
 *                    responses after sending queries.
 * @return a vector of node IDs that responded.
 */
std::vector<uint8_t> discoverNodes(uint32_t timeout_ms);

/**
 * Reset and configure a drive in profile velocity mode.  The
 * configuration sequence performs the following steps for the
 * specified node:
 *
 * 1. Set the producer heartbeat time using setProducerHeartbeatTime().
 * 2. Set profile acceleration (0x6083) and deceleration (0x6084)
 *    to the provided values.
 * 3. Select Profile Velocity mode (write 3 to 0x6060).
 * 4. Enable operation by writing the controlword sequence 0x0006
 *    followed by 0x000F (0x6040).
 *
 * On success the function returns true.  Any SDO abort during the
 * sequence will cause the function to return false.
 *
 * @param nodeId          Node ID of the drive to configure.
 * @param heartbeat_ms    Heartbeat period in milliseconds (0 to disable).
 * @param accel           Desired profile acceleration (units depend on drive configuration).
 * @param decel           Desired profile deceleration (units depend on drive configuration).
 * @return true on success, false on SDO failure.
 */
bool resetMotor(uint8_t nodeId, uint16_t heartbeat_ms, uint32_t accel, uint32_t decel);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */