/*
 * profile_velocity_mode.h
 *
 * Helper functions for configuring and commanding a CANopen drive in
 * CiA402 Profile Velocity Mode.  The functions here wrap a number
 * of common SDO writes needed to switch the drive into Profile
 * Velocity mode, set the motion parameters and assign a target
 * velocity.  They rely on the helpers declared in sdo_helpers.h.
 */

#ifndef PROFILE_VELOCITY_MODE_H
#define PROFILE_VELOCITY_MODE_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include "CANopen.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Set the operation mode to Profile Velocity (mode code 3).  Writes
 * value 3 to object 0x6060:00. */
CO_SDO_abortCode_t setModeProfileVelocity(CO_SDOclient_t* sdoClient, uint8_t nodeId);

/* Configure the motion parameters for profile velocity.  The
 * acceleration/deceleration values are drive specific (for example
 * counts per second squared) and the maximum motor speed is used as
 * a limit when calculating velocity commands.
 */
CO_SDO_abortCode_t configureProfileVelocity(CO_SDOclient_t* sdoClient,
                                            uint8_t nodeId,
                                            uint32_t profileAcceleration,
                                            uint32_t profileDeceleration,
                                            uint32_t quickStopDeceleration,
                                            uint32_t maxMotorSpeed);

/* Write a signed 32‑bit target velocity to object 0x60FF:00.  The
 * units depend on the encoder resolution and sampling rate configured
 * on the drive. */
CO_SDO_abortCode_t setTargetVelocity(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                     int32_t targetVelocity);

/* Set or clear the halt bit (bit 8) in the controlword to start or
 * stop motion.  Writing halt=0 starts motion; halt=1 decelerates and
 * stops. */
CO_SDO_abortCode_t setHaltBit(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                              bool halt);

#ifdef __cplusplus
}
#endif

#endif /* PROFILE_VELOCITY_MODE_H */