/*
 * profile_velocity_mode.cpp
 *
 * Implements a minimal set of helper functions for working with the
 * Profile Velocity Mode defined by the CiA402 drive profile.  These
 * helpers show how to switch the mode of operation, set motion
 * parameters and assign a target velocity via SDO.  The halt bit
 * control allows the master to start or stop the motion.  For more
 * advanced motion control consider using PDO mappings.
 */

#include "profile_velocity_mode.h"
#include "sdo_helpers.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "PROFILE_VELOCITY";

/* CiA402 defines mode of operation 3 as Profile Velocity. */
#define OPMODE_PROFILE_VELOCITY 3

CO_SDO_abortCode_t setModeProfileVelocity(CO_SDOclient_t* sdoClient, uint8_t nodeId) {
    return writeSDOInt8(sdoClient, nodeId, 0x6060, 0, (int8_t)OPMODE_PROFILE_VELOCITY);
}

CO_SDO_abortCode_t configureProfileVelocity(CO_SDOclient_t* sdoClient,
                                            uint8_t nodeId,
                                            uint32_t profileAcceleration,
                                            uint32_t profileDeceleration,
                                            uint32_t quickStopDeceleration,
                                            uint32_t maxMotorSpeed) {
    CO_SDO_abortCode_t rc;
    rc = writeSDOUint32(sdoClient, nodeId, 0x6083, 0, profileAcceleration);
    if (rc != CO_SDO_AB_NONE) return rc;
    rc = writeSDOUint32(sdoClient, nodeId, 0x6084, 0, profileDeceleration);
    if (rc != CO_SDO_AB_NONE) return rc;
    rc = writeSDOUint32(sdoClient, nodeId, 0x6085, 0, quickStopDeceleration);
    if (rc != CO_SDO_AB_NONE) return rc;
    rc = writeSDOUint32(sdoClient, nodeId, 0x6080, 0, maxMotorSpeed);
    return rc;
}

CO_SDO_abortCode_t setTargetVelocity(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                     int32_t targetVelocity) {
    return writeSDOInt32(sdoClient, nodeId, 0x60FF, 0, targetVelocity);
}

CO_SDO_abortCode_t setHaltBit(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                              bool halt) {
    /* Read the current controlword */
    uint16_t control = 0;
    CO_SDO_abortCode_t rc = readSDOUint16(sdoClient, nodeId, 0x6040, 0, &control);
    if (rc != CO_SDO_AB_NONE) {
        ESP_LOGE(TAG, "Failed to read controlword from node %u: abort=%lu", (unsigned)nodeId, (unsigned long)rc);
        return rc;
    }
    if (halt) {
        control |= (1U << 8);
    } else {
        control &= ~(1U << 8);
    }
    rc = writeSDOUint16(sdoClient, nodeId, 0x6040, 0, control);
    return rc;
}