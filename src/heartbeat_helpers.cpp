/*
 * heartbeat_helpers.cpp
 *
 * Implementation of helper functions for configuring the CANopen
 * heartbeat objects 0x1016 (Consumer heartbeat time) and 0x1017
 * (Producer heartbeat time).  These functions wrap the generic SDO
 * read/write helpers in sdo_helpers.cpp to assemble the required
 * little‑endian values and interpret the fields defined in the
 * CiA 301 specification and the maxon EPOS firmware specification.
 */

#include "heartbeat_helpers.h"

/*
 * Logging is intentionally omitted from this module to avoid a
 * dependency on the ESP‑IDF logging headers when building on
 * non‑ESP platforms.  If desired, callers can instrument the return
 * values themselves or add their own logging around these calls.
 */

extern "C" {

CO_SDO_abortCode_t setProducerHeartbeatTime(CO_SDOclient_t* sdoClient,
                                            uint8_t nodeId,
                                            uint16_t time_ms)
{
    /* The producer heartbeat time is stored at index 0x1017, sub‑index 0.
     * A value of zero disables the heartbeat producer【451871747377351†L1210-L1234】. */
    return writeSDOUint16(sdoClient, nodeId, 0x1017, 0x00, time_ms);
}

CO_SDO_abortCode_t readProducerHeartbeatTime(CO_SDOclient_t* sdoClient,
                                             uint8_t nodeId,
                                             uint16_t* timeOut)
{
    if (timeOut == NULL) {
        return CO_SDO_AB_GENERAL;
    }
    uint16_t tmp = 0;
    CO_SDO_abortCode_t ac = readSDOUint16(sdoClient, nodeId, 0x1017, 0x00, &tmp);
    if (ac != CO_SDO_AB_NONE) {
        return ac;
    }
    *timeOut = tmp;
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t setConsumerHeartbeatEntry(CO_SDOclient_t* sdoClient,
                                             uint8_t nodeId,
                                             uint8_t monitorNodeId,
                                             uint16_t expectedTime_ms,
                                             uint8_t portCode)
{
    /* Validate arguments: sub-index zero is reserved for the number of
     * entries, so monitored node IDs must start at 1.  If either the
     * producer ID or the time is zero we clear the entry per the
     * specification【451871747377351†L1180-L1183】. */
    uint32_t value = 0;
    if (monitorNodeId != 0 && expectedTime_ms != 0 && portCode != 0) {
        value = ((uint32_t)portCode << 24) |
                ((uint32_t)monitorNodeId << 16) |
                ((uint32_t)expectedTime_ms & 0xFFFFu);
    }
    return writeSDOUint32(sdoClient, nodeId, 0x1016, monitorNodeId, value);
}

CO_SDO_abortCode_t readConsumerHeartbeatEntry(CO_SDOclient_t* sdoClient,
                                              uint8_t nodeId,
                                              uint8_t monitorNodeId,
                                              uint8_t* portCode,
                                              uint8_t* producerId,
                                              uint16_t* expectedTime)
{
    uint32_t value = 0;
    CO_SDO_abortCode_t ac = readSDOUint32(sdoClient, nodeId, 0x1016, monitorNodeId, &value);
    if (ac != CO_SDO_AB_NONE) {
        return ac;
    }
    /* If the entry is zero then the heartbeat supervision is disabled. */
    uint8_t pc = (uint8_t)((value >> 24) & 0xFF);
    uint8_t prodId = (uint8_t)((value >> 16) & 0xFF);
    uint16_t time = (uint16_t)(value & 0xFFFFu);
    if (portCode) {
        *portCode = pc;
    }
    if (producerId) {
        *producerId = prodId;
    }
    if (expectedTime) {
        *expectedTime = time;
    }
    return CO_SDO_AB_NONE;
}

} // extern "C"