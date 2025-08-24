/*
 * heartbeat_helpers.h
 *
 * Convenience functions for configuring and querying the CANopen
 * heartbeat protocol.  In the CANopen specification the heartbeat
 * provides a periodic 'I'm alive' signal from the producer (see
 * object 0x1017) and a consumer mechanism (object 0x1016) to
 * supervise other nodes.  Each function in this header wraps the
 * generic SDO read/write helpers to assemble or decompose the
 * underlying 16‑ and 32‑bit values used by these objects.
 *
 * The producer heartbeat time (0x1017) is a UNSIGNED16 value in
 * milliseconds.  Writing zero disables the heartbeat producer
 * entirely【451871747377351†L1210-L1234】.  The consumer heartbeat
 * object (0x1016) is an array of UNSIGNED32 entries, one per
 * monitored node.  Each entry encodes the port code, node‑ID and
 * expected heartbeat timeout as follows: bits 31‑24 contain the
 * port code, bits 23‑16 the node‑ID of the producer, and bits 15‑0
 * the maximum expected heartbeat time in milliseconds【451871747377351†L1156-L1194】.
 */

#ifndef HEARTBEAT_HELPERS_H
#define HEARTBEAT_HELPERS_H

#include <stdint.h>
#include "CANopen.h"
#include "sdo_helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Port codes used in the Consumer Heartbeat Time entry.  According
 * to the EPOS firmware specification the value 0x01 selects the
 * internal CAN port and 0x02 selects the external accessible
 * CAN port【451871747377351†L1198-L1203】.  Use the appropriate
 * constant for your system configuration. */
#define HEARTBEAT_PORT_INTERNAL 0x01
#define HEARTBEAT_PORT_EXTERNAL 0x02

/*
 * Configure the producer heartbeat time on a remote node.
 *
 * @param sdoClient  An initialised SDO client object
 * @param nodeId     The node identifier of the remote device
 * @param time_ms    Heartbeat period in milliseconds; 0 disables
 *
 * @return SDO abort code (CO_SDO_AB_NONE on success)
 */
CO_SDO_abortCode_t setProducerHeartbeatTime(CO_SDOclient_t* sdoClient,
                                            uint8_t nodeId,
                                            uint16_t time_ms);

/*
 * Read back the producer heartbeat time from a remote node.
 *
 * @param sdoClient  An initialised SDO client object
 * @param nodeId     The node identifier of the remote device
 * @param timeOut    Pointer to store the returned heartbeat period in ms
 *
 * @return SDO abort code (CO_SDO_AB_NONE on success)
 */
CO_SDO_abortCode_t readProducerHeartbeatTime(CO_SDOclient_t* sdoClient,
                                             uint8_t nodeId,
                                             uint16_t* timeOut);

/*
 * Configure a consumer heartbeat entry on a remote node.  The entry
 * combines a port code, the node‑Id of the producer to be
 * supervised and the maximum expected heartbeat period.  The sub
 * index corresponds to the node‑ID of the monitored producer (1…127)
 * as per EPOS firmware specification【451871747377351†L1156-L1187】.
 *
 * If expectedTime_ms is zero or monitorNodeId is zero the entry will
 * be disabled【451871747377351†L1180-L1183】.  Port codes are defined
 * above.
 *
 * @param sdoClient       An initialised SDO client object
 * @param nodeId          The node identifier of the remote device (consumer)
 * @param monitorNodeId   The node‑ID of the heartbeat producer to monitor
 * @param expectedTime_ms Maximum expected heartbeat interval in ms
 * @param portCode        CAN port code (HEARTBEAT_PORT_INTERNAL or EXTERNAL)
 *
 * @return SDO abort code (CO_SDO_AB_NONE on success)
 */
CO_SDO_abortCode_t setConsumerHeartbeatEntry(CO_SDOclient_t* sdoClient,
                                             uint8_t nodeId,
                                             uint8_t monitorNodeId,
                                             uint16_t expectedTime_ms,
                                             uint8_t portCode);

/*
 * Read a consumer heartbeat entry from a remote node.  Parses the
 * 32‑bit entry into its constituent fields.
 *
 * @param sdoClient      An initialised SDO client object
 * @param nodeId         The node identifier of the remote device (consumer)
 * @param monitorNodeId  The node‑ID of the producer whose entry to read
 * @param portCode       Pointer to store the returned port code (may be NULL)
 * @param producerId     Pointer to store the returned producer node‑ID (may be NULL)
 * @param expectedTime   Pointer to store the returned expected time in ms (may be NULL)
 *
 * @return SDO abort code (CO_SDO_AB_NONE on success)
 */
CO_SDO_abortCode_t readConsumerHeartbeatEntry(CO_SDOclient_t* sdoClient,
                                              uint8_t nodeId,
                                              uint8_t monitorNodeId,
                                              uint8_t* portCode,
                                              uint8_t* producerId,
                                              uint16_t* expectedTime);

/*
 * Disable a consumer heartbeat entry.  This is equivalent to writing
 * zero to the entry and clears any supervision of the specified
 * producer.  It is a convenience wrapper around
 * setConsumerHeartbeatEntry() with time=0 and portCode=0.
 */
static inline CO_SDO_abortCode_t clearConsumerHeartbeatEntry(CO_SDOclient_t* sdoClient,
                                                             uint8_t nodeId,
                                                             uint8_t monitorNodeId)
{
    return setConsumerHeartbeatEntry(sdoClient, nodeId, monitorNodeId, 0, 0);
}

#ifdef __cplusplus
}
#endif

#endif /* HEARTBEAT_HELPERS_H */