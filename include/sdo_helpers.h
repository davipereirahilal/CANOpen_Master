/*
 * sdo_helpers.h
 *
 * A collection of convenience functions for reading and writing values
 * over CANopen SDO.  The CANopenNode library exposes a generic
 * read_SDO() and write_SDO() function which operate on raw byte buffers.
 * These helpers wrap those calls to handle common integer types and take
 * care of the little‑endian byte ordering defined by the CiA402 profile.
 *
 * Each function accepts a pointer to an initialised CO_SDOclient_t, the
 * remote node identifier, and the object index/subindex.  On writes the
 * value is passed directly; on reads the value is returned via a pointer.
 */

#ifndef SDO_HELPERS_H
#define SDO_HELPERS_H

#include "Arduino.h"
#include <stdint.h>
#include <stddef.h>
#include "CANopen.h"


#ifdef __cplusplus
extern "C" {
#endif


/* Forward declarations of the low level SDO accessors.  These are
 * implemented in the existing application code and linked at compile
 * time. */
extern CO_SDO_abortCode_t read_SDO (CO_SDOclient_t* SDO_C, uint8_t nodeId,
                                    uint16_t index, uint8_t subIndex,
                                    uint8_t* buf, size_t bufSize,
                                    size_t* readSize);
extern CO_SDO_abortCode_t write_SDO(CO_SDOclient_t* SDO_C, uint8_t nodeId,
                                    uint16_t index, uint8_t subIndex,
                                    uint8_t* data, size_t dataSize);

/* Reading helpers */
CO_SDO_abortCode_t readSDOUint8 (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                 uint16_t index, uint8_t subIndex, uint8_t* valueOut);
CO_SDO_abortCode_t readSDOInt8  (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                 uint16_t index, uint8_t subIndex, int8_t* valueOut);
CO_SDO_abortCode_t readSDOUint16(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                 uint16_t index, uint8_t subIndex, uint16_t* valueOut);
CO_SDO_abortCode_t readSDOInt16 (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                 uint16_t index, uint8_t subIndex, int16_t* valueOut);
CO_SDO_abortCode_t readSDOUint32(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                 uint16_t index, uint8_t subIndex, uint32_t* valueOut);
CO_SDO_abortCode_t readSDOInt32 (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                 uint16_t index, uint8_t subIndex, int32_t* valueOut);
CO_SDO_abortCode_t readSDOUint64(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                 uint16_t index, uint8_t subIndex, uint64_t* valueOut);
CO_SDO_abortCode_t readSDOInt64 (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                 uint16_t index, uint8_t subIndex, int64_t* valueOut);

/* Writing helpers */
CO_SDO_abortCode_t writeSDOUint8 (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                  uint16_t index, uint8_t subIndex, uint8_t value);
CO_SDO_abortCode_t writeSDOInt8  (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                  uint16_t index, uint8_t subIndex, int8_t value);
CO_SDO_abortCode_t writeSDOUint16(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                  uint16_t index, uint8_t subIndex, uint16_t value);
CO_SDO_abortCode_t writeSDOInt16 (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                  uint16_t index, uint8_t subIndex, int16_t value);
CO_SDO_abortCode_t writeSDOUint32(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                  uint16_t index, uint8_t subIndex, uint32_t value);
CO_SDO_abortCode_t writeSDOInt32 (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                  uint16_t index, uint8_t subIndex, int32_t value);
CO_SDO_abortCode_t writeSDOUint64(CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                  uint16_t index, uint8_t subIndex, uint64_t value);
CO_SDO_abortCode_t writeSDOInt64 (CO_SDOclient_t* sdoClient, uint8_t nodeId,
                                  uint16_t index, uint8_t subIndex, int64_t value);

#ifdef __cplusplus
}
#endif

#endif /* SDO_HELPERS_H */