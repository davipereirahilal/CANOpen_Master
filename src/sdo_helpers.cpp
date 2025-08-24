/*
 * sdo_helpers.cpp
 *
 * Implementation of helper routines for reading and writing SDO objects
 * with common integral types.  These wrappers call the generic read_SDO()
 * and write_SDO() functions provided by the application code and handle
 * conversion between types and the little‑endian representation used by
 * CANopen.  They return the same CO_SDO_abortCode_t values as the
 * underlying functions.
 */

#include "sdo_helpers.h"
#include "freertos/task.h"
#include "esp_log.h"

extern "C" {


static const char* TAG = "SDO_HELPERS";

/*
 * The following functions implement reads and writes of common integral
 * types without the use of macros.  Values are encoded and decoded
 * explicitly as little‑endian byte sequences as specified by the
 * CiA402 profile.  On write the value is placed into a temporary
 * buffer and passed to write_SDO().  On read a buffer is filled via
 * read_SDO(), the size is checked and the bytes are assembled into
 * the output variable.
 */

CO_SDO_abortCode_t writeSDOUint8(CO_SDOclient_t* sdoClient,
                                 uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, uint8_t value) {
    /* 8‑bit values require no byte order conversion. */
    return write_SDO(sdoClient, nodeId, index, subIndex, &value, sizeof(value));
}

CO_SDO_abortCode_t writeSDOInt8(CO_SDOclient_t* sdoClient,
                                uint8_t nodeId, uint16_t index,
                                uint8_t subIndex, int8_t value) {
    return write_SDO(sdoClient, nodeId, index, subIndex, (uint8_t*)&value, sizeof(value));
}

CO_SDO_abortCode_t writeSDOUint16(CO_SDOclient_t* sdoClient,
                                  uint8_t nodeId, uint16_t index,
                                  uint8_t subIndex, uint16_t value) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(value & 0x00FF);
    buf[1] = (uint8_t)((value >> 8) & 0x00FF);
    return write_SDO(sdoClient, nodeId, index, subIndex, buf, sizeof(buf));
}

CO_SDO_abortCode_t writeSDOInt16(CO_SDOclient_t* sdoClient,
                                 uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, int16_t value) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(value & 0x00FF);
    buf[1] = (uint8_t)(((uint16_t)value >> 8) & 0x00FF);
    return write_SDO(sdoClient, nodeId, index, subIndex, buf, sizeof(buf));
}

CO_SDO_abortCode_t writeSDOUint32(CO_SDOclient_t* sdoClient,
                                  uint8_t nodeId, uint16_t index,
                                  uint8_t subIndex, uint32_t value) {
    uint8_t buf[4];
    buf[0] = (uint8_t)((value >>  0) & 0xFF);
    buf[1] = (uint8_t)((value >>  8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
    return write_SDO(sdoClient, nodeId, index, subIndex, buf, sizeof(buf));
}

CO_SDO_abortCode_t writeSDOInt32(CO_SDOclient_t* sdoClient,
                                 uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, int32_t value) {
    uint8_t buf[4];
    buf[0] = (uint8_t)((value >>  0) & 0xFF);
    buf[1] = (uint8_t)((value >>  8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
    return write_SDO(sdoClient, nodeId, index, subIndex, buf, sizeof(buf));
}

CO_SDO_abortCode_t writeSDOUint64(CO_SDOclient_t* sdoClient,
                                  uint8_t nodeId, uint16_t index,
                                  uint8_t subIndex, uint64_t value) {
    uint8_t buf[8];
    buf[0] = (uint8_t)((value >>  0) & 0xFF);
    buf[1] = (uint8_t)((value >>  8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
    buf[4] = (uint8_t)((value >> 32) & 0xFF);
    buf[5] = (uint8_t)((value >> 40) & 0xFF);
    buf[6] = (uint8_t)((value >> 48) & 0xFF);
    buf[7] = (uint8_t)((value >> 56) & 0xFF);
    return write_SDO(sdoClient, nodeId, index, subIndex, buf, sizeof(buf));
}

CO_SDO_abortCode_t writeSDOInt64(CO_SDOclient_t* sdoClient,
                                 uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, int64_t value) {
    uint8_t buf[8];
    buf[0] = (uint8_t)((value >>  0) & 0xFF);
    buf[1] = (uint8_t)((value >>  8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
    buf[4] = (uint8_t)((value >> 32) & 0xFF);
    buf[5] = (uint8_t)((value >> 40) & 0xFF);
    buf[6] = (uint8_t)((value >> 48) & 0xFF);
    buf[7] = (uint8_t)((value >> 56) & 0xFF);
    return write_SDO(sdoClient, nodeId, index, subIndex, buf, sizeof(buf));
}


/* Reading functions */

CO_SDO_abortCode_t readSDOUint8(CO_SDOclient_t* sdoClient,
                                uint8_t nodeId, uint16_t index,
                                uint8_t subIndex, uint8_t* valueOut) {
    uint8_t buf[1];
    size_t readSize = 0;
    CO_SDO_abortCode_t rc = read_SDO(sdoClient, nodeId, index, subIndex,
                                     buf, sizeof(buf), &readSize);
    if (rc != CO_SDO_AB_NONE) return rc;
    if (readSize < 1) return CO_SDO_AB_GENERAL;
    *valueOut = buf[0];
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t readSDOInt8(CO_SDOclient_t* sdoClient,
                               uint8_t nodeId, uint16_t index,
                               uint8_t subIndex, int8_t* valueOut) {
    uint8_t buf[1];
    size_t readSize = 0;
    CO_SDO_abortCode_t rc = read_SDO(sdoClient, nodeId, index, subIndex,
                                     buf, sizeof(buf), &readSize);
    if (rc != CO_SDO_AB_NONE) return rc;
    if (readSize < 1) return CO_SDO_AB_GENERAL;
    *valueOut = (int8_t)buf[0];
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t readSDOUint16(CO_SDOclient_t* sdoClient,
                                 uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, uint16_t* valueOut) {
    uint8_t buf[2] = {0};
    size_t readSize = 0;
    CO_SDO_abortCode_t rc = read_SDO(sdoClient, nodeId, index, subIndex,
                                     buf, sizeof(buf), &readSize);
    if (rc != CO_SDO_AB_NONE) return rc;
    if (readSize < 2) return CO_SDO_AB_GENERAL;
    uint16_t result = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    *valueOut = result;
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t readSDOInt16(CO_SDOclient_t* sdoClient,
                                uint8_t nodeId, uint16_t index,
                                uint8_t subIndex, int16_t* valueOut) {
    uint8_t buf[2] = {0};
    size_t readSize = 0;
    CO_SDO_abortCode_t rc = read_SDO(sdoClient, nodeId, index, subIndex,
                                     buf, sizeof(buf), &readSize);
    if (rc != CO_SDO_AB_NONE) return rc;
    if (readSize < 2) return CO_SDO_AB_GENERAL;
    uint16_t temp = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    *valueOut = (int16_t)temp;
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t readSDOUint32(CO_SDOclient_t* sdoClient,
                                 uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, uint32_t* valueOut) {
    uint8_t buf[4] = {0};
    size_t readSize = 0;
    CO_SDO_abortCode_t rc = read_SDO(sdoClient, nodeId, index, subIndex,
                                     buf, sizeof(buf), &readSize);
    if (rc != CO_SDO_AB_NONE) return rc;
    if (readSize < 4) return CO_SDO_AB_GENERAL;
    uint32_t result = ((uint32_t)buf[0]) |
                      ((uint32_t)buf[1] << 8) |
                      ((uint32_t)buf[2] << 16) |
                      ((uint32_t)buf[3] << 24);
    *valueOut = result;
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t readSDOInt32(CO_SDOclient_t* sdoClient,
                                uint8_t nodeId, uint16_t index,
                                uint8_t subIndex, int32_t* valueOut) {
    uint8_t buf[4] = {0};
    size_t readSize = 0;
    CO_SDO_abortCode_t rc = read_SDO(sdoClient, nodeId, index, subIndex,
                                     buf, sizeof(buf), &readSize);
    if (rc != CO_SDO_AB_NONE) return rc;
    if (readSize < 4) return CO_SDO_AB_GENERAL;
    uint32_t temp = ((uint32_t)buf[0]) |
                    ((uint32_t)buf[1] << 8) |
                    ((uint32_t)buf[2] << 16) |
                    ((uint32_t)buf[3] << 24);
    *valueOut = (int32_t)temp;
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t readSDOUint64(CO_SDOclient_t* sdoClient,
                                 uint8_t nodeId, uint16_t index,
                                 uint8_t subIndex, uint64_t* valueOut) {
    uint8_t buf[8] = {0};
    size_t readSize = 0;
    CO_SDO_abortCode_t rc = read_SDO(sdoClient, nodeId, index, subIndex,
                                     buf, sizeof(buf), &readSize);
    if (rc != CO_SDO_AB_NONE) return rc;
    if (readSize < 8) return CO_SDO_AB_GENERAL;
    uint64_t result = ((uint64_t)buf[0]) |
                      ((uint64_t)buf[1] << 8) |
                      ((uint64_t)buf[2] << 16) |
                      ((uint64_t)buf[3] << 24) |
                      ((uint64_t)buf[4] << 32) |
                      ((uint64_t)buf[5] << 40) |
                      ((uint64_t)buf[6] << 48) |
                      ((uint64_t)buf[7] << 56);
    *valueOut = result;
    return CO_SDO_AB_NONE;
}

CO_SDO_abortCode_t readSDOInt64(CO_SDOclient_t* sdoClient,
                                uint8_t nodeId, uint16_t index,
                                uint8_t subIndex, int64_t* valueOut) {
    uint8_t buf[8] = {0};
    size_t readSize = 0;
    CO_SDO_abortCode_t rc = read_SDO(sdoClient, nodeId, index, subIndex,
                                     buf, sizeof(buf), &readSize);
    if (rc != CO_SDO_AB_NONE) return rc;
    if (readSize < 8) return CO_SDO_AB_GENERAL;
    uint64_t temp = ((uint64_t)buf[0]) |
                    ((uint64_t)buf[1] << 8) |
                    ((uint64_t)buf[2] << 16) |
                    ((uint64_t)buf[3] << 24) |
                    ((uint64_t)buf[4] << 32) |
                    ((uint64_t)buf[5] << 40) |
                    ((uint64_t)buf[6] << 48) |
                    ((uint64_t)buf[7] << 56);
    *valueOut = (int64_t)temp;
    return CO_SDO_AB_NONE;
}

} /* extern "C" */