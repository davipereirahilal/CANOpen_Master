/*
 * fault_handling.cpp
 *
 * Implementation of routines to detect and resolve fault states on
 * CiA402 compliant drives.  A fault is signalled by bit 3 of the
 * status word.  Clearing the fault requires writing a controlword
 * with the Fault Reset bit set.  The helpers here provide a simple
 * abstraction over these operations using the sdo_helpers API.
 */

#include "fault_handling.h"
#include "sdo_helpers.h"
#include "debug_helpers.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "FAULT_HANDLING";

bool isFaultActive(uint16_t statusWord) {
    /* In the CiA402 status word the fault bit is bit 3. */
    return (statusWord & (1U << 3)) != 0;
}

CO_SDO_abortCode_t clearFault(CO_SDOclient_t* sdoClient, uint8_t nodeId) {
    /* Set only bit 7 (0x80) of the controlword to reset the fault.  This
     * transitions the drive from Fault back to Ready to switch on.
     */
    return writeSDOUint16(sdoClient, nodeId, 0x6040, 0, (uint16_t)0x0080);
}

CO_SDO_abortCode_t checkAndClearFault(CO_SDOclient_t* sdoClient, uint8_t nodeId) {
    uint16_t status = 0;
    CO_SDO_abortCode_t rc = readSDOUint16(sdoClient, nodeId, 0x6041, 0, &status);
    if (rc != CO_SDO_AB_NONE) {
        ESP_LOGE(TAG, "Failed to read statusword from node %u: abort=%lu", (unsigned)nodeId, (unsigned long)rc);
        return rc;
    }
    if (isFaultActive(status)) {
        ESP_LOGW(TAG, "Fault detected (status=0x%04X). Attempting to reset...", status);
        rc = clearFault(sdoClient, nodeId);
        if (rc != CO_SDO_AB_NONE) {
            ESP_LOGE(TAG, "Failed to send fault reset: abort=%lu", (unsigned long)rc);
        } else {
            ESP_LOGI(TAG, "Fault reset command sent successfully");
        }
        return rc;
    }
    return CO_SDO_AB_NONE;
}