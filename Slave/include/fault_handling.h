/*
 * fault_handling.h
 *
 * Provides helper functions to detect and recover from fault conditions
 * reported by a CiA402 drive.  A fault is indicated by bit 3 of the
 * status word (0x6041).  To clear a fault the master must send a
 * controlword (0x6040) with the Fault Reset bit (bit 7) set.  After
 * resetting the fault the drive transitions to the "Ready to switch on"
 * state and can be enabled again.  See the CiA402 state machine for
 * further details.
 */

#ifndef FAULT_HANDLING_H
#define FAULT_HANDLING_H

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>
#include "CANopen.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Return true if the fault bit (bit 3) is set in the status word. */
bool isFaultActive(uint16_t statusWord);

/*
 * Clear the fault in the remote drive.  This sends a controlword
 * (index 0x6040, subindex 0) with bit 7 set (0x80) which instructs
 * the slave to reset the fault according to CiA402.  The caller must
 * supply an initialised SDO client and the target node identifier.
 */
CO_SDO_abortCode_t clearFault(CO_SDOclient_t* sdoClient, uint8_t nodeId);

/*
 * Convenience function which reads the status word of the drive
 * (object 0x6041:00) and automatically calls clearFault() if a fault
 * is detected.  Returns the abort code from the last SDO transaction.
 */
CO_SDO_abortCode_t checkAndClearFault(CO_SDOclient_t* sdoClient, uint8_t nodeId);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_HANDLING_H */