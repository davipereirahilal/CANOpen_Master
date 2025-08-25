/*
 * debug_helpers.h
 *
 * Utility functions for inspecting and printing CiA402 status words and
 * other debugging information.  The status word (object 0x6041) is a
 * 16‑bit field containing both state machine and mode specific flags.
 * These helpers can aid in diagnosing motion control problems by
 * decoding and logging the individual bits.
 */

#ifndef DEBUG_HELPERS_H
#define DEBUG_HELPERS_H

#include "Arduino.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Print each set bit of a status word with a descriptive label.  The tag
 * parameter is used as the log tag when printing via ESP_LOGI().
 */
void debugPrintStatusWord(const char* tag, uint16_t statusWord);

/*
 * Decode the high level CiA402 state machine state from the lower bits of
 * the status word.  Returns a human readable string such as "Fault",
 * "Ready to switch on", "Switched on" or "Operation enabled".  If the
 * state cannot be determined a generic "Unknown" string is returned.
 */
const char* decodeStateFromStatusWord(uint16_t statusWord);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_HELPERS_H */