/*
 * debug_helpers.cpp
 *
 * Implementation of debug logging routines for CiA402 devices.  The
 * functions in this file assist in interpreting the 16‑bit status word
 * returned by the drive (object 0x6041) and provide a simple state
 * decoder based on the CiA402 state machine.  Some bits have mode
 * specific meanings; the profile velocity mode uses bits 10–12 for
 * target reached, internal limit active and speed bit respectively
 * according to documentation【503078929450452†L110-L122】【503078929450452†L110-L119】.
 */

#include "debug_helpers.h"
#include "esp_log.h"

/*
 * Human readable descriptions of each bit in the status word.  This
 * table contains names for the standard CiA402 bits as well as the
 * profile velocity specific ones.  Bits 10 and 11 are used for
 * "Target reached" and "Internal limit active" respectively
 *【503078929450452†L110-L122】.  Bit 12 is used for the speed bit which is set when the
 * actual velocity goes below the velocity threshold【503078929450452†L110-L129】.
 */
static const char* STATUS_BIT_STRINGS[16] = {
    /* bit 0 */ "Ready to switch on",
    /* bit 1 */ "Switched on",
    /* bit 2 */ "Operation enabled",
    /* bit 3 */ "Fault",
    /* bit 4 */ "Voltage enabled",
    /* bit 5 */ "Quick stop",
    /* bit 6 */ "Switch on disabled",
    /* bit 7 */ "Warning",
    /* bit 8 */ "Manufacturer specific",
    /* bit 9 */ "Remote / mode specific",
    /* bit 10 */ "Target reached",      /* profile velocity: set when velocity target is within window【503078929450452†L110-L122】 */
    /* bit 11 */ "Internal limit active",/* profile velocity: max speed/torque limit active【503078929450452†L110-L119】 */
    /* bit 12 */ "Speed bit",           /* profile velocity: indicates standstill when set【503078929450452†L110-L129】 */
    /* bit 13 */ "Operation mode specific",
    /* bit 14 */ "Manufacturer specific",
    /* bit 15 */ "Manufacturer specific"
};

void debugPrintStatusWord(const char* tag, uint16_t statusWord) {
    for (int i = 0; i < 16; i++) {
        if ((statusWord >> i) & 0x1) {
            ESP_LOGI(tag, "Status bit %d set (%s)", i, STATUS_BIT_STRINGS[i]);
        }
    }
}

const char* decodeStateFromStatusWord(uint16_t statusWord) {
    /* Determine CiA402 state from bits 0–3 and 6.  For a full state
     * machine see the CiA402 specification; here we provide a simple
     * mapping for the most common states. */
    bool ready       = (statusWord & (1 << 0)) != 0;
    bool switchedOn  = (statusWord & (1 << 1)) != 0;
    bool operation   = (statusWord & (1 << 2)) != 0;
    bool fault       = (statusWord & (1 << 3)) != 0;
    bool voltage     = (statusWord & (1 << 4)) != 0;
    bool quickStop   = (statusWord & (1 << 5)) != 0;
    bool disabled    = (statusWord & (1 << 6)) != 0;

    if (fault) {
        return "Fault";
    }
    if (disabled) {
        return "Switch on disabled";
    }
    /* According to CiA402, the following combinations of bits denote
     * different states.  See the state machine diagram for details. */
    if (!ready && !switchedOn && !operation) {
        return "Not ready";
    }
    if (ready && !switchedOn && !operation) {
        return "Ready to switch on";
    }
    if (ready && switchedOn && !operation) {
        return "Switched on";
    }
    if (ready && switchedOn && operation) {
        return "Operation enabled";
    }
    return "Unknown";
}