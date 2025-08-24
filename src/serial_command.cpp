/*
 * serial_command.cpp
 *
 * Implementation of a FreeRTOS task for processing serial commands.
 * Commands are read from the default Serial interface (UART0) and
 * may be used to discover devices, reset motors, or perform other
 * actions.  The parser is rudimentary and can be extended to
 * support additional commands.
 */

#include "serial_command.h"

#include <Arduino.h>
#include <vector>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor_control.h"
#include "can_messages.h"
#include "OD.h"

extern SemaphoreHandle_t CAN_ctrl_task_sem;

/* Forward declaration of command handler */
static void processCommand(const String& cmd);

/**
 * Task function that accumulates input from Serial until a newline
 * character is encountered.  When a complete command line is
 * received, it is passed to processCommand() for interpretation.
 */
void serialCommandTask(void* arg) {
    String buffer;
    for (;;) {
        // Read available characters one by one
        while (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                if (buffer.length() > 0) {
                    processCommand(buffer);
                    buffer = "";
                }
            } else {
                buffer += c;
            }
        }
        // Sleep briefly to yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//Function for reading SDOs provided by CANOpenNode within CO_SDOclient.h.
CO_SDO_abortCode_t read_SDO_findnodes(CO_SDOclient_t* SDO_C, uint8_t nodeId, uint16_t index, uint8_t subIndex, uint8_t* buf, size_t bufSize, size_t* readSize, uint16_t timeout_ms) {
	CO_SDO_return_t SDO_ret;
	const unsigned int retryAttempts = 5;

	// setup client (this can be skipped, if remote device don't change)
	SDO_ret = CO_SDOclient_setup(SDO_C,
		CO_CAN_ID_SDO_CLI + nodeId,
		CO_CAN_ID_SDO_SRV + nodeId,
		nodeId);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
		return CO_SDO_AB_GENERAL;
	}

	// initiate upload
	SDO_ret = CO_SDOclientUploadInitiate(SDO_C, index, subIndex, timeout_ms, false);
	if (SDO_ret != CO_SDO_RT_ok_communicationEnd) {
		return CO_SDO_AB_GENERAL;
	}

	do {
		uint32_t timeDifference_us = 1000;
		CO_SDO_abortCode_t abortCode = CO_SDO_AB_NONE;

		SDO_ret = CO_SDOclientUpload(SDO_C,
			timeDifference_us,
			false,
			&abortCode,
			NULL, NULL, NULL);

		if (SDO_ret < 0) {
			CO_SDOclientClose(SDO_C);
			ESP_LOGE("CANSDO_Read", "Abort %s", SDO_ret_to_string(SDO_ret));
			return abortCode;
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	} while (SDO_ret > 0);

	// copy data to the user buffer (for long data function must be called
	// several times inside the loop)
	*readSize = CO_SDOclientUploadBufRead(SDO_C, buf, bufSize);
	CO_SDOclientClose(SDO_C);
	return CO_SDO_AB_NONE;
}

std::vector<uint8_t> NodeScan(uint8_t FirstNode, uint8_t LastNode, uint32_t timeout_ms) {

  std::vector<uint8_t> foundNodes;
  uint8_t buf[8];
  size_t readSize = 0;

  for (uint8_t id = FirstNode; id <= LastNode; ++id) {
    // Tenta ler 0x1018:01 (Vendor ID) do nodeId alvo.
    // Retorna true se o SDO respondeu (nó presente e acessível).
    CO_SDO_abortCode_t ac = read_SDO_findnodes(CO->SDOclient, id, 0x1018, 0x01, buf, sizeof(buf), &readSize, timeout_ms);
    if (ac == CO_SDO_AB_NONE && readSize > 0) {
        foundNodes.push_back(id);
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // small delay to avoid bus overload
  }
  return foundNodes;
}


////////////////////////////////organizar depois//////////////////////////////////////////////////////////
// ---- helpers: parse/format ----
static const char* nmtStateName(uint8_t s) {
  switch (s) {
    case 0x00: return "INITIALIZING/BOOT-UP";
    case 0x7F: return "PRE-OPERATIONAL";
    case 0x05: return "OPERATIONAL";
    case 0x04: return "STOPPED";
    default:   return "UNKNOWN";
  }
}


static void printHex32(uint32_t v) {
  Serial.printf("0x%08lX", (unsigned long)v);
}

// Leitores com lock e checagem de retorno (retornam true/false)
static bool ODreadU8 (uint16_t idx, uint8_t sub, uint8_t*  out) {
  ODR_t odr; OD_entry_t* e = OD_find(OD, idx);
  if (!e) return false;
  CO_LOCK_OD(CO->CANmodule);
  odr = OD_get_u8(e, sub, out, false);
  CO_UNLOCK_OD(CO->CANmodule);
  return odr == ODR_OK;
}

static bool ODreadU16(uint16_t idx, uint8_t sub, uint16_t* out) {
  ODR_t odr; OD_entry_t* e = OD_find(OD, idx);
  if (!e) return false;
  CO_LOCK_OD(CO->CANmodule);
  odr = OD_get_u16(e, sub, out, false);
  CO_UNLOCK_OD(CO->CANmodule);
  return odr == ODR_OK;
}

static bool ODreadU32(uint16_t idx, uint8_t sub, uint32_t* out) {
  ODR_t odr; OD_entry_t* e = OD_find(OD, idx);
  if (!e) return false;
  CO_LOCK_OD(CO->CANmodule);
  odr = OD_get_u32(e, sub, out, false);
  CO_UNLOCK_OD(CO->CANmodule);
  return odr == ODR_OK;
}

// ---- comando principal ----
static void cmd_master_info() {
  // 1) Identificação/NMT
  uint8_t nodeId = CO->NMT->nodeId;
  uint8_t nmtState = (uint8_t)CO->NMT->operatingState;
  Serial.println("======================= MASTER INFO =======================");
  Serial.printf("Node-ID          : %u\n", nodeId);
  Serial.printf("NMT state        : %s (%u)\n", nmtStateName(nmtState), nmtState);

  // 2) Heartbeat Producer (0x1017:00)
  uint16_t hb_ms = 0;
  if (ODreadU16(0x1017, 0x00, &hb_ms)) {
    Serial.printf("HB producer (1017:00): %u ms\n", (unsigned)hb_ms);
  } else {
    Serial.println("HB producer (1017:00): <read error>");
  }

  // 3) Heartbeat Consumer table (0x1016)
  uint8_t nSubs = 0;
  if (ODreadU8(0x1016, 0x00, &nSubs)) {
    Serial.printf("HB consumer (1016:00): %u entries\n", nSubs);
    for (uint8_t si = 1; si <= nSubs; ++si) {
      uint32_t v = 0;
      if (!ODreadU32(0x1016, si, &v)) {
        Serial.printf("  1016:%u = <read error>\n", si);
        continue;
      }
      if (v == 0) {
        // entrada desabilitada
        continue;
      }
      uint8_t  portCode  = (uint8_t)((v >> 24) & 0xFF);
      uint8_t  prodId    = (uint8_t)((v >> 16) & 0xFF);
      uint16_t expTimeMs = (uint16_t)(v & 0xFFFFu);
      Serial.printf("  1016:%u -> producer=%u, expected=%u ms, port=%u\n",
                    si, prodId, (unsigned)expTimeMs, portCode);
    }
  } else {
    Serial.println("HB consumer (1016:00): <read error>");
  }

  // 4) Error Register (0x1001) e Pre-defined Error Field (0x1003)
  uint8_t errReg = 0;
  if (ODreadU8(0x1001, 0x00, &errReg)) {
    Serial.printf("Error register (1001:00): 0x%02X\n", errReg);
  } else {
    Serial.println("Error register (1001:00): <read error>");
  }

  uint8_t errCount = 0;
  if (ODreadU8(0x1003, 0x00, &errCount)) {
    Serial.printf("Error history (1003:00): %u entr%s\n",
                  errCount, errCount==1?"y":"ies");
    uint8_t toShow = errCount > 5 ? 5 : errCount; // limita a 5 pra não poluir
    for (uint8_t i = 1; i <= toShow; ++i) {
      uint32_t entry = 0;
      if (ODreadU32(0x1003, i, &entry)) {
        Serial.printf("  1003:%u = ", i); printHex32(entry); Serial.println();
      } else {
        Serial.printf("  1003:%u = <read error>\n", i);
      }
    }
    if (errCount > 5) Serial.println("  ...");
  } else {
    Serial.println("Error history (1003:00): <read error>");
  }

  // 5) Identity (0x1018)
  uint8_t idSubs = 0;
  if (ODreadU8(0x1018, 0x00, &idSubs) && idSubs >= 4) {
    uint32_t vendor=0, product=0, revision=0, serial=0;
    bool ok1=ODreadU32(0x1018, 0x01, &vendor);
    bool ok2=ODreadU32(0x1018, 0x02, &product);
    bool ok3=ODreadU32(0x1018, 0x03, &revision);
    bool ok4=ODreadU32(0x1018, 0x04, &serial);
    Serial.print("Identity (1018): ");
    if (ok1 && ok2 && ok3 && ok4) {
      Serial.print("Vendor=");   printHex32(vendor);
      Serial.print(", Product="); printHex32(product);
      Serial.print(", Revision=");printHex32(revision);
      Serial.print(", Serial=");  printHex32(serial);
      Serial.println();
    } else {
      Serial.println("<read error>");
    }
  } else {
    Serial.println("Identity (1018): <read error or not implemented>");
  }

  Serial.println("========================================");
}
//////////////////////////////////////////////////////////////////////////////////////////


// Helpers para parse de inteiros com validação (somente decimal)
static bool parseU8 (const String& s, uint8_t&  out, uint8_t  minV, uint8_t  maxV){
  const char* p = s.c_str(); if(*p=='\0') return false;
  char* end=nullptr; unsigned long v = strtoul(p, &end, 10);
  if(end==p || *end!='\0' || v < minV || v > maxV) return false;
  out = (uint8_t)v; return true;
}
static bool parseU16(const String& s, uint16_t& out, uint16_t minV, uint16_t maxV){
  const char* p = s.c_str(); if(*p=='\0') return false;
  char* end=nullptr; unsigned long v = strtoul(p, &end, 10);
  if(end==p || *end!='\0' || v < minV || v > maxV) return false;
  out = (uint16_t)v; return true;
}


/**
 * Interpret and execute a command received over the serial port.
 * Supported commands (example):
 *   discover            – scan the CAN bus for active nodes
 *   reset <id>          – reset and configure the motor with node ID <id>
 *   help                – show available commands
 */
static void processCommand(const String& cmd) {
    const char* delimiters = " \t\r\n";

    String line = cmd;
    line.trim(); // remove espaços/CR/LF nas pontas

    std::vector<String> tokens;
    char* cstr = strdup(line.c_str());
    for (char* tok = strtok(cstr, delimiters); tok; tok = strtok(nullptr, delimiters)) {
        tokens.push_back(String(tok));
    }
    free(cstr);

    if (tokens.empty()) return;

    String command = tokens[0];
    command.toLowerCase();


    static std::vector<uint8_t> nodes;

    if (command == "discover") {
        nodes = NodeScan(1, 125, 30);
        if (nodes.empty()) {
            Serial.println("No nodes found.");
        } else {
            Serial.print("Found nodes: ");
            for (uint8_t id : nodes) {
                Serial.print(id);
                Serial.println(" ");
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // small delay to avoid bus overload
        }
        
    } else if (command == "reset") {
        if (tokens.size() < 2) {
            Serial.println("Usage: reset <nodeId>");
            return;
        }
        uint8_t nodeId = (uint8_t)tokens[1].toInt();
        Serial.print("Resetting and configuring node ");
        Serial.println(nodeId);
        if (resetMotor(nodeId, 100, 10000, 40000)) {
            Serial.println("Node configured successfully.");
        } else {
            Serial.println("Failed to configure node.");
        }


    } else if (command == "heartbeat") {
        uint16_t timeout = 50;
        if(nodes.empty()){
            Serial.println("No nodes found. Use 'discover' command first or connect a device to CAN.");
            return;
        }
        for(uint8_t nodeId : nodes){
            CO_SDO_abortCode_t ac_prod = readProducerHeartbeatTime(CO->SDOclient, nodeId, &timeout);
            if (ac_prod == CO_SDO_AB_NONE) {
                Serial.print("Heartbeat time setted for node ");
                Serial.print(nodeId);
                Serial.print(" is ");
                Serial.print(timeout);
                Serial.println(" ms.");
            } else {
                Serial.print("Failed to read heartbeat time for node ");
                Serial.print(nodeId);
                Serial.print(". Abort code: ");
                Serial.println(ac_prod, HEX);
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // small delay to avoid bus overload
            CO_SDO_abortCode_t ac_cons = readConsumerHeartbeatEntry(CO->SDOclient, nodeId, 1, NULL, NULL, &timeout);// ESTRANHO
            if (ac_cons == CO_SDO_AB_NONE) {
                Serial.print("Node ");
                Serial.print(nodeId);
                Serial.print(" will expect heartbeat every ");
                Serial.print(timeout);
                Serial.println(" ms.");
            } else {
                Serial.print("Failed to read heartbeat consumer entry for node ");
                Serial.print(nodeId);
                Serial.print(". Abort code: ");
                Serial.println(ac_cons, HEX);
            }
        }
    } else if (command == "master_info") {
        cmd_master_info();
        
    } else if (command == "get_master_heartbeat") {
        uint16_t value = 0;
        CO_LOCK_OD(CO->CANmodule);
        OD_get_u16(OD_find(OD, OD_H1017_PRODUCER_HB_TIME), 0x00, &value, false);
        CO_UNLOCK_OD(CO->CANmodule);
        Serial.print("Current master heartbeat time is ");
        Serial.print(value);
        Serial.println(" ms.");
        
    } else if (command == "set_master_heartbeat") {
        // Uso: set_master_heartbeat <ms>
        if (tokens.size() < 2) {
            Serial.println("Usage: set_master_heartbeat <ms>");
            return;
        }
        uint16_t period;
        if (!parseU16(tokens[1], period, 0, 60000)) { // 0 desabilita, 60 s como exemplo
            Serial.println("Invalid <ms> (valid: 0..60000).");
            return;
        }
        // escreve 0x1017:00 (U16) no nó remoto
        CO_LOCK_OD(CO->CANmodule);
        ODR_t odr = OD_set_u16(OD_find(OD, OD_H1017_PRODUCER_HB_TIME), 0x00, period, false);
        CO_UNLOCK_OD(CO->CANmodule);
        if (odr == ODR_OK) {
            Serial.printf("Master: 0x1017 set to %u ms\n", (unsigned)period);
            // Notifica a task de controle para reenviar o heartbeat com o novo tempo
            xSemaphoreGive(CAN_ctrl_task_sem);
        } else {
            Serial.printf("Failed to write 0x1017 in local OD (ODR=%d)\n", odr);
        }

    } else if (command == "get_node_heartbeat") {
        // Uso: get_node_heartbeat <nodeId>
        if (tokens.size() < 2) {
            Serial.println("Usage: get_node_heartbeat <nodeId>");
            return;
        }
        uint8_t nodeId;
        if (!parseU8(tokens[1], nodeId, 1, 127)) {
            Serial.println("Invalid <nodeId> (valid: 1..127).");
            return;
        }
        uint16_t hb_ms = 0;
        CO_SDO_abortCode_t ac = readProducerHeartbeatTime(CO->SDOclient, nodeId, &hb_ms);
        if (ac == CO_SDO_AB_NONE) {
            Serial.printf("Node %u producer heartbeat = %u ms\n", nodeId, (unsigned)hb_ms);
        } else {
            Serial.printf("SDO read 0x1017 from node %u failed (abort=0x%08X)\n",
                        nodeId, (unsigned)ac);
        }

    } else if (command == "set_node_heartbeat") {
        // Uso: set_node_heartbeat <nodeId> <ms>
        if (tokens.size() < 3) {
            Serial.println("Usage: set_node_heartbeat <nodeId> <ms>");
            return;
        }
        uint8_t  nodeId;
        uint16_t period;
        if (!parseU8(tokens[1], nodeId, 1, 127)) {
            Serial.println("Invalid <nodeId> (valid: 1..127).");
            return;
        }
        if (!parseU16(tokens[2], period, 0, 60000)) { // 0 desabilita, 60 s como exemplo
            Serial.println("Invalid <ms> (valid: 0..60000).");
            return;
        }
        // escreve 0x1017:00 (U16) no nó remoto
        CO_SDO_abortCode_t ac = writeSDOUint16(CO->SDOclient, nodeId, 0x1017, 0x00, period);
        if (ac == CO_SDO_AB_NONE) {
            Serial.printf("Node %u: 0x1017 set to %u ms\n", nodeId, (unsigned)period);
        } else {
            Serial.printf("SDO write 0x1017 failed (abort=0x%08X)\n", (unsigned)ac);
        }
    } else if (command == "get_consumer_entry") {
         Serial.println("Command not implemented yet.");
    } else if (command == "set_consumer_entry") {
        Serial.println("Command not implemented yet.");
    } else if (command == "help") {
        Serial.println("Available commands:");
        Serial.println("  discover                    - scan for nodes on the CAN bus");
        Serial.println("  reset <nodeId>              - reset and configure the specified node");
        Serial.println("  heartbeat                   - read HB info for all discovered nodes");
        Serial.println("  get_master_heartbeat        - read this master's heartbeat time (0x1017)");
        Serial.println("  get_node_heartbeat <id>     - read 0x1017 from a remote node");
        Serial.println("  set_node_heartbeat <id> <ms>- write 0x1017 on a remote node");
        Serial.println("  get_consumer_entry <id> <k> - read 0x1016:<k> from a remote node");
        Serial.println("  set_consumer_entry <id> <k> <node> <hb_ms> - write 0x1016:<k> on a remote node");
    } else {
        Serial.print("Unknown command: ");
        Serial.println(command);
        Serial.println("Type 'help' for a list of commands.");
    }
}