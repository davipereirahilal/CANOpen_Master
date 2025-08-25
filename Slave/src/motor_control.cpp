/*
 * motor_control.cpp
 *
 * Implementation of high-level motor control helpers for CANopen
 * drives.  This module provides functions to discover other nodes
 * on the bus via Node Guarding and to initialise a drive into
 * Profile Velocity mode with user-specified parameters.  It assumes
 * that the CANopen stack has been initialised (via CO_main) and that
 * the global FreeRTOS queues CAN_TX_queue and CAN_RX_queue are
 * available for transmitting and receiving raw CAN messages.
 */

#include "motor_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include <algorithm>
#include <cstring>

extern QueueHandle_t CAN_TX_queue;
extern QueueHandle_t CAN_RX_queue;

static const char* TAG_MC = "MOTOR_CTRL";

std::vector<uint8_t> discoverNodes(uint32_t timeout_ms)
{
    std::vector<uint8_t> discovered;
    // Send Node Guard remote frames for node IDs 1..127
    for (uint8_t id = 1; id < 128; ++id) {
        twai_message_t req = {};
        req.identifier = static_cast<uint32_t>(0x700 + id);
        req.extd = 0;      // standard frame
        req.rtr = 1;       // remote transmission request
        req.data_length_code = 0;
        // push into CAN_TX_queue for the CAN_ctrl_task to transmit
        TX_Messages txMsg;
        txMsg.message = req;
        // send with small timeout to avoid blocking
        xQueueSend(CAN_TX_queue, &txMsg, 10);
        // wait a tiny bit between frames to avoid bus overload
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    // Capture responses
    const uint32_t endTick = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < endTick) {
        RX_Messages rxMsg;
        // non-blocking receive
        if (xQueueReceive(CAN_RX_queue, &rxMsg, 0) == pdTRUE) {
            twai_message_t& msg = rxMsg.message;
            // A heartbeat frame has COB-ID 0x700 + nodeId and data[0] contains the state
            if (!msg.extd && !msg.rtr && msg.identifier >= 0x701 && msg.identifier <= 0x77F && msg.data_length_code > 0) {
                uint8_t nodeId = static_cast<uint8_t>(msg.identifier - 0x700);
                if (std::find(discovered.begin(), discovered.end(), nodeId) == discovered.end()) {
                    discovered.push_back(nodeId);
                    ESP_LOGI(TAG_MC, "Discovered node %u", (unsigned)nodeId);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return discovered;
}

bool resetMotor(uint8_t nodeId, uint16_t heartbeat_ms, uint32_t accel, uint32_t decel)
{
    CO_SDOclient_t sdoClient; // allocate a client on the stack
    CO_SDO_abortCode_t ac;
    // 1. Set producer heartbeat time (0x1017)
    ac = setProducerHeartbeatTime(&sdoClient, nodeId, heartbeat_ms);
    if (ac != CO_SDO_AB_NONE) {
        ESP_LOGE(TAG_MC, "Failed to set heartbeat for node %u (abort 0x%X)", (unsigned)nodeId, ac);
        return false;
    }
    // 2. Configure profile acceleration (0x6083) and deceleration (0x6084)
    ac = writeSDOUint32(&sdoClient, nodeId, 0x6083, 0x00, accel);
    if (ac != CO_SDO_AB_NONE) {
        ESP_LOGE(TAG_MC, "Failed to write acceleration (abort 0x%X)", ac);
        return false;
    }
    ac = writeSDOUint32(&sdoClient, nodeId, 0x6084, 0x00, decel);
    if (ac != CO_SDO_AB_NONE) {
        ESP_LOGE(TAG_MC, "Failed to write deceleration (abort 0x%X)", ac);
        return false;
    }
    // Also configure quick stop deceleration (0x6085) equal to decel as a default
    ac = writeSDOUint32(&sdoClient, nodeId, 0x6085, 0x00, decel);
    if (ac != CO_SDO_AB_NONE) {
        ESP_LOGW(TAG_MC, "Failed to write quick stop deceleration (abort 0x%X)", ac);
        // continue even if it fails
    }
    // 3. Select Profile Velocity mode (0x6060 = 3)
    ac = writeSDOInt8(&sdoClient, nodeId, 0x6060, 0x00, 3);
    if (ac != CO_SDO_AB_NONE) {
        ESP_LOGE(TAG_MC, "Failed to set modes_of_operation (abort 0x%X)", ac);
        return false;
    }
    // 4. Enable operation: controlword 0x6040
    // Shutdown (0x0006) then Switch On & Enable Operation (0x000F)
    ac = writeSDOUint16(&sdoClient, nodeId, 0x6040, 0x00, 0x0006);
    if (ac != CO_SDO_AB_NONE) {
        ESP_LOGE(TAG_MC, "Failed to write controlword 0x0006 (abort 0x%X)", ac);
        return false;
    }
    // small delay to allow state transition
    vTaskDelay(pdMS_TO_TICKS(10));
    ac = writeSDOUint16(&sdoClient, nodeId, 0x6040, 0x00, 0x000F);
    if (ac != CO_SDO_AB_NONE) {
        ESP_LOGE(TAG_MC, "Failed to write controlword 0x000F (abort 0x%X)", ac);
        return false;
    }
    ESP_LOGI(TAG_MC, "Node %u configured for Profile Velocity mode", (unsigned)nodeId);
    return true;
}