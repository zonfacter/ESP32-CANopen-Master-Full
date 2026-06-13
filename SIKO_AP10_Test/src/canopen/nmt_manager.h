/**
 * @file nmt_manager.h
 * @brief CANopen NMT Zustandsmaschine
 * @project SIKO AP10 Testprojekt
 * @date 2025
 *
 * Verwaltet den NMT-Zustand eines CANopen-Knotens.
 * Erkennt Boot-Up, sendet Start/Stop/Reset-Kommandos,
 * überwacht den Heartbeat.
 *
 * NMT-Zustände:
 *   INITIALISATION → PRE-OPERATIONAL → OPERATIONAL
 *                                    → STOPPED
 *
 * SPEICHERN ALS: src/canopen/nmt_manager.h
 */

#ifndef NMT_MANAGER_H
#define NMT_MANAGER_H

#include <Arduino.h>
#include "canopen_driver.h"

// ============================================================================
// NMT Zustände
// ============================================================================

enum NmtState {
    NMT_UNKNOWN         = 0x00,
    NMT_INITIALISATION  = 0x01,  // Interner Zustand beim Boot
    NMT_PRE_OPERATIONAL = 0x7F,  // Nach Boot-Up (AP10 Standardzustand)
    NMT_OPERATIONAL     = 0x05,
    NMT_STOPPED         = 0x04
};

// NMT Kommandos (Byte 0 im NMT-Frame)
enum NmtCommand {
    NMT_CMD_START         = 0x01,  // → OPERATIONAL
    NMT_CMD_STOP          = 0x02,  // → STOPPED
    NMT_CMD_PRE_OP        = 0x80,  // → PRE-OPERATIONAL
    NMT_CMD_RESET_NODE    = 0x81,  // Reset Applikation
    NMT_CMD_RESET_COMM    = 0x82   // Reset Kommunikation
};

// ============================================================================
// NMT Manager Klasse
// ============================================================================

class NmtManager {
private:
    CanopenDriver* m_driver;
    uint8_t        m_nodeId;

    NmtState m_state;
    bool     m_bootUpReceived;
    uint32_t m_lastHeartbeat;
    uint32_t m_heartbeatTimeout;  // ms, 0 = kein Monitoring

    // Callback bei Zustandsänderung
    std::function<void(NmtState)> m_stateCallback;

public:
    NmtManager(CanopenDriver* driver, uint8_t nodeId = 125)
        : m_driver(driver)
        , m_nodeId(nodeId)
        , m_state(NMT_UNKNOWN)
        , m_bootUpReceived(false)
        , m_lastHeartbeat(0)
        , m_heartbeatTimeout(3000)
        , m_stateCallback(nullptr)
    {}

    void setStateCallback(std::function<void(NmtState)> cb) {
        m_stateCallback = cb;
    }

    void setHeartbeatTimeout(uint32_t ms) { m_heartbeatTimeout = ms; }
    void setNodeId(uint8_t id)            { m_nodeId = id; }
    uint8_t getNodeId()             const { return m_nodeId; }
    NmtState getState()             const { return m_state; }
    bool isBootUpReceived()         const { return m_bootUpReceived; }
    bool isOperational()            const { return m_state == NMT_OPERATIONAL; }

    // -----------------------------------------------------------------------
    // Eingehende CAN-Frames verarbeiten
    // Muss aus dem RX-Callback aufgerufen werden
    // -----------------------------------------------------------------------
    void processFrame(uint32_t cobId, const uint8_t* data, uint8_t length) {
        // Heartbeat / Boot-Up COB-ID = 0x700 + Node-ID
        uint32_t heartbeatCobId = 0x700 + m_nodeId;

        if (cobId == heartbeatCobId && length >= 1) {
            m_lastHeartbeat = millis();
            uint8_t stateRaw = data[0];

            if (stateRaw == 0x00) {
                // Boot-Up Nachricht!
                Serial.printf("[NMT] Boot-Up empfangen von Node %d\n", m_nodeId);
                m_bootUpReceived = true;
                setState(NMT_PRE_OPERATIONAL);
            } else {
                // Heartbeat mit Zustandsbyte
                NmtState newState = (NmtState)stateRaw;
                if (newState != m_state) {
                    Serial.printf("[NMT] Zustand geändert: 0x%02X\n", stateRaw);
                    setState(newState);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // NMT Kommandos senden
    // -----------------------------------------------------------------------

    /**
     * @brief Sendet NMT Start → AP10 wechselt in OPERATIONAL
     * Danach sendet AP10 TPDO1 (Positionswert)
     */
    bool sendStart() {
        Serial.printf("[NMT] Sende START an Node %d\n", m_nodeId);
        return sendNmtCommand(NMT_CMD_START);
    }

    bool sendStop() {
        Serial.printf("[NMT] Sende STOP an Node %d\n", m_nodeId);
        return sendNmtCommand(NMT_CMD_STOP);
    }

    bool sendPreOperational() {
        Serial.printf("[NMT] Sende PRE-OPERATIONAL an Node %d\n", m_nodeId);
        return sendNmtCommand(NMT_CMD_PRE_OP);
    }

    bool sendResetNode() {
        Serial.printf("[NMT] Sende RESET NODE an Node %d\n", m_nodeId);
        m_bootUpReceived = false;
        m_state = NMT_UNKNOWN;
        return sendNmtCommand(NMT_CMD_RESET_NODE);
    }

    bool sendResetComm() {
        Serial.printf("[NMT] Sende RESET COMM an Node %d\n", m_nodeId);
        return sendNmtCommand(NMT_CMD_RESET_COMM);
    }

    // -----------------------------------------------------------------------
    // Heartbeat-Überwachung (im loop() aufrufen)
    // -----------------------------------------------------------------------
    void update() {
        if (m_heartbeatTimeout == 0) return;
        if (m_lastHeartbeat == 0)    return;  // Noch kein Heartbeat empfangen

        uint32_t age = millis() - m_lastHeartbeat;
        if (age > m_heartbeatTimeout) {
            if (m_state != NMT_UNKNOWN) {
                Serial.printf("[NMT] Heartbeat Timeout! Letzter: %lu ms\n", age);
                setState(NMT_UNKNOWN);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Heartbeat-Alter in ms
    // -----------------------------------------------------------------------
    uint32_t getHeartbeatAge() const {
        if (m_lastHeartbeat == 0) return 0xFFFFFFFF;
        return millis() - m_lastHeartbeat;
    }

    const char* getStateName() const {
        switch (m_state) {
            case NMT_INITIALISATION:  return "INITIALISATION";
            case NMT_PRE_OPERATIONAL: return "PRE-OPERATIONAL";
            case NMT_OPERATIONAL:     return "OPERATIONAL";
            case NMT_STOPPED:         return "STOPPED";
            default:                  return "UNBEKANNT";
        }
    }

private:
    bool sendNmtCommand(NmtCommand cmd) {
        // NMT COB-ID = 0x000, Byte0 = Kommando, Byte1 = Node-ID (0 = alle)
        uint8_t data[2] = { (uint8_t)cmd, m_nodeId };
        return m_driver->sendFrame(0x000, data, 2);
    }

    void setState(NmtState newState) {
        m_state = newState;
        if (m_stateCallback) {
            m_stateCallback(newState);
        }
    }
};

#endif // NMT_MANAGER_H
