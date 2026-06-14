/**
 * @file dunker_drive.h
 * @brief Geraeteabstraktion fuer Dunkermotoren BG-Antriebe (CANopen / CiA 402)
 * @date 2026
 *
 * Milestone 4 (v1): DS402-Basis
 *   - Statusword-Monitoring (0x6041) per zyklischem SDO-Read
 *   - Controlword-Kommandos (0x6040): Fault-Reset / Shutdown / Switch-On / Enable
 *   - CiA-402-Statusmaschine dekodiert (State-Name + Fault/Warning-Flags)
 *
 * NODE-UNABHAENGIG: Node-ID kommt aus dem Konstruktor / rebind(), kein Hardcoding.
 * Non-blocking: alle SDO-Zugriffe laufen ueber den internen async SdoClient.
 *
 * SPEICHERN ALS: src/devices/dunker_drive.h
 */

#pragma once
#ifndef DUNKER_DRIVE_H
#define DUNKER_DRIVE_H

#include <Arduino.h>
#include <stdint.h>
#include "../canopen/canopen_driver.h"
#include "../canopen/sdo_client.h"

// ============================================================================
// CiA 402 Objektverzeichnis (Teilmenge fuer v1)
// ============================================================================
namespace DS402_OBJ {
    constexpr uint16_t CONTROLWORD = 0x6040; // UNSIGNED16 (write)
    constexpr uint16_t STATUSWORD  = 0x6041; // UNSIGNED16 (read)
    constexpr uint16_t ERROR_CODE  = 0x603F; // UNSIGNED16 (read, optional)
}

// CiA 402 Controlword-Kommandos (Bitmuster fuer die Statusmaschine)
namespace DS402_CMD {
    constexpr uint16_t SHUTDOWN         = 0x0006; // -> Ready to switch on
    constexpr uint16_t SWITCH_ON        = 0x0007; // -> Switched on
    constexpr uint16_t ENABLE_OPERATION = 0x000F; // -> Operation enabled
    constexpr uint16_t DISABLE_VOLTAGE  = 0x0000;
    constexpr uint16_t QUICK_STOP       = 0x0002;
    constexpr uint16_t FAULT_RESET      = 0x0080; // steigende Flanke Bit 7
}

enum class DS402State : uint8_t {
    Unknown = 0,
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
};

struct DunkerData {
    bool       connected        = false;
    bool       statuswordValid  = false;
    uint16_t   statusword       = 0;
    DS402State state            = DS402State::Unknown;
    bool       fault            = false; // Statusword Bit 3
    bool       warning          = false; // Statusword Bit 7
    bool       operationEnabled = false;
    uint16_t   lastControlword  = 0;
    uint32_t   lastRxMs         = 0;
    uint32_t   updateCount      = 0;
};

// ============================================================================
// DunkerDrive
// ============================================================================
class DunkerDrive {
public:
    DunkerDrive(CanopenDriver* can, uint8_t nodeId)
        : m_can(can)
        , m_sdo(can, nodeId)
        , m_nodeId(nodeId)
    {
        m_sdo.setTimeout(400);
        m_hb = 0x700 + nodeId;
    }

    uint8_t           nodeId()  const { return m_nodeId; }
    const DunkerData& getData() const { return m_data; }

    // Auf eine andere Node-ID umbinden (Objekt wird wiederverwendet -> kein Leak)
    void rebind(uint8_t nodeId) {
        m_nodeId = nodeId;
        m_sdo = SdoClient(m_can, nodeId);
        m_sdo.setTimeout(400);
        m_hb = 0x700 + nodeId;
        m_cwPending = false;
        m_nextStatusMs = 0;
        m_data = DunkerData{};
    }

    // -----------------------------------------------------------------------
    // Kommandos (non-blocking; Single-Slot, letztes Kommando gewinnt bei busy)
    // -----------------------------------------------------------------------
    void cmdFaultReset()      { queueControlword(DS402_CMD::FAULT_RESET); }
    void cmdShutdown()        { queueControlword(DS402_CMD::SHUTDOWN); }
    void cmdSwitchOn()        { queueControlword(DS402_CMD::SWITCH_ON); }
    void cmdEnableOperation() { queueControlword(DS402_CMD::ENABLE_OPERATION); }
    void cmdDisableVoltage()  { queueControlword(DS402_CMD::DISABLE_VOLTAGE); }

    // -----------------------------------------------------------------------
    // processFrame() - aus dem CAN RX-Callback aufrufen
    // -----------------------------------------------------------------------
    void processFrame(uint32_t cobId, const uint8_t* data, uint8_t len) {
        if (!data) return;
        m_sdo.processFrame(cobId, data, len);

        // Heartbeat haelt die Verbindung "alive"
        if (cobId == m_hb) {
            m_data.lastRxMs  = millis();
            m_data.connected = true;
        }
    }

    // -----------------------------------------------------------------------
    // update() - regelmaessig aus loop() aufrufen
    // -----------------------------------------------------------------------
    void update() {
        m_sdo.update();
        const uint32_t now = millis();

        if (m_data.lastRxMs != 0 && (now - m_data.lastRxMs) > 2000) {
            m_data.connected = false;
        }

        if (m_sdo.isBusy()) return;

        // 1) Ausstehendes Controlword hat Vorrang
        if (m_cwPending) {
            m_cwPending = false;
            const uint16_t cw = m_cwValue;
            Serial.printf("[DUNKER] Node %u: write 6040h = 0x%04X\n", (unsigned)m_nodeId, cw);
            m_sdo.writeAsync(DS402_OBJ::CONTROLWORD, 0x00, cw, 2,
                [this, cw](SdoResult r, uint32_t){
                    if (r == SDO_OK) m_data.lastControlword = cw;
                    else Serial.printf("[DUNKER] 6040h write failed (%d)\n", (int)r);
                    // Statusword direkt nach dem Kommando frisch lesen
                    m_nextStatusMs = millis();
                });
            return;
        }

        // 2) Zyklischer Statusword-Poll (0x6041)
        if ((int32_t)(now - m_nextStatusMs) >= 0) {
            m_nextStatusMs = now + m_statusPeriodMs;
            m_sdo.readAsync(DS402_OBJ::STATUSWORD, 0x00, [this](SdoResult r, uint32_t v){
                if (r == SDO_OK) {
                    m_data.statusword      = (uint16_t)(v & 0xFFFF);
                    m_data.statuswordValid = true;
                    m_data.lastRxMs        = millis();
                    m_data.connected       = true;
                    m_data.updateCount++;
                    decodeStatusword();
                }
            });
            return;
        }
    }

    // -----------------------------------------------------------------------
    // Helfer
    // -----------------------------------------------------------------------
    static const char* stateName(DS402State s) {
        switch (s) {
            case DS402State::NotReadyToSwitchOn:  return "Not ready";
            case DS402State::SwitchOnDisabled:    return "Switch-on disabled";
            case DS402State::ReadyToSwitchOn:     return "Ready to switch on";
            case DS402State::SwitchedOn:          return "Switched on";
            case DS402State::OperationEnabled:    return "Operation enabled";
            case DS402State::QuickStopActive:     return "Quick stop active";
            case DS402State::FaultReactionActive: return "Fault reaction";
            case DS402State::Fault:               return "FAULT";
            default:                              return "---";
        }
    }

private:
    void queueControlword(uint16_t v) { m_cwValue = v; m_cwPending = true; }

    void decodeStatusword() {
        const uint16_t sw = m_data.statusword;
        m_data.fault   = (sw & (1u << 3)) != 0;
        m_data.warning = (sw & (1u << 7)) != 0;

        // CiA 402 Statusmaschine (Maskierung der relevanten Bits)
        DS402State st = DS402State::Unknown;
        if      ((sw & 0x4F) == 0x00) st = DS402State::NotReadyToSwitchOn;
        else if ((sw & 0x4F) == 0x40) st = DS402State::SwitchOnDisabled;
        else if ((sw & 0x6F) == 0x21) st = DS402State::ReadyToSwitchOn;
        else if ((sw & 0x6F) == 0x23) st = DS402State::SwitchedOn;
        else if ((sw & 0x6F) == 0x27) st = DS402State::OperationEnabled;
        else if ((sw & 0x6F) == 0x07) st = DS402State::QuickStopActive;
        else if ((sw & 0x4F) == 0x0F) st = DS402State::FaultReactionActive;
        else if ((sw & 0x4F) == 0x08) st = DS402State::Fault;

        m_data.state            = st;
        m_data.operationEnabled = (st == DS402State::OperationEnabled);
    }

    CanopenDriver* m_can;
    SdoClient      m_sdo;
    uint8_t        m_nodeId;
    uint32_t       m_hb = 0;

    bool     m_cwPending = false;
    uint16_t m_cwValue   = 0;

    uint32_t m_nextStatusMs   = 0;
    uint32_t m_statusPeriodMs = 250;

    DunkerData m_data;
};

#endif // DUNKER_DRIVE_H
