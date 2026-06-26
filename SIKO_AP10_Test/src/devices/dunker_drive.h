/**
 * @file dunker_drive.h
 * @brief Geraeteabstraktion fuer Dunkermotoren BG-Antriebe (CANopen / CiA 402)
 * @date 2026
 *
 * Milestone 4 (v1): DS402-Basis
 *   - Statusword-Monitoring (0x6041), Controlword-Kommandos (0x6040)
 *   - CiA-402-Statusmaschine dekodiert
 *
 * Milestone 5: Bewegung (Standard-CiA-402-Objekte)
 *   - Mode of operation (0x6060/0x6061): PP / PV / Homing
 *   - Profile Position: Zielposition (0x607A), Profilgeschwindigkeit (0x6081),
 *     New-Setpoint ueber Controlword Bit4
 *   - Jog ueber Profile Velocity (0x60FF), Halt (Controlword Bit8)
 *   - Istwerte: Position (0x6064), Geschwindigkeit (0x606C)
 *
 * Milestone 6: I/O + Bremse
 *   - Digitale Eingaenge (0x60FD) lesen   -> STANDARD CiA 402
 *   - Digitale Ausgaenge (0x60FE:01) schreiben -> STANDARD CiA 402
 *   - Bremse: KEIN universelles Standardobjekt. Index wird via configureBrake()
 *     gesetzt (aus der EDS). Ohne Konfiguration: no-op + Warnung (kein Raten).
 *
 * NODE-UNABHAENGIG (Konstruktor / rebind()). Non-blocking (interner async SDO).
 * SPEICHERN ALS: src/devices/dunker_drive.h
 */

#pragma once
#ifndef DUNKER_DRIVE_H
#define DUNKER_DRIVE_H

#include <Arduino.h>
#include <stdint.h>
#include "../canopen/canopen_driver.h"
#include "../canopen/sdo_client.h"

static constexpr uint32_t DUNKER_SDO_TIMEOUT_MS = 400;
static constexpr uint32_t DUNKER_IMA_SDO_TIMEOUT_MS = 3000;

// ============================================================================
// CiA 402 Objektverzeichnis (Teilmenge)
// ============================================================================
namespace DS402_OBJ {
    constexpr uint16_t CONTROLWORD     = 0x6040; // U16  (write)
    constexpr uint16_t STATUSWORD      = 0x6041; // U16  (read)
    constexpr uint16_t MODE_SET        = 0x6060; // I8   (write)
    constexpr uint16_t MODE_DISPLAY    = 0x6061; // I8   (read)
    constexpr uint16_t POSITION_ACTUAL = 0x6064; // I32  (read)
    constexpr uint16_t VELOCITY_ACTUAL = 0x606C; // I32  (read)
    constexpr uint16_t TARGET_POSITION = 0x607A; // I32  (write)
    constexpr uint16_t PROFILE_VELOCITY= 0x6081; // U32  (write)
    constexpr uint16_t TARGET_VELOCITY = 0x60FF; // I32  (write, PV mode)
    constexpr uint16_t DIGITAL_INPUTS  = 0x60FD; // U32  (read)   STANDARD
    constexpr uint16_t DIGITAL_OUTPUTS = 0x60FE; // U32  (write @ sub 1) STANDARD
}

// CiA 402 Modes of operation
namespace DS402_MODE {
    constexpr int8_t PROFILE_POSITION = 1;
    constexpr int8_t VELOCITY         = 2;
    constexpr int8_t PROFILE_VELOCITY = 3;
    constexpr int8_t HOMING           = 6;
}

// Controlword-Bitmuster
namespace DS402_CMD {
    constexpr uint16_t SHUTDOWN         = 0x0006; // -> Ready to switch on
    constexpr uint16_t SWITCH_ON        = 0x0007; // -> Switched on
    constexpr uint16_t ENABLE_OPERATION = 0x000F; // -> Operation enabled
    constexpr uint16_t DISABLE_VOLTAGE  = 0x0000;
    constexpr uint16_t QUICK_STOP       = 0x0002;
    constexpr uint16_t FAULT_RESET      = 0x0080; // steigende Flanke Bit 7
    constexpr uint16_t NEW_SETPOINT     = 0x001F; // 0x000F | Bit4 (new setpoint)
    constexpr uint16_t NEW_SETPOINT_IMM = 0x003F; // + Bit5 (change immediately)
    constexpr uint16_t HALT             = 0x010F; // 0x000F | Bit8 (halt)
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

enum class ImaConfigState : uint8_t {
    Idle = 0,
    Running,
    Verifying,
    Done,
    Error,
    ParamResetting,
    ParamResetDone,
};

struct ImaVerifyResult {
    bool txPdo1_obj1 = false;
    bool txPdo1_obj2 = false;
    bool txPdo2_obj1 = false;
    bool txPdo2_obj2 = false;
    bool txPdo3_obj1 = false;
    bool txPdo3_obj2 = false;
    bool txPdo4_obj1 = false;
    bool txPdo4_obj2 = false;
    bool rxPdo1_obj1 = false;
    bool rxPdo1_obj2 = false;
    bool rxPdo2_obj1 = false;
    bool rxPdo2_obj2 = false;

    bool allOk() const {
        return txPdo1_obj1 && txPdo1_obj2 &&
               txPdo2_obj1 && txPdo2_obj2 &&
               txPdo3_obj1 && txPdo3_obj2 &&
               txPdo4_obj1 && txPdo4_obj2 &&
               rxPdo1_obj1 && rxPdo1_obj2 &&
               rxPdo2_obj1 && rxPdo2_obj2;
    }
};

struct DunkerData {
    bool       connected        = false;
    // DS402 state machine
    bool       statuswordValid  = false;
    uint16_t   statusword       = 0;
    DS402State state            = DS402State::Unknown;
    bool       fault            = false;
    bool       warning          = false;
    bool       operationEnabled = false;
    uint16_t   lastControlword  = 0;
    // Motion (M5)
    bool       modeValid        = false;
    int8_t     mode             = 0;     // 0x6061
    bool       posValid         = false;
    int32_t    positionActual   = 0;     // 0x6064
    bool       velValid         = false;
    int32_t    velocityActual   = 0;     // 0x606C
    int32_t    targetPosition   = 0;     // last commanded 0x607A
    uint32_t   profileVelocity  = 0;     // last commanded 0x6081
    // I/O (M6)
    bool       diValid          = false;
    uint32_t   digitalInputs    = 0;     // 0x60FD
    uint32_t   digitalOutputs   = 0;     // last written 0x60FE:01
    bool       brakeConfigured  = false;
    // IMA Config (M7)
    ImaConfigState imaState     = ImaConfigState::Idle;
    ImaVerifyResult imaVerify;
    char       imaStatusMsg[80] = "---";
    // stats
    uint32_t   lastRxMs         = 0;
    uint32_t   updateCount      = 0;
};

// ============================================================================
// DunkerDrive
// ============================================================================
class DunkerDrive {
public:
    DunkerDrive(CanopenDriver* can, uint8_t nodeId)
        : m_can(can), m_sdo(can, nodeId), m_nodeId(nodeId)
    {
        m_sdo.setTimeout(DUNKER_SDO_TIMEOUT_MS);
        m_hb = 0x700 + nodeId;
    }

    uint8_t           nodeId()  const { return m_nodeId; }
    const DunkerData& getData() const { return m_data; }

    void rebind(uint8_t nodeId) {
        m_nodeId = nodeId;
        m_sdo = SdoClient(m_can, nodeId);
        m_sdo.setTimeout(DUNKER_SDO_TIMEOUT_MS);
        m_hb = 0x700 + nodeId;
        m_qHead = m_qTail = 0;
        m_pollIdx = 0;
        m_nextPollMs = 0;
        m_data = DunkerData{};
        m_data.brakeConfigured = (m_brakeIndex != 0);
        resetImaRuntime();
    }

    void deactivate() {
        m_sdo.cancel();
        m_qHead = m_qTail = 0;
        m_pollIdx = 0;
        m_nextPollMs = 0;
        m_data.connected = false;
        resetImaRuntime();
        m_data.imaState = ImaConfigState::Idle;
        snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg), "---");
        m_sdo.setTimeout(DUNKER_SDO_TIMEOUT_MS);
    }

    // -----------------------------------------------------------------------
    // M4: DS402 state-machine commands (Controlword)
    // -----------------------------------------------------------------------
    void cmdFaultReset()      { writeCw(DS402_CMD::FAULT_RESET); }
    void cmdShutdown()        { writeCw(DS402_CMD::SHUTDOWN); }
    void cmdSwitchOn()        { writeCw(DS402_CMD::SWITCH_ON); }
    void cmdEnableOperation() { writeCw(DS402_CMD::ENABLE_OPERATION); }
    void cmdDisableVoltage()  { writeCw(DS402_CMD::DISABLE_VOLTAGE); }
    void cmdHalt()            { writeCw(DS402_CMD::HALT); }

    // -----------------------------------------------------------------------
    // M5: motion
    // -----------------------------------------------------------------------
    void setMode(int8_t mode) {
        Serial.printf("[DUNKER] Node %u: set mode 6060h = %d\n", (unsigned)m_nodeId, (int)mode);
        qPush(DS402_OBJ::MODE_SET, 0x00, (uint32_t)(uint8_t)mode, 1);
    }

    // Profile-Position move: optional velocity, then target, then new-setpoint edge.
    void gotoPosition(int32_t target, uint32_t velocity) {
        m_data.targetPosition  = target;
        m_data.profileVelocity = velocity;
        Serial.printf("[DUNKER] Node %u: PP goto pos=%ld vel=%lu\n",
                      (unsigned)m_nodeId, (long)target, (unsigned long)velocity);
        qPush(DS402_OBJ::MODE_SET,         0x00, (uint32_t)(uint8_t)DS402_MODE::PROFILE_POSITION, 1);
        if (velocity > 0) qPush(DS402_OBJ::PROFILE_VELOCITY, 0x00, velocity, 4);
        qPush(DS402_OBJ::TARGET_POSITION,  0x00, (uint32_t)target, 4);
        // New-setpoint rising edge on controlword bit4 (change immediately)
        qPush(DS402_OBJ::CONTROLWORD, 0x00, DS402_CMD::ENABLE_OPERATION, 2);
        qPush(DS402_OBJ::CONTROLWORD, 0x00, DS402_CMD::NEW_SETPOINT_IMM, 2);
        qPush(DS402_OBJ::CONTROLWORD, 0x00, DS402_CMD::ENABLE_OPERATION, 2); // clear bit4
    }

    // Jog via Profile Velocity. dir: -1/+1 ; speed in device units. dir 0 = stop.
    void jog(int dir, uint32_t speed) {
        if (dir == 0) {
            Serial.printf("[DUNKER] Node %u: jog stop\n", (unsigned)m_nodeId);
            qPush(DS402_OBJ::TARGET_VELOCITY, 0x00, 0, 4);
            return;
        }
        const int32_t v = (dir < 0) ? -(int32_t)speed : (int32_t)speed;
        Serial.printf("[DUNKER] Node %u: jog vel=%ld\n", (unsigned)m_nodeId, (long)v);
        qPush(DS402_OBJ::MODE_SET,        0x00, (uint32_t)(uint8_t)DS402_MODE::PROFILE_VELOCITY, 1);
        qPush(DS402_OBJ::TARGET_VELOCITY, 0x00, (uint32_t)v, 4);
        qPush(DS402_OBJ::CONTROLWORD,     0x00, DS402_CMD::ENABLE_OPERATION, 2);
    }

    void halt() { cmdHalt(); }

    // -----------------------------------------------------------------------
    // Dunker manufacturer config via SDO 0x2000. Subindices verified on real
    // hardware (the BG manual excerpt had :02/:03 the other way round):
    //   0x2000:01 = 0x6E657277  (Freischaltung / write enable)
    //   0x2000:02 = node-id
    //   0x2000:03 = baudrate index (0=1M..8=10k)
    // -----------------------------------------------------------------------
    static constexpr uint32_t DUNKER_WREN = 0x6E657277UL;

    // :02/:03 are UNSIGNED32 (1 byte was rejected with abort 0x06070010).
    static constexpr uint8_t DUNKER_CFG_SIZE = 4;

    void cfgNodeIdSdo(uint8_t newNodeId) {
        Serial.printf("[DUNKER] Node %u: set node-id via 0x2000:02 -> %u\n",
                      (unsigned)m_nodeId, (unsigned)newNodeId);
        qPush(0x2000, 0x01, DUNKER_WREN, 4);                 // unlock (U32)
        qPush(0x2000, 0x02, newNodeId, DUNKER_CFG_SIZE);     // node-id (subindex 2)
    }

    // baudIndex: 0=1M, 1=800k, 2=500k, 3=250k, 4=125k, 5=100k, 6=50k, 7=20k, 8=10k
    void cfgBaudSdo(uint8_t baudIndex) {
        Serial.printf("[DUNKER] Node %u: set baud index %u via 0x2000:03\n",
                      (unsigned)m_nodeId, (unsigned)baudIndex);
        qPush(0x2000, 0x01, DUNKER_WREN, 4);                 // unlock (U32)
        qPush(0x2000, 0x03, baudIndex, DUNKER_CFG_SIZE);     // baudrate (subindex 3)
    }

    // -----------------------------------------------------------------------
    // M7: IMA parameter configuration (PDO mapping + EEPROM)
    // -----------------------------------------------------------------------
    void imaConfigApply() {
        if (imaBusy() || !qEmpty() || m_sdo.isBusy()) {
            Serial.println("[DUNKER] IMA apply ignored: drive SDO busy");
            return;
        }

        Serial.printf("[DUNKER] Node %u: IMA configuration apply\n", (unsigned)m_nodeId);
        resetImaRuntime();
        m_sdo.setTimeout(DUNKER_IMA_SDO_TIMEOUT_MS);
        m_data.imaState = ImaConfigState::Running;
        snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg), "IMA Uebertragung laeuft...");

        qPushIma(0x1011, 0x01, 0x64616F6CUL, 4); // "load"

        // Disable PDOs via communication parameters (COB-ID bit31), then clear
        // mapping tables. Standard CANopen COB-IDs are used when re-enabling.
        qPushIma(0x1800, 0x01, 0x80000180UL + m_nodeId, 4);
        qPushIma(0x1801, 0x01, 0x80000280UL + m_nodeId, 4);
        qPushIma(0x1802, 0x01, 0x80000380UL + m_nodeId, 4);
        qPushIma(0x1803, 0x01, 0x80000480UL + m_nodeId, 4);
        qPushIma(0x1400, 0x01, 0x80000200UL + m_nodeId, 4);
        qPushIma(0x1401, 0x01, 0x80000300UL + m_nodeId, 4);

        for (uint16_t idx = 0x1A00; idx <= 0x1A06; idx++) qPushIma(idx, 0x00, 0, 4);
        for (uint16_t idx = 0x1600; idx <= 0x1606; idx++) qPushIma(idx, 0x00, 0, 4);

        qPushIma(0x1A00, 0x01, 0x60410010UL, 4);
        qPushIma(0x1A00, 0x02, 0x60640020UL, 4);
        qPushIma(0x1A00, 0x00, 2, 4);

        qPushIma(0x1A01, 0x01, 0x30020020UL, 4);
        qPushIma(0x1A01, 0x02, 0x31200010UL, 4);
        qPushIma(0x1A01, 0x00, 2, 4);

        qPushIma(0x1A02, 0x01, 0x3A040020UL, 4);
        qPushIma(0x1A02, 0x02, 0x37730020UL, 4);
        qPushIma(0x1A02, 0x00, 2, 4);

        qPushIma(0x1A03, 0x01, 0x31130020UL, 4);
        qPushIma(0x1A03, 0x02, 0x39700108UL, 4);
        qPushIma(0x1A03, 0x00, 2, 4);

        qPushIma(0x1600, 0x01, 0x60400010UL, 4);
        qPushIma(0x1600, 0x02, 0x607A0020UL, 4);
        qPushIma(0x1600, 0x00, 2, 4);

        qPushIma(0x1601, 0x01, 0x31500008UL, 4);
        qPushIma(0x1601, 0x02, 0x60420010UL, 4);
        qPushIma(0x1601, 0x00, 2, 4);

        qPushIma(0x1800, 0x01, 0x00000180UL + m_nodeId, 4);
        qPushIma(0x1801, 0x01, 0x00000280UL + m_nodeId, 4);
        qPushIma(0x1802, 0x01, 0x00000380UL + m_nodeId, 4);
        qPushIma(0x1803, 0x01, 0x00000480UL + m_nodeId, 4);
        qPushIma(0x1400, 0x01, 0x00000200UL + m_nodeId, 4);
        qPushIma(0x1401, 0x01, 0x00000300UL + m_nodeId, 4);

        qPushIma(0x2000, 0x01, DUNKER_WREN, 4);
        qPushIma(0x2000, 0x04, 0, 4);
        qPushIma(0x1010, 0x01, 0x65766173UL, 4); // "save"

        m_imaWriteCount = m_imaQueuedCount;
        if (m_imaQueueOverflow || m_imaWriteCount == 0) {
            m_qHead = m_qTail = 0;
            m_sdo.setTimeout(DUNKER_SDO_TIMEOUT_MS);
            m_data.imaState = ImaConfigState::Error;
            snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg), "FEHLER: IMA Queue voll");
        }
    }

    void imaVerify() {
        if (imaBusy() || !qEmpty() || m_sdo.isBusy()) {
            Serial.println("[DUNKER] IMA verify ignored: drive SDO busy");
            return;
        }
        m_data.imaState = ImaConfigState::Verifying;
        m_data.imaVerify = ImaVerifyResult{};
        m_sdo.setTimeout(DUNKER_IMA_SDO_TIMEOUT_MS);
        m_imaVerifyIdx = 0;
        m_imaVerifyActive = true;
        snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg), "Verifikation laeuft...");
    }

    void imaParamReset() {
        if (imaBusy() || !qEmpty() || m_sdo.isBusy()) {
            Serial.println("[DUNKER] IMA param reset ignored: drive SDO busy");
            return;
        }

        Serial.printf("[DUNKER] Node %u: parameter reset only (0x1011:01)\n", (unsigned)m_nodeId);
        resetImaRuntime();
        m_sdo.setTimeout(DUNKER_IMA_SDO_TIMEOUT_MS);
        m_data.imaState = ImaConfigState::ParamResetting;
        m_imaParamResetOnly = true;
        snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg), "Parameter-Reset laeuft...");
        qPushIma(0x1011, 0x01, 0x64616F6CUL, 4); // "load"
        m_imaWriteCount = m_imaQueuedCount;
    }

    // -----------------------------------------------------------------------
    // M6: digital I/O + brake
    // -----------------------------------------------------------------------
    // Set/clear one physical output bit (0x60FE:01). Read-modify-write on the
    // locally cached mask (the drive's bitmask sub-index 0x60FE:02 from the EDS
    // governs which bits are actually user-controllable).
    void setOutputBit(uint8_t bit, bool on) {
        if (bit > 31) return;
        if (on) m_data.digitalOutputs |=  (1UL << bit);
        else    m_data.digitalOutputs &= ~(1UL << bit);
        Serial.printf("[DUNKER] Node %u: 60FE:01 = 0x%08lX\n",
                      (unsigned)m_nodeId, (unsigned long)m_data.digitalOutputs);
        qPush(DS402_OBJ::DIGITAL_OUTPUTS, 0x01, m_data.digitalOutputs, 4);
    }

    // Configure the manufacturer-specific brake object from the EDS.
    // Until configured, setBrake() is a no-op (we do NOT guess the object).
    void configureBrake(uint16_t index, uint8_t sub, uint32_t releaseValue,
                        uint32_t engageValue, uint8_t size) {
        m_brakeIndex   = index;
        m_brakeSub     = sub;
        m_brakeRelease = releaseValue;
        m_brakeEngage  = engageValue;
        m_brakeSize    = size;
        m_data.brakeConfigured = (index != 0);
        Serial.printf("[DUNKER] Brake object configured: 0x%04X:%u\n", index, (unsigned)sub);
    }

    void setBrake(bool release) {
        if (m_brakeIndex == 0) {
            Serial.println("[DUNKER] setBrake ignored: brake object not configured (needs EDS)");
            return;
        }
        const uint32_t v = release ? m_brakeRelease : m_brakeEngage;
        Serial.printf("[DUNKER] Node %u: brake %s -> 0x%04X:%u = 0x%lX\n",
                      (unsigned)m_nodeId, release ? "RELEASE" : "ENGAGE",
                      m_brakeIndex, (unsigned)m_brakeSub, (unsigned long)v);
        qPush(m_brakeIndex, m_brakeSub, v, m_brakeSize);
    }

    // -----------------------------------------------------------------------
    // processFrame() - aus dem CAN RX-Callback
    // -----------------------------------------------------------------------
    void processFrame(uint32_t cobId, const uint8_t* data, uint8_t len) {
        if (!data) return;
        m_sdo.processFrame(cobId, data, len);
        if (cobId == m_hb) {
            m_data.lastRxMs  = millis();
            m_data.connected = true;
        }
    }

    // -----------------------------------------------------------------------
    // update() - regelmaessig aus loop()
    // -----------------------------------------------------------------------
    void update() {
        m_sdo.update();
        const uint32_t now = millis();

        if (m_data.lastRxMs != 0 && (now - m_data.lastRxMs) > 2000) {
            m_data.connected = false;
        }

        if (m_sdo.isBusy()) return;

        if (m_imaVerifyActive) {
            runImaVerify();
            return;
        }

        // Pending writes have priority (controlword, motion, I/O, IMA)
        if (!qEmpty()) {
            PendingWrite w = qPop();
            m_sdo.writeAsync(w.index, w.sub, w.value, w.size,
                [this, w](SdoResult r, uint32_t){
                    if (r == SDO_OK) {
                        if (w.index == DS402_OBJ::CONTROLWORD) m_data.lastControlword = (uint16_t)w.value;
                        if (w.isIma) {
                            m_imaWriteDoneCount++;
                            if (m_imaWriteDoneCount >= m_imaWriteCount) {
                                if (m_imaParamResetOnly) {
                                    m_imaParamResetOnly = false;
                                    m_sdo.setTimeout(DUNKER_SDO_TIMEOUT_MS);
                                    m_data.imaState = ImaConfigState::ParamResetDone;
                                    snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg),
                                             "Parameter-Reset OK. Power-Cycle erforderlich.");
                                    Serial.println("[DUNKER] IMA parameter reset done; power-cycle required");
                                } else {
                                    m_data.imaState = ImaConfigState::Verifying;
                                    m_data.imaVerify = ImaVerifyResult{};
                                    m_imaVerifyIdx = 0;
                                    m_imaVerifyActive = true;
                                    snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg),
                                             "IMA geschrieben, Verifikation laeuft...");
                                    Serial.println("[DUNKER] IMA writes done, starting verify");
                                }
                            }
                        }
                    } else if (w.index == 0x2000 && w.sub != 0x01 && r == SDO_TIMEOUT) {
                        // The drive applies node-id/baud and re-initialises WITHOUT
                        // sending an SDO confirmation -> a timeout here is expected.
                        // The value still takes effect after a power-cycle.
                        Serial.printf("[DUNKER] 0x2000:%u no SDO confirm (expected; value applies after power-cycle)\n",
                                      (unsigned)w.sub);
                        if (w.isIma) {
                            m_imaWriteDoneCount++;
                        }
                    } else if (w.isIma) {
                        m_qHead = m_qTail = 0;
                        m_imaVerifyActive = false;
                        m_imaParamResetOnly = false;
                        m_sdo.setTimeout(DUNKER_SDO_TIMEOUT_MS);
                        m_data.imaState = ImaConfigState::Error;
                        snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg),
                                 "FEHLER: SDO 0x%04X:%u (%d)", w.index, (unsigned)w.sub, (int)r);
                        Serial.printf("[DUNKER] IMA write 0x%04X:%u failed (%d)\n",
                                      w.index, (unsigned)w.sub, (int)r);
                    } else {
                        Serial.printf("[DUNKER] write 0x%04X:%u failed (%d)\n", w.index, (unsigned)w.sub, (int)r);
                    }
                });
            // After a command, refresh statusword promptly
            m_nextPollMs = now;
            return;
        }

        if (imaBusy()) return;

        // Round-robin read poll
        if ((int32_t)(now - m_nextPollMs) >= 0) {
            m_nextPollMs = now + m_pollPeriodMs;
            pollNext();
            return;
        }
    }

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

    static const char* modeName(int8_t m) {
        switch (m) {
            case DS402_MODE::PROFILE_POSITION: return "Profile Position";
            case DS402_MODE::VELOCITY:         return "Velocity";
            case DS402_MODE::PROFILE_VELOCITY: return "Profile Velocity";
            case DS402_MODE::HOMING:           return "Homing";
            case 0:                            return "---";
            default:                           return "?";
        }
    }

    static const char* imaStateName(ImaConfigState s) {
        switch (s) {
            case ImaConfigState::Idle:           return "Bereit";
            case ImaConfigState::Running:        return "IMA Uebertragung...";
            case ImaConfigState::Verifying:      return "Verifikation...";
            case ImaConfigState::Done:           return "Abgeschlossen OK";
            case ImaConfigState::Error:          return "FEHLER";
            case ImaConfigState::ParamResetting: return "Parameter-Reset laeuft...";
            case ImaConfigState::ParamResetDone: return "Parameter-Reset OK (Power-Cycle!)";
            default:                             return "---";
        }
    }

private:
    struct ImaVerifyStep {
        uint16_t index;
        uint8_t sub;
        uint32_t expected;
        bool ImaVerifyResult::* resultField;
        const char* label;
    };
    static const ImaVerifyStep s_imaVerifySteps[12];

    struct PendingWrite { uint16_t index; uint8_t sub; uint32_t value; uint8_t size; bool isIma = false; };
    static constexpr uint8_t QN = 64;

    void writeCw(uint16_t cw) { qPush(DS402_OBJ::CONTROLWORD, 0x00, cw, 2); }

    bool qEmpty() const { return m_qHead == m_qTail; }

    bool qPush(uint16_t index, uint8_t sub, uint32_t value, uint8_t size, bool isIma = false) {
        const uint8_t next = (uint8_t)((m_qTail + 1) % QN);
        if (next == m_qHead) {
            Serial.println("[DUNKER] write queue full - command dropped");
            if (isIma) m_imaQueueOverflow = true;
            return false;
        }
        m_q[m_qTail] = PendingWrite{ index, sub, value, size, isIma };
        m_qTail = next;
        return true;
    }

    void qPushIma(uint16_t index, uint8_t sub, uint32_t value, uint8_t size) {
        if (qPush(index, sub, value, size, true)) m_imaQueuedCount++;
    }

    PendingWrite qPop() {
        PendingWrite w = m_q[m_qHead];
        m_qHead = (uint8_t)((m_qHead + 1) % QN);
        return w;
    }

    bool imaBusy() const {
        return m_data.imaState == ImaConfigState::Running ||
               m_data.imaState == ImaConfigState::Verifying ||
               m_data.imaState == ImaConfigState::ParamResetting ||
               m_imaVerifyActive;
    }

    void resetImaRuntime() {
        m_imaVerifyActive = false;
        m_imaVerifyIdx = 0;
        m_imaQueuedCount = 0;
        m_imaWriteCount = 0;
        m_imaWriteDoneCount = 0;
        m_imaParamResetOnly = false;
        m_imaQueueOverflow = false;
    }

    void runImaVerify() {
        if (m_imaVerifyIdx >= 12) {
            m_imaVerifyActive = false;
            m_sdo.setTimeout(DUNKER_SDO_TIMEOUT_MS);
            if (m_data.imaVerify.allOk()) {
                m_data.imaState = ImaConfigState::Done;
                snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg),
                         "OK - alle 12 Mapping-Objekte korrekt.");
                Serial.println("[DUNKER] IMA verify: all OK");
            } else {
                uint8_t ok = 0;
                const ImaVerifyResult& v = m_data.imaVerify;
                auto countOk = [&](bool b){ if (b) ok++; };
                countOk(v.txPdo1_obj1); countOk(v.txPdo1_obj2);
                countOk(v.txPdo2_obj1); countOk(v.txPdo2_obj2);
                countOk(v.txPdo3_obj1); countOk(v.txPdo3_obj2);
                countOk(v.txPdo4_obj1); countOk(v.txPdo4_obj2);
                countOk(v.rxPdo1_obj1); countOk(v.rxPdo1_obj2);
                countOk(v.rxPdo2_obj1); countOk(v.rxPdo2_obj2);
                m_data.imaState = ImaConfigState::Error;
                snprintf(m_data.imaStatusMsg, sizeof(m_data.imaStatusMsg),
                         "FEHLER: %u/12 Mapping-Checks OK", (unsigned)ok);
                Serial.printf("[DUNKER] IMA verify: %u/12 OK\n", (unsigned)ok);
            }
            return;
        }

        const ImaVerifyStep& step = s_imaVerifySteps[m_imaVerifyIdx];
        const uint8_t verifyIdx = m_imaVerifyIdx;
        m_sdo.readAsync(step.index, step.sub, [this, verifyIdx](SdoResult r, uint32_t v) {
            const ImaVerifyStep& st = s_imaVerifySteps[verifyIdx];
            const bool ok = (r == SDO_OK && v == st.expected);
            m_data.imaVerify.*(st.resultField) = ok;
            Serial.printf("[DUNKER] IMA verify %s: 0x%04X:%u = 0x%08lX expected 0x%08lX -> %s\n",
                          st.label, st.index, (unsigned)st.sub,
                          (unsigned long)v, (unsigned long)st.expected,
                          ok ? "OK" : "FAIL");
            m_imaVerifyIdx++;
        });
    }

    void pollNext() {
        switch (m_pollIdx) {
            case 0:
                m_sdo.readAsync(DS402_OBJ::STATUSWORD, 0x00, [this](SdoResult r, uint32_t v){
                    if (r == SDO_OK) {
                        m_data.statusword = (uint16_t)(v & 0xFFFF);
                        m_data.statuswordValid = true;
                        m_data.lastRxMs = millis();
                        m_data.connected = true;
                        m_data.updateCount++;
                        decodeStatusword();
                    }
                });
                break;
            case 1:
                m_sdo.readAsync(DS402_OBJ::POSITION_ACTUAL, 0x00, [this](SdoResult r, uint32_t v){
                    if (r == SDO_OK) { m_data.positionActual = (int32_t)v; m_data.posValid = true; }
                });
                break;
            case 2:
                m_sdo.readAsync(DS402_OBJ::MODE_DISPLAY, 0x00, [this](SdoResult r, uint32_t v){
                    if (r == SDO_OK) { m_data.mode = (int8_t)(v & 0xFF); m_data.modeValid = true; }
                });
                break;
            case 3:
                m_sdo.readAsync(DS402_OBJ::VELOCITY_ACTUAL, 0x00, [this](SdoResult r, uint32_t v){
                    if (r == SDO_OK) { m_data.velocityActual = (int32_t)v; m_data.velValid = true; }
                });
                break;
            default:
                m_sdo.readAsync(DS402_OBJ::DIGITAL_INPUTS, 0x00, [this](SdoResult r, uint32_t v){
                    if (r == SDO_OK) { m_data.digitalInputs = v; m_data.diValid = true; }
                });
                break;
        }
        m_pollIdx = (uint8_t)((m_pollIdx + 1) % 5);
    }

    void decodeStatusword() {
        const uint16_t sw = m_data.statusword;
        m_data.fault   = (sw & (1u << 3)) != 0;
        m_data.warning = (sw & (1u << 7)) != 0;

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

    PendingWrite m_q[QN];
    uint8_t      m_qHead = 0, m_qTail = 0;

    uint8_t  m_pollIdx     = 0;
    uint32_t m_nextPollMs  = 0;
    uint32_t m_pollPeriodMs = 120;

    // Manufacturer-specific brake object (from EDS); 0 = not configured
    uint16_t m_brakeIndex   = 0;
    uint8_t  m_brakeSub     = 0;
    uint32_t m_brakeRelease = 0;
    uint32_t m_brakeEngage  = 0;
    uint8_t  m_brakeSize    = 2;

    bool     m_imaVerifyActive   = false;
    uint8_t  m_imaVerifyIdx      = 0;
    uint8_t  m_imaQueuedCount    = 0;
    uint8_t  m_imaWriteCount     = 0;
    uint8_t  m_imaWriteDoneCount = 0;
    bool     m_imaParamResetOnly = false;
    bool     m_imaQueueOverflow  = false;

    DunkerData m_data;
};

inline const DunkerDrive::ImaVerifyStep DunkerDrive::s_imaVerifySteps[12] = {
    { 0x1A00, 0x01, 0x60410010UL, &ImaVerifyResult::txPdo1_obj1, "TxPDO1 Obj1 Statusword" },
    { 0x1A00, 0x02, 0x60640020UL, &ImaVerifyResult::txPdo1_obj2, "TxPDO1 Obj2 Actual Position" },
    { 0x1A01, 0x01, 0x30020020UL, &ImaVerifyResult::txPdo2_obj1, "TxPDO2 Obj1 Statusreg" },
    { 0x1A01, 0x02, 0x31200010UL, &ImaVerifyResult::txPdo2_obj2, "TxPDO2 Obj2 Inputs" },
    { 0x1A02, 0x01, 0x3A040020UL, &ImaVerifyResult::txPdo3_obj1, "TxPDO3 Obj1 Velocity" },
    { 0x1A02, 0x02, 0x37730020UL, &ImaVerifyResult::txPdo3_obj2, "TxPDO3 Obj2 Following Error" },
    { 0x1A03, 0x01, 0x31130020UL, &ImaVerifyResult::txPdo4_obj1, "TxPDO4 Obj1 Motor Current" },
    { 0x1A03, 0x02, 0x39700108UL, &ImaVerifyResult::txPdo4_obj2, "TxPDO4 Obj2 SSI Status" },
    { 0x1600, 0x01, 0x60400010UL, &ImaVerifyResult::rxPdo1_obj1, "RxPDO1 Obj1 Controlword" },
    { 0x1600, 0x02, 0x607A0020UL, &ImaVerifyResult::rxPdo1_obj2, "RxPDO1 Obj2 Target Position" },
    { 0x1601, 0x01, 0x31500008UL, &ImaVerifyResult::rxPdo2_obj1, "RxPDO2 Obj1 Outputs" },
    { 0x1601, 0x02, 0x60420010UL, &ImaVerifyResult::rxPdo2_obj2, "RxPDO2 Obj2 VL Velocity" },
};

#endif // DUNKER_DRIVE_H
