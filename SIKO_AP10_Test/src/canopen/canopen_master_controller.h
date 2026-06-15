#pragma once

/*
 * Universal CANopen Master Controller
 *
 * Core idea:
 * - IDLE: only CAN RX running (if desired), no device drivers, no auto recovery
 * - SCANNING: passive discovery + optional classification
 * - ONLINE: user-selected node; monitoring & (optional) device-specific logic enabled
 *
 * This is a header-only skeleton to integrate into the existing sketch.
 */

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

#include "src/canopen/canopen_driver.h"
#include "src/canopen/nmt_manager.h"
#include "src/canopen/sdo_client.h"

enum class MasterMode : uint8_t {
    Idle = 0,
    Scanning,
    OnlineStandard,
    OnlineKnown,
};

// Online init stages for known devices
enum class OnlineInitStage : uint8_t {
    None = 0,

    // Stage A: get the node into a clean comm state (for known devices)
    KickResetComm,

    // Stage B: start the node (NMT Start)
    KickStart,

    // Stage C: identify device via SDO (1018h etc.)
    Identify_1018_Entries,
    Identify_1018_Vendor,
    Identify_1018_Product,

    Identify_650A_Subs,
    Identify_650A_Offset,
    Identify_650A_Min,
    Identify_650A_Max,

    Done,
};


enum class KnownDeviceType : uint8_t {
    Unknown = 0,
    SIKO_AP04,
    SIKO_AP10,
    Dunker_75CI,
};

// Simple classifier placeholder.
// NOTE: We intentionally do NOT guess by Node-ID anymore.
// Device classification must be done via SDO identity (1018h) after explicit connect.
static inline KnownDeviceType guessKnownDevice(uint8_t nodeId) {
    (void)nodeId;
    return KnownDeviceType::Unknown;
}

static inline bool knownDeviceWantsAutoStart(KnownDeviceType t) {
    // AP04: we want to ensure it reaches OPERATIONAL after explicit connect.
    return t == KnownDeviceType::SIKO_AP04 || t == KnownDeviceType::SIKO_AP10;
}

// Map a CANopen identity (1018h vendor-id / product-code) to a known device type.
// Used by the passive "Identify All" feature (no connect required).
static inline KnownDeviceType classifyByIdentity(uint32_t vendorId, uint32_t productCode) {
    // SIKO vendor-id = 0x00000195
    if (vendorId == 0x00000195UL) {
        // AP04 reports product-code ASCII "CAN" = 0x004E4143
        if (productCode == 0x004E4143UL) return KnownDeviceType::SIKO_AP04;
        // Any other SIKO product -> treat as AP10 family (positioning display)
        return KnownDeviceType::SIKO_AP10;
    }
    // Dunkermotoren GmbH vendor-id = 0x00000257 (CiA CANopen vendor-id registry).
    // We only have one Dunker type in the enum so far; product-code could later
    // distinguish individual drive models.
    if (vendorId == 0x00000257UL) {
        return KnownDeviceType::Dunker_75CI;
    }
    // Unknown vendor -> stay honest.
    return KnownDeviceType::Unknown;
}

struct DiscoveredNode {
    bool     seen = false;
    uint8_t  nodeId = 0;
    uint8_t  lastHbState = 0xFF;   // 0x00 bootup, 0x7F preop, 0x05 op, 0x04 stopped
    uint32_t lastSeenMs = 0;

    // SDO identity classification (read after explicit connect)
    bool     sdoOk = false; // at least one identify read succeeded
    uint32_t vendorId = 0;      // 1018h:01
    uint32_t productCode = 0;   // 1018h:02 (ASCII "CAN" for AP04)
    uint32_t revision = 0;      // 1018h:03 (revision number)
    uint32_t serial = 0;        // 1018h:04 (serial number)
    uint8_t  identityEntries = 0; // 1018h:00

    // AP04 module identification (650Ah)
    uint8_t  moduleIdSubs = 0;  // 650Ah:00 (expected 3)
    int32_t  manufacturerOffset = 0; // 650Ah:01 (SIGNED32)
    int32_t  minPosRaw = 0;          // 650Ah:02 (SIGNED32)
    int32_t  maxPosRaw = 0;          // 650Ah:03 (SIGNED32)

    // Optional: in-flight identify state (debug)
    bool     identifyInProgress = false;

    KnownDeviceType known = KnownDeviceType::Unknown;
};

class CanopenMasterController {
public:
    explicit CanopenMasterController(CanopenDriver* drv)
        : m_drv(drv)
    {}

    void begin(uint8_t txPin, uint8_t rxPin, uint32_t baud) {
        m_txPin = txPin;
        m_rxPin = rxPin;
        setBaudrate(baud);
        setMode(MasterMode::Idle);
        resetDiscovery();
    }

    MasterMode mode() const { return m_mode; }
    uint32_t baudrate() const { return m_baud; }

    void setMode(MasterMode m) {
        m_mode = m;
        // hard rule: leaving online mode disables auto actions
        if (m_mode == MasterMode::Idle || m_mode == MasterMode::Scanning) {
            m_onlineNodeId = 0;
            m_knownType = KnownDeviceType::Unknown;
            m_initStage = OnlineInitStage::None;
            m_initDueMs = 0;
            // IMPORTANT: do NOT delete m_nmt/m_sdo here. The CAN RX task may be
            // inside onFrame() dereferencing them; freeing races -> use-after-free
            // crash (Guru Meditation). They are reused on the next connect; the
            // mode guard in onFrame() prevents use while offline.
        }
    }

    void resetDiscovery() {
        for (int i = 0; i < 128; i++) {
            m_nodes[i] = DiscoveredNode{};
            m_nodes[i].nodeId = (uint8_t)i;
        }
    }

    const DiscoveredNode& node(uint8_t nodeId) const { return m_nodes[nodeId]; }
    DiscoveredNode& node(uint8_t nodeId) { return m_nodes[nodeId]; }

    // Passive RX hook: call from global CAN RX callback
    void onFrame(uint32_t cobId, const uint8_t* data, uint8_t len) {
        // Heartbeat/BootUp
        if ((cobId & 0x780) == 0x700 && data && len >= 1) {
            const uint8_t nid = (uint8_t)(cobId - 0x700);
            if (nid >= 1 && nid <= 127) {
                auto& n = m_nodes[nid];
                n.seen = true;
                n.lastHbState = data[0];
                n.lastSeenMs = millis();

                // If we are connected to this node and we see Boot-Up (0x00),
                // delay-start the SDO identity sequence to avoid racing the reboot.
                if (nid == m_onlineNodeId && data[0] == 0x00 && n.identifyInProgress) {
                    m_bootupSeen = true;
                    if (m_initStage == OnlineInitStage::Identify_1018_Vendor) {
                        m_initDueMs = millis() + 250;
                    }
                }
            }
        }

        // Forward to ONLINE NMT manager only when ONLINE
        if ((m_mode == MasterMode::OnlineStandard || m_mode == MasterMode::OnlineKnown) && m_nmt) {
            m_nmt->processFrame(cobId, data, len);
        }
        // Forward to ONLINE SDO client only when ONLINE
        if ((m_mode == MasterMode::OnlineStandard || m_mode == MasterMode::OnlineKnown) && m_sdo) {
            m_sdo->processFrame(cobId, data, len);
        }
    }

    // Scan control
    void startScan() {
        resetDiscovery();
        setMode(MasterMode::Scanning);
        m_scanStartMs = millis();
    }

    // Main tick (call from loop)
    void update() {
        // OnlineStandard and OnlineKnown both run the identification state machine.
        // Known-only behaviors (like AP04 SYNC) are guarded by m_knownType.
        if (m_mode != MasterMode::OnlineStandard && m_mode != MasterMode::OnlineKnown) return;
        if (!m_nmt) return;

        const uint32_t now = millis();

        // Keep SDO timeouts progressing (otherwise a missing response can stall forever)
        if (m_sdo) m_sdo->update();

        // Option B (user decision): emit SYNC immediately after explicit CONNECT,
        // and keep emitting it until identification says otherwise.
        // This prevents AP04 from dropping back to Pre-Op when its "external heartbeat"
        // source is configured to SYNC.
        if (m_syncEnabled) {
            const uint32_t period = (m_syncPeriodMs == 0) ? 100 : m_syncPeriodMs;
            if (now - m_lastSyncTxMs >= period) {
                m_lastSyncTxMs = now;
                m_drv->sendFrame(0x080, (const uint8_t*)nullptr, 0);
            }
        }

        if (m_initStage == OnlineInitStage::None) return;
        if (m_initDueMs != 0 && (int32_t)(now - m_initDueMs) < 0) return;

        // Stage 1: ResetComm -> wait -> Start
        if (m_initStage == OnlineInitStage::KickResetComm) {
            Serial.printf("[MASTER] Init: ResetComm node %u\n", (unsigned)m_onlineNodeId);
            m_nmt->sendResetComm();
            m_initStage = OnlineInitStage::KickStart;
            m_initDueMs = now + 600;
            return;
        }
        if (m_initStage == OnlineInitStage::KickStart) {
            Serial.printf("[MASTER] Init: Start node %u\n", (unsigned)m_onlineNodeId);
            m_nmt->sendStart();
            // After start, run SDO identity reads.
            // We prefer to wait for Boot-Up (0x00) if it arrives, but we won't block.
            m_bootupSeen = false;
            m_initStage = OnlineInitStage::Identify_1018_Entries;
            m_initDueMs = now + 150;
            m_nodes[m_onlineNodeId].identifyInProgress = true;
            return;
        }

        // Stage 2: SDO identity (1018h) read chain
        if (!m_sdo) {
            Serial.println("[MASTER] Identify skipped: no SDO client");
            m_nodes[m_onlineNodeId].identifyInProgress = false;
            m_initStage = OnlineInitStage::Done;
            m_initDueMs = 0;
            return;
        }

        auto& nd = m_nodes[m_onlineNodeId];

        auto scheduleNext = [&](OnlineInitStage next, uint32_t delayMs) {
            m_initStage = next;
            m_initDueMs = now + delayMs;
        };

        // One request at a time; SdoClient is async and uses callbacks.
        if (m_sdo->isBusy()) {
            // let SDO complete
            return;
        }

        if (m_initStage == OnlineInitStage::Identify_1018_Entries) {
            // If the node rebooted due to ResetComm, try to wait for Boot-Up (0x00)
            // to avoid sending SDO too early. But do NOT block forever.
            if (!m_bootupSeen && now < (m_connectMs + 600)) {
                m_initDueMs = now + 100;
                return;
            }

            Serial.printf("[MASTER] Identify: read 1018h:00 (Entries) node %u\n", (unsigned)m_onlineNodeId);
            m_sdo->readAsync(0x1018, 0x00, [this](SdoResult r, uint32_t v){
                auto& n = m_nodes[m_onlineNodeId];
                if (r == SDO_OK) {
                    n.identityEntries = (uint8_t)(v & 0xFF);
                    n.sdoOk = true;
                } else {
                    Serial.printf("[MASTER] Identify: 1018h:00 failed (%d)\n", (int)r);
                }
            });
            scheduleNext(OnlineInitStage::Identify_1018_Vendor, 20);
            return;
        }

        if (m_initStage == OnlineInitStage::Identify_1018_Vendor) {
            Serial.printf("[MASTER] Identify: read 1018h:01 (Vendor-ID) node %u\n", (unsigned)m_onlineNodeId);
            m_sdo->readAsync(0x1018, 0x01, [this](SdoResult r, uint32_t v){
                auto& n = m_nodes[m_onlineNodeId];
                if (r == SDO_OK) {
                    n.vendorId = v;
                    n.sdoOk = true;
                } else {
                    Serial.printf("[MASTER] Identify: 1018h:01 failed (%d)\n", (int)r);
                }
            });
            scheduleNext(OnlineInitStage::Identify_1018_Product, 20);
            return;
        }

        if (m_initStage == OnlineInitStage::Identify_1018_Product) {
            Serial.printf("[MASTER] Identify: read 1018h:02 (Product-Code) node %u\n", (unsigned)m_onlineNodeId);
            m_sdo->readAsync(0x1018, 0x02, [this](SdoResult r, uint32_t v){
                auto& n = m_nodes[m_onlineNodeId];
                if (r == SDO_OK) {
                    n.productCode = v;
                    n.sdoOk = true;
                } else {
                    Serial.printf("[MASTER] Identify: 1018h:02 failed (%d)\n", (int)r);
                }
            });
            scheduleNext(OnlineInitStage::Identify_650A_Subs, 20);
            return;
        }

        if (m_initStage == OnlineInitStage::Identify_650A_Subs) {
            Serial.printf("[MASTER] Identify: read 650Ah:00 (Sub count) node %u\n", (unsigned)m_onlineNodeId);
            m_sdo->readAsync(0x650A, 0x00, [this](SdoResult r, uint32_t v){
                auto& n = m_nodes[m_onlineNodeId];
                if (r == SDO_OK) {
                    n.moduleIdSubs = (uint8_t)(v & 0xFF);
                    n.sdoOk = true;
                } else {
                    Serial.printf("[MASTER] Identify: 650Ah:00 failed (%d)\n", (int)r);
                }
            });
            scheduleNext(OnlineInitStage::Identify_650A_Offset, 20);
            return;
        }

        if (m_initStage == OnlineInitStage::Identify_650A_Offset) {
            Serial.printf("[MASTER] Identify: read 650Ah:01 (Offset) node %u\n", (unsigned)m_onlineNodeId);
            m_sdo->readAsync(0x650A, 0x01, [this](SdoResult r, uint32_t v){
                auto& n = m_nodes[m_onlineNodeId];
                if (r == SDO_OK) {
                    n.manufacturerOffset = (int32_t)v;
                    n.sdoOk = true;
                } else {
                    Serial.printf("[MASTER] Identify: 650Ah:01 failed (%d)\n", (int)r);
                }
            });
            scheduleNext(OnlineInitStage::Identify_650A_Min, 20);
            return;
        }

        if (m_initStage == OnlineInitStage::Identify_650A_Min) {
            Serial.printf("[MASTER] Identify: read 650Ah:02 (Min) node %u\n", (unsigned)m_onlineNodeId);
            m_sdo->readAsync(0x650A, 0x02, [this](SdoResult r, uint32_t v){
                auto& n = m_nodes[m_onlineNodeId];
                if (r == SDO_OK) {
                    n.minPosRaw = (int32_t)v;
                    n.sdoOk = true;
                } else {
                    Serial.printf("[MASTER] Identify: 650Ah:02 failed (%d)\n", (int)r);
                }
            });
            scheduleNext(OnlineInitStage::Identify_650A_Max, 20);
            return;
        }

        if (m_initStage == OnlineInitStage::Identify_650A_Max) {
            Serial.printf("[MASTER] Identify: read 650Ah:03 (Max) node %u\n", (unsigned)m_onlineNodeId);
            m_sdo->readAsync(0x650A, 0x03, [this](SdoResult r, uint32_t v){
                auto& n = m_nodes[m_onlineNodeId];
                if (r == SDO_OK) {
                    n.maxPosRaw = (int32_t)v;
                    n.sdoOk = true;
                } else {
                    Serial.printf("[MASTER] Identify: 650Ah:03 failed (%d)\n", (int)r);
                }
            });

            // Finalize classification on next update tick.
            scheduleNext(OnlineInitStage::Done, 50);
            return;
        }

        if (m_initStage == OnlineInitStage::Done) {
            nd.identifyInProgress = false;

            // Classify AP04 based on manual facts:
            // - 1018h:00 default entries = 2
            // - 1018h:01 vendor-id for SIKO = 0x00000195
            // - 1018h:02 product-code ASCII "CAN" = 0x004E4143
            // - 650Ah:00 has 3 sub-indices (offset/min/max)
            const bool looksLikeSikoVendor = (nd.vendorId == 0x00000195UL);
            const bool looksLikeCanVariant = (nd.productCode == 0x004E4143UL);
            const bool looksLikeAp04ModuleId = (nd.moduleIdSubs == 3);

            const bool isAp04 = looksLikeAp04ModuleId || (looksLikeSikoVendor && looksLikeCanVariant);

            if (isAp04) {
                nd.known = KnownDeviceType::SIKO_AP04;
                m_knownType = nd.known;

                Serial.printf("[MASTER] Identify AP04: entries=%u vendor=0x%08lX product=0x%08lX (\"CAN\") 650A:00=%u -> SIKO_AP04\n",
                    (unsigned)nd.identityEntries,
                    (unsigned long)nd.vendorId,
                    (unsigned long)nd.productCode,
                    (unsigned)nd.moduleIdSubs);

                // Keep SYNC enabled for AP04.
                m_syncEnabled = true;

                if (m_mode != MasterMode::OnlineKnown) setMode(MasterMode::OnlineKnown);
            } else if (nd.sdoOk) {
                // Got SDO identity but it's not AP04 -> classify by vendor/product
                // (Dunker, SIKO AP10, ...). This keeps a connected Dunker recognised
                // so the UI can route to its DS402 page.
                nd.known    = classifyByIdentity(nd.vendorId, nd.productCode);
                m_knownType = nd.known;

                Serial.printf("[MASTER] Identify (non-AP04): vendor=0x%08lX product=0x%08lX -> %s\n",
                    (unsigned long)nd.vendorId,
                    (unsigned long)nd.productCode,
                    (nd.known == KnownDeviceType::Dunker_75CI) ? "Dunker_75CI" :
                    (nd.known == KnownDeviceType::SIKO_AP10)   ? "SIKO_AP10"   : "Unknown");

                // Non-AP04 devices don't need master SYNC (Dunker uses DS402 controlword).
                m_syncEnabled = false;

                if (nd.known == KnownDeviceType::Unknown) {
                    if (m_mode != MasterMode::OnlineStandard) setMode(MasterMode::OnlineStandard);
                } else {
                    if (m_mode != MasterMode::OnlineKnown) setMode(MasterMode::OnlineKnown);
                }
            } else {
                Serial.println("[MASTER] Identify failed/timeout -> Unknown (keep SYNC ON for safety until disconnect)");
                nd.known = KnownDeviceType::Unknown;
                m_knownType = KnownDeviceType::Unknown;

                // Safety choice: do NOT turn SYNC off here.
                // If the device is AP04 with external heartbeat=SYNC, turning it off would drop it to Pre-Op.
                m_syncEnabled = true;

                if (m_mode != MasterMode::OnlineStandard) setMode(MasterMode::OnlineStandard);
            }

            // Stop init sequence.
            m_initStage = OnlineInitStage::None;
            m_initDueMs = 0;
            return;
        }
    }

    bool scanDone(uint32_t minDurationMs = 1200) const {
        return (m_mode == MasterMode::Scanning) && (millis() - m_scanStartMs >= minDurationMs);
    }

    // Online connect: only explicit user action
    void connectNode(uint8_t nodeId, KnownDeviceType known = KnownDeviceType::Unknown) {
        if (nodeId < 1 || nodeId > 127) return;
        m_onlineNodeId = nodeId;

        // We do NOT guess by Node-ID anymore.
        // If the UI pre-classified the node (e.g. user selected a known template), it may pass it in.
        // Otherwise we start as Unknown and identify via SDO.
        m_knownType = known;

        // Reuse the NMT/SDO objects instead of delete+new: the RX task may be
        // using them concurrently, and freeing them races -> crash. Allocate
        // once (lazily), then just re-point them at the new node-id.
        if (!m_nmt) m_nmt = new NmtManager(m_drv, nodeId);
        else        m_nmt->setNodeId(nodeId);
        if (!m_sdo) m_sdo = new SdoClient(m_drv, nodeId);
        else        m_sdo->setNodeId(nodeId);
        if (m_sdo) m_sdo->setTimeout(800);

        // Start in Standard unless we already know it's a known device.
        setMode(m_knownType == KnownDeviceType::Unknown ? MasterMode::OnlineStandard : MasterMode::OnlineKnown);

        // Option B: enable SYNC immediately after explicit connect.
        // We'll turn it off again if identify says "Unknown".
        m_syncEnabled = true;

        m_connectMs = millis();
        m_bootupSeen = false;

        // Always run the init chain (ResetComm -> Start -> Identify).
        m_initStage = OnlineInitStage::KickResetComm;
        m_initDueMs = millis() + 50;

        // Mark discovery record
        m_nodes[nodeId].seen = true;
        m_nodes[nodeId].lastSeenMs = millis();
        m_nodes[nodeId].identifyInProgress = true;
        m_nodes[nodeId].sdoOk = false;
        m_nodes[nodeId].known = KnownDeviceType::Unknown;
    }

    void disconnect() {
        setMode(MasterMode::Idle);
    }

    // NMT user actions (no automatic recovery here)
    bool nmtStart() { return m_nmt ? m_nmt->sendStart() : false; }
    bool nmtStop() { return m_nmt ? m_nmt->sendStop() : false; }
    bool nmtPreOp() { return m_nmt ? m_nmt->sendPreOperational() : false; }
    bool nmtResetNode() { return m_nmt ? m_nmt->sendResetNode() : false; }
    bool nmtResetComm() { return m_nmt ? m_nmt->sendResetComm() : false; }

    // -----------------------------------------------------------------------
    // AP04 configuration helpers (used by UI after explicit connect)
    // -----------------------------------------------------------------------

    // Robust write for 5F0Ah (Node-ID). We try 1 byte first, then 2 bytes.
    // On success we send ResetComm so the new node-id becomes active.
    void ap04SetNodeIdRobust(uint8_t newNodeId) {
        if (!m_sdo || !m_nmt || m_onlineNodeId == 0) {
            Serial.println("[MASTER] ap04SetNodeIdRobust: not connected");
            return;
        }
        if (newNodeId < 1 || newNodeId > 127) {
            Serial.println("[MASTER] ap04SetNodeIdRobust: invalid range (1..127)");
            return;
        }
        if (m_sdo->isBusy()) {
            Serial.println("[MASTER] ap04SetNodeIdRobust: SDO busy");
            return;
        }

        Serial.printf("[MASTER] AP04: set Node-ID via 5F0Ah to %u (robust)\n", (unsigned)newNodeId);

        // Attempt 1: UNSIGNED8 @ 5F0Ah:00
        const uint8_t sub = 0x00;
        const uint16_t idx = 0x5F0A;

        bool ok = m_sdo->writeAsync(idx, sub, (uint32_t)newNodeId, 1,
            [this, newNodeId](SdoResult r, uint32_t){
                if (r == SDO_OK) {
                    Serial.println("[MASTER] AP04: 5F0Ah write OK (1 byte) -> ResetComm");
                    m_nmt->sendResetComm();
                    return;
                }

                Serial.printf("[MASTER] AP04: 5F0Ah write (1 byte) failed (%d) -> try 2 bytes\n", (int)r);

                // Attempt 2: UNSIGNED16 (some devices store node-id as 16-bit)
                if (!m_sdo || m_sdo->isBusy()) return;
                m_sdo->writeAsync(0x5F0A, 0x00, (uint32_t)newNodeId, 2,
                    [this](SdoResult r2, uint32_t){
                        if (r2 == SDO_OK) {
                            Serial.println("[MASTER] AP04: 5F0Ah write OK (2 bytes) -> ResetComm");
                            m_nmt->sendResetComm();
                        } else {
                            Serial.printf("[MASTER] AP04: 5F0Ah write (2 bytes) failed (%d)\n", (int)r2);
                        }
                    }
                );
            }
        );

        if (!ok) Serial.println("[MASTER] AP04: writeAsync start failed");
    }

    // Baudrate switching (for scan): stop driver and re-init
    bool setBaudrate(uint32_t baud) {
        m_baud = baud;
        if (!m_drv) return false;
        m_drv->deinit();
        if (!m_drv->init(m_txPin, m_rxPin, m_baud)) return false;
        m_drv->setRxCallback(m_rxCb);
        return m_drv->start();
    }

    // Bind callback pointer (sketch sets this once)
    void setRxCallback(CanFrameCallback cb) {
        m_rxCb = cb;
        if (m_drv) m_drv->setRxCallback(cb);
    }

private:
    CanopenDriver* m_drv = nullptr;
    MasterMode m_mode = MasterMode::Idle;

    uint8_t  m_txPin = 0;
    uint8_t  m_rxPin = 0;
    uint32_t m_baud = 250000;

    DiscoveredNode m_nodes[128];

    // scan
    uint32_t m_scanStartMs = 0;

    // online
    uint8_t m_onlineNodeId = 0;
    KnownDeviceType m_knownType = KnownDeviceType::Unknown;

    // connect/boot tracking
    uint32_t m_connectMs = 0;
    bool     m_bootupSeen = false;

    NmtManager* m_nmt = nullptr;
    SdoClient*  m_sdo = nullptr;

    OnlineInitStage m_initStage = OnlineInitStage::None;
    uint32_t m_initDueMs = 0;

    // SYNC (Option B): enabled immediately after explicit connect,
    // then kept or disabled after SDO identity classification.
    bool     m_syncEnabled = false;
    uint32_t m_lastSyncTxMs = 0;
    uint16_t m_syncPeriodMs = 100;

    CanFrameCallback m_rxCb = nullptr;
};
