/**
 * @file SIKO_AP10_Test_sniffer_on.ino
 * @brief Universal CANopen Master + CAN Sniffer (Waveshare ESP32-S3-Touch-LCD-4.3B)
 * @date 2026
 *
 * NOTES:
 * - Sniffer is decoupled via FreeRTOS queue + task (SnifferManager)
 * - Sniffer is ON/OFF via UI (initially OFF)
 * - No LVGL calls in CAN RX callback
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>

#include "SnifferManager.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

#include "lvgl_v8_port.h"

// UI
#include "src/ui/start_menu_ui.h"
#include "src/ui/node_detail_ui.h"
#include "src/ui/siko_ap_ui.h"

// Universal master state machine
#include "src/canopen/canopen_master_controller.h"
#include "src/canopen/sdo_client.h"

// CAN driver (loopback-capable variant)
#include "src/canopen/canopen_driver.h"

// AP04 device binding
#include "src/devices/siko_ap04.h"

// Dunker DS402 drive + UI (Milestone 4)
#include "src/devices/dunker_drive.h"
#include "src/ui/dunker_ui.h"

// ============================================================================
// Konfiguration
// ============================================================================

struct Config {
    static constexpr uint8_t  CAN_TX_PIN = 15;
    static constexpr uint8_t  CAN_RX_PIN = 16;

    static constexpr uint32_t CAN_DEFAULT_BAUDRATE = 250000;

    static constexpr uint32_t UI_UPDATE_MS    = 200;
    static constexpr uint32_t STATUS_PRINT_MS = 5000;

    static constexpr uint32_t SCAN_WINDOW_MS = 1200;

    // CANopen practical baudrates. 10k/20k are intentionally omitted: they need a
    // large TWAI prescaler that some cores/SoCs don't expose, and CANopen devices
    // virtually never run that slow.
    static constexpr uint32_t SCAN_BAUDS[] = {
        1000000, 500000, 250000, 125000, 100000, 50000
    };
    static constexpr uint8_t SCAN_BAUD_COUNT = sizeof(SCAN_BAUDS) / sizeof(SCAN_BAUDS[0]);
};

// ============================================================================
// Globals
// ============================================================================

Board*        panel = nullptr;
CanopenDriver canDriver;

static SnifferManager sniffer;
static bool snifferEnabled = false; // UI-controlled
static bool loopbackEnabled = false; // UI-controlled (No-ACK mode)

// Centralized CAN re-init request (prevents double-reinit races)
static volatile bool canReinitPending = false;
static uint32_t      canReinitBaud = Config::CAN_DEFAULT_BAUDRATE;
static bool          canReinitLoopback = false;
static bool          canReinitInProgress = false;

static uint32_t lastUiUpdate    = 0;
static uint32_t lastStatusPrint = 0;

static StartMenuUI startUi;
static NodeDetailUI nodeUi;
static AP10_UI ap04Ui; // reuse the existing AP10/AP04 UI page as AP04 page
static CanopenMasterController master(&canDriver);

static uint8_t selectedNodeId = 0;

// Dunker DS402 page (node-independent; bound to the selected node on connect)
static DunkerUI    dunkerUi;
static DunkerDrive* dunkerDev = nullptr;
static bool        dunkerPageLoaded = false;
static bool        dunkerUiIsActive = false;

// SDO ping scan (1..32) on the currently selected baud (no baud switching)
static bool pingScanRunning = false;
static uint8_t pingScanNode = 1;
static uint32_t pingScanLastSendMs = 0;
static bool pingScanFound[33] = {false};
static uint8_t pingScanFoundCount = 0;
static uint32_t pingScanBaudSnapshot = 0;

// Identify-All: iterate over ALL nodes found by the last ping scan and read 1018h
static bool identifyAllRunning = false;
static uint8_t identifyAllNode = 0;   // node currently being identified (1..32)
static uint8_t identifyAllStep = 0;   // 0..3 = read sub 01..04, then finalize
static uint32_t identifyAllNextMs = 0;
static SdoClient identifySdo(&canDriver, 1);

// Standard node-detail page: on-demand read of 1018h (identity) + 1001h (error register)
static SdoClient stdSdo(&canDriver, 1);
static bool stdReadRunning = false;
static uint8_t stdReadNode = 0;
static uint8_t stdReadStep = 0;
static uint32_t stdReadNextMs = 0;
static uint8_t stdErrReg = 0;
static bool stdErrRegValid = false;

// AP04 page/device state
static SikoAP04* ap04Dev = nullptr;
static bool ap04PageLoaded = false;
static bool ap04UiIsActive = false;

// Navigation flags set from UI callback; handled in loop()
static volatile bool navPendingDisconnect = false;
static volatile bool navOpenNodeDetail = false;
static volatile bool navPendingUiSwitch = false;

// Deferred reconnect after AP04 Node-ID change
static volatile bool ap04PendingReconnect = false;
static uint32_t ap04ReconnectDueMs = 0;
static uint8_t  ap04ReconnectNewNodeId = 0;

// Scan state
static bool     scanRunning = false;
static uint8_t  scanBaudIdx = 0;
static uint32_t scanStepStartMs = 0;
static uint32_t activeBaud = Config::CAN_DEFAULT_BAUDRATE;

// ============================================================================
// CAN RX Callback
// ============================================================================

static void onCanFrame(uint32_t cobId, const uint8_t* data, uint8_t length)
{
    // Feed sniffer (task context, OK). ext/rtr not available in this callback -> false/false.
    sniffer.processFrame(cobId, length, data, false, false);

    // Dunker DS402 drive (only while its page is active)
    if (dunkerUiIsActive && dunkerDev) {
        dunkerDev->processFrame(cobId, data, length);
    }

    // Identify SDO client
    identifySdo.processFrame(cobId, data, length);

    // Standard node-detail SDO client (1018h/1001h on-demand reads)
    stdSdo.processFrame(cobId, data, length);

    // Ping scan detect: any SDO response in 0x581..0x5A0 marks node found
    if (pingScanRunning && cobId >= 0x581 && cobId <= 0x5A0) {
        const uint8_t nid = (uint8_t)(cobId - 0x580);
        if (nid >= 1 && nid <= 32 && !pingScanFound[nid]) {
            pingScanFound[nid] = true;
            pingScanFoundCount++;
            // Make ping-discovered nodes appear in the start-menu list
            auto& dn = master.node(nid);
            dn.seen = true;
            dn.lastSeenMs = millis();
            Serial.printf("[PING] Node %u responded (COB-ID=0x%03lX)\n", (unsigned)nid, (unsigned long)cobId);
        }
    }

    // Existing master/device pipeline
    master.onFrame(cobId, data, length);

    if (ap04UiIsActive && ap04Dev) {
        ap04Dev->processFrame(cobId, data, length);
    }
}

// ============================================================================
// Display init
// ============================================================================

static bool initDisplay()
{
    Serial.println("[Display] Starte...");

    panel = new Board();
    if (!panel) { Serial.println("[Display] ERROR: new Board() fehlgeschlagen"); return false; }

    if (!panel->init()) {
        Serial.println("[Display] ERROR: init() fehlgeschlagen");
        return false;
    }

    auto* lcd = panel->getLCD();
    if (!lcd) { Serial.println("[Display] ERROR: getLCD() null"); return false; }

#if LVGL_PORT_AVOID_TEARING_MODE
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
    Serial.printf("[Display] FrameBuffer Anzahl: %d\n", LVGL_PORT_DISP_BUFFER_NUM);
#endif

#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    {
        auto* bus = lcd->getBus();
        if (bus && bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
            static_cast<BusRGB*>(bus)->configRGB_BounceBufferSize(800 * 20);
            Serial.println("[Display] Bounce-Buffer gesetzt (800*20)");
        }
    }
#endif

    if (!panel->begin()) {
        Serial.println("[Display] ERROR: begin() fehlgeschlagen");
        return false;
    }
    Serial.println("[Display] Panel OK");

    auto* touch = panel->getTouch();
    Serial.println("[Display] Starte LVGL Port...");
    if (!lvgl_port_init(lcd, touch)) {
        Serial.println("[Display] ERROR: lvgl_port_init fehlgeschlagen");
        return false;
    }

    Serial.println("[Display] LVGL OK");
    return true;
}

// ============================================================================
// Helpers
// ============================================================================

static const char* hbStateToText(uint8_t st)
{
    switch (st) {
        case 0x00: return "Boot-Up";
        case 0x7F: return "Pre-Op";
        case 0x05: return "Operational";
        case 0x04: return "Stopped";
        default:   return "Unknown";
    }
}

static NodeDetail_Info makeNodeInfo(uint8_t nid)
{
    NodeDetail_Info info;
    info.nodeId = nid;

    const auto& dn = master.node(nid);
    info.stateText = dn.seen ? hbStateToText(dn.lastHbState) : "---";

    if (dn.identifyInProgress) info.typeText = "Identifying...";
    else {
        info.typeText  = (dn.known == KnownDeviceType::SIKO_AP04) ? "SIKO AP04" :
                         (dn.known == KnownDeviceType::SIKO_AP10) ? "SIKO AP10" :
                         (dn.known == KnownDeviceType::Dunker_75CI) ? "Dunker 75CI" :
                         "Unknown";
    }

    const uint32_t now = millis();
    info.lastSeenMs = (dn.seen && dn.lastSeenMs > 0) ? (now - dn.lastSeenMs) : 0;

    // Identity (from discovery / identify-all / on-demand READ INFO)
    info.identityKnown = dn.sdoOk;
    info.vendorId      = dn.vendorId;
    info.productCode   = dn.productCode;
    info.revision      = dn.revision;
    info.serial        = dn.serial;

    // Error register (1001h) is valid only for the node we last read
    info.errRegValid = (stdErrRegValid && stdReadNode == nid);
    info.errReg      = stdErrReg;
    return info;
}

static void refreshStartMenuList()
{
    StartMenu_NodeRow rows[128];
    size_t n = 0;

    const uint32_t now = millis();
    for (uint8_t nid = 1; nid <= 127; nid++) {
        const auto& dn = master.node(nid);
        if (!dn.seen) continue;

        rows[n].nodeId = nid;
        rows[n].stateText = hbStateToText(dn.lastHbState);
        rows[n].lastSeenMs = (dn.lastSeenMs > 0) ? (now - dn.lastSeenMs) : 0;

        switch (dn.known) {
            case KnownDeviceType::SIKO_AP04:  rows[n].typeText = "SIKO AP04"; break;
            case KnownDeviceType::SIKO_AP10:  rows[n].typeText = "SIKO AP10"; break;
            case KnownDeviceType::Dunker_75CI:rows[n].typeText = "Dunker 75CI"; break;
            default:                          rows[n].typeText = "Unknown"; break;
        }

        n++;
        if (n >= 128) break;
    }

    startUi.setNodes(rows, n);
}

static void refreshNodeDetail()
{
    if (selectedNodeId < 1 || selectedNodeId > 127) return;
    nodeUi.setInfo(makeNodeInfo(selectedNodeId));
}

static void refreshAp04()
{
    if (!ap04Dev) return;
    ap04Ui.updateAP04(ap04Dev->getData());
}

static void requestCanReinit(uint32_t baud, bool loopNoAck)
{
    canReinitBaud = baud;
    canReinitLoopback = loopNoAck;
    canReinitPending = true;
}

static void startAutoBaudScan()
{
    scanRunning = true;
    scanBaudIdx = 0;
    scanStepStartMs = millis();

    master.setMode(MasterMode::Scanning);
    master.resetDiscovery();

    Serial.println("[SCAN] Auto-Baud Scan gestartet");

    activeBaud = Config::SCAN_BAUDS[scanBaudIdx];
    Serial.printf("[SCAN] Set baud %lu\n", (unsigned long)activeBaud);

    requestCanReinit(activeBaud, loopbackEnabled);
}

static void scanTick(uint32_t now)
{
    if (!scanRunning) return;
    if (now - scanStepStartMs < Config::SCAN_WINDOW_MS) return;

    bool anySeen = false;
    for (uint8_t nid = 1; nid <= 127; nid++) {
        if (master.node(nid).seen) { anySeen = true; break; }
    }

    if (anySeen) {
        Serial.printf("[SCAN] Treffer @ %lu bps\n", (unsigned long)activeBaud);
        scanRunning = false;
        lvgl_port_lock(-1);
        refreshStartMenuList();
        lvgl_port_unlock();
        master.setMode(MasterMode::Idle);
        return;
    }

    scanBaudIdx++;
    if (scanBaudIdx >= Config::SCAN_BAUD_COUNT) {
        Serial.println("[SCAN] Kein CANopen Traffic erkannt (alle Baudraten getestet)");
        scanRunning = false;
        master.setMode(MasterMode::Idle);
        lvgl_port_lock(-1);
        refreshStartMenuList();
        lvgl_port_unlock();
        return;
    }

    scanStepStartMs = now;
    activeBaud = Config::SCAN_BAUDS[scanBaudIdx];
    Serial.printf("[SCAN] Set baud %lu\n", (unsigned long)activeBaud);

    master.resetDiscovery();
    requestCanReinit(activeBaud, loopbackEnabled);
}

// ============================================================================
// Setup / Loop
// ============================================================================

void setup()
{
    Serial.begin(115200);
    delay(50);
    Serial.println("\n[APP] Universal CANopen Master boot");

    // Sniffer backend
    {
        SnifferManager::Config scfg;
        scfg.queueLen = 128;
        scfg.maxRecentFrames = 120;
        sniffer.begin(scfg);
        sniffer.setEnabled(false);        // Sniffer ON/OFF
        sniffer.setSerialOutput(true);    // Serial output in Phase A
    }

    if (!initDisplay()) {
        Serial.println("[APP] FATAL: Display fehlgeschlagen!");
        while (true) delay(1000);
    }

    // CAN init (default baud)
    canDriver.setLoopbackEnabled(loopbackEnabled);
    if (!canDriver.init(Config::CAN_TX_PIN, Config::CAN_RX_PIN, Config::CAN_DEFAULT_BAUDRATE)) {
        Serial.println("[CAN] ERROR: init fehlgeschlagen!");
    } else {
        canDriver.setRxCallback(onCanFrame);
        if (!canDriver.start()) {
            Serial.println("[CAN] ERROR: start fehlgeschlagen!");
        } else {
            Serial.println("[CAN] Gestartet!");
        }
    }

    master.begin(Config::CAN_TX_PIN, Config::CAN_RX_PIN, Config::CAN_DEFAULT_BAUDRATE);
    master.setRxCallback(onCanFrame);

    // Track initial applied state for the centralized re-init path
    canReinitBaud = Config::CAN_DEFAULT_BAUDRATE;
    canReinitLoopback = loopbackEnabled;

    // UI: Start Menu + Node Detail
    lvgl_port_lock(-1);

    // Start menu callbacks
    StartMenu_Callbacks cbs;
    cbs.onScanAuto = [](){
        startAutoBaudScan();
    };

    // Sniffer toggle (Start Menu)
    cbs.onSnifferToggle = [](bool en){
        snifferEnabled = en;
        sniffer.setEnabled(en);
        Serial.printf("[SNIFF] Sniffer toggled via UI: %s\n", en ? "ON" : "OFF");
    };

    // Fixed baud quick-set (125k/250k/500k) (Tools screen)
    cbs.onSetFixedBaud = [](uint32_t baud){
        Serial.printf("[UI] Set fixed baud: %lu\n", (unsigned long)baud);
        scanRunning = false; // stop auto scan if running
        activeBaud = baud;

        requestCanReinit(activeBaud, loopbackEnabled);

        // reflect in UI
        lvgl_port_lock(-1);
        startUi.setActiveBaud(activeBaud);
        lvgl_port_unlock();
    };

    // NMT start broadcast (Start Remote Node, all nodes)
    cbs.onNmtStartBroadcast = [](){
        Serial.println("[UI] NMT START ALL -> send 0x000: {01,00}");
        uint8_t msg[2] = { 0x01, 0x00 };
        canDriver.sendFrame(0x000, msg, 2);
    };

    // SDO ping scan on fixed, already-selected baud (no baud switching)
    cbs.onPingScan1_32 = [](){
        pingScanRunning = true;
        pingScanNode = 1;
        pingScanLastSendMs = 0;
        pingScanFoundCount = 0;
        identifyAllRunning = false;
        memset(pingScanFound, 0, sizeof(pingScanFound));
        pingScanBaudSnapshot = activeBaud;
        Serial.printf("[PING] Start SDO ping scan 1..32 @ %lu bps\n", (unsigned long)pingScanBaudSnapshot);

        // Helpful: ensure sniffer is ON so user sees traffic
        if (!snifferEnabled) {
            snifferEnabled = true;
            sniffer.setEnabled(true);
            startUi.setSnifferEnabled(true);
            Serial.println("[SNIFF] Auto-enabled because ping scan started");
        }
    };

    // Identify ALL nodes found by the last ping scan (read 1018h vendor/product/rev/serial)
    cbs.onIdentifyAll = [](){
        if (identifyAllRunning) {
            Serial.println("[IDENT] Busy");
            return;
        }
        if (pingScanFoundCount == 0) {
            Serial.println("[IDENT] No nodes found (run PING SCAN first)");
            return;
        }
        if (activeBaud != pingScanBaudSnapshot) {
            Serial.println("[IDENT] Baud differs from last scan; please keep baud fixed");
            return;
        }

        // Start at the first found node
        identifyAllNode = 0;
        for (uint8_t i = 1; i <= 32; i++) {
            if (pingScanFound[i]) { identifyAllNode = i; break; }
        }
        if (identifyAllNode == 0) return;

        identifySdo.setTimeout(400);
        identifyAllRunning = true;
        identifyAllStep = 0;
        identifyAllNextMs = millis();
        Serial.printf("[IDENT-ALL] Start identify for %u node(s)\n", (unsigned)pingScanFoundCount);
    };

    // Loopback toggle (NO_ACK) - centralized re-init
    cbs.onLoopbackToggle = [](bool en){
        loopbackEnabled = en;
        Serial.printf("[UI] Loopback(No-ACK) -> %s (re-init CAN)\n", en ? "ON" : "OFF");

        // Optional convenience: enable sniffer automatically when loopback is ON
        if (en && !snifferEnabled) {
            snifferEnabled = true;
            sniffer.setEnabled(true);
            startUi.setSnifferEnabled(true);
            Serial.println("[SNIFF] Auto-enabled because loopback is ON");
        }

        requestCanReinit(activeBaud, loopbackEnabled);

        // Keep UI in sync
        lvgl_port_lock(-1);
        startUi.setActiveBaud(activeBaud);
        lvgl_port_unlock();
    };

    cbs.onOpenNode = [](uint8_t nodeId){
        Serial.printf("[UI] Node selected: %u -> open detail\n", (unsigned)nodeId);
        selectedNodeId = nodeId;
        lvgl_port_lock(-1);
        refreshNodeDetail();
        nodeUi.load();
        lvgl_port_unlock();
    };

    cbs.onConnectNode = [](uint8_t nodeId){
        Serial.printf("[UI] CONNECT node: %u\n", (unsigned)nodeId);
        selectedNodeId = nodeId;

        const KnownDeviceType kt = KnownDeviceType::Unknown;
        master.connectNode(nodeId, kt);

        lvgl_port_lock(-1);
        refreshNodeDetail();
        nodeUi.load();
        lvgl_port_unlock();
    };

    startUi.setCallbacks(cbs);
    startUi.create();

    // Node detail callbacks
    NodeDetail_Callbacks ncbs;
    ncbs.onBack = [](){
        Serial.println("[UI] Back to start menu");
        lv_scr_load_anim(startUi.screen(), LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    };

    ncbs.onNmtStart = [](uint8_t nodeId){
        Serial.println("[UI] NMT START");
        if (master.mode() == MasterMode::Idle || master.mode() == MasterMode::Scanning) {
            master.connectNode(nodeId, KnownDeviceType::Unknown);
        }
        master.nmtStart();
    };
    ncbs.onNmtPreOp = [](uint8_t){ Serial.println("[UI] NMT PRE-OP"); master.nmtPreOp(); };
    ncbs.onNmtStop = [](uint8_t){ Serial.println("[UI] NMT STOP"); master.nmtStop(); };
    ncbs.onNmtResetNode = [](uint8_t){ Serial.println("[UI] NMT RESET NODE"); master.nmtResetNode(); };
    ncbs.onNmtResetComm = [](uint8_t){ Serial.println("[UI] NMT RESET COMM"); master.nmtResetComm(); };

    // Read identity (1018h) + error register (1001h) on demand for the open node
    ncbs.onReadInfo = [](uint8_t nodeId){
        if (stdReadRunning) { Serial.println("[STD] Busy"); return; }
        if (nodeId < 1 || nodeId > 127) return;
        if (stdSdo.isBusy()) { Serial.println("[STD] SDO busy"); return; }
        stdReadNode = nodeId;
        stdSdo.setNodeId(nodeId);
        stdSdo.setTimeout(400);
        stdErrRegValid = false;
        stdReadRunning = true;
        stdReadStep = 0;
        stdReadNextMs = millis();
        Serial.printf("[STD] Read 1018h+1001h for node %u\n", (unsigned)nodeId);
    };

    ncbs.onSetNodeId = [](uint8_t nodeId, uint8_t newNodeId){
        Serial.printf("[UI] SET NODE-ID node=%u -> %u\n", (unsigned)nodeId, (unsigned)newNodeId);
        if (master.mode() == MasterMode::Idle || master.mode() == MasterMode::Scanning) {
            master.connectNode(nodeId, KnownDeviceType::Unknown);
        }
        master.ap04SetNodeIdRobust(newNodeId);

        selectedNodeId = newNodeId;
        ap04PendingReconnect = true;
        ap04ReconnectNewNodeId = newNodeId;
        ap04ReconnectDueMs = millis() + 1200;
    };

    nodeUi.setCallbacks(ncbs);
    nodeUi.create();

    lvgl_port_unlock();

    // Initial list (empty)
    lvgl_port_lock(-1);
    refreshStartMenuList();
    lvgl_port_unlock();

    Serial.println("[UI] Start menu ready");
    Serial.println("[APP] Ready (IDLE, no auto-connect)");

    // Default: sniffer off until UI toggles it
    snifferEnabled = false;
    sniffer.setEnabled(snifferEnabled);
    startUi.setSnifferEnabled(snifferEnabled);
    startUi.setActiveBaud(activeBaud);
    Serial.println("[SNIFF] Sniffer is OFF (toggle via Start Menu)");
}

void loop()
{
    const uint32_t now = millis();

    // Scan state machine tick
    scanTick(now);

    // Centralized CAN re-init handler (baud + loopback)
    if (canReinitPending && !canReinitInProgress) {
        canReinitInProgress = true;
        canReinitPending = false;

        // Ensure loopback mode is applied BEFORE (re)install
        canDriver.setLoopbackEnabled(canReinitLoopback);

        Serial.printf("[APP] Re-init CAN: baud=%lu mode=%s\n",
                      (unsigned long)canReinitBaud,
                      canReinitLoopback ? "NO_ACK" : "NORMAL");

        master.setBaudrate(canReinitBaud);
        activeBaud = canReinitBaud;

        canReinitInProgress = false;
    }

    // Deferred disconnect BEFORE master.update()
    if (navPendingDisconnect) {
        Serial.println("[APP] Deferred disconnect");

        ap04UiIsActive = false;
        ap04PageLoaded = false;
        ap04Dev = nullptr;

        // Dunker page teardown (object is reused via rebind(), so keep the pointer)
        dunkerUiIsActive = false;
        dunkerPageLoaded = false;

        master.disconnect();
        navPendingDisconnect = false;
    }

    // Deferred reconnect after node-id change
    if (ap04PendingReconnect && (int32_t)(millis() - ap04ReconnectDueMs) >= 0) {
        ap04PendingReconnect = false;

        const uint8_t newId = ap04ReconnectNewNodeId;
        Serial.printf("[APP] AP04 reconnect to new node-id: %u\n", (unsigned)newId);

        master.disconnect();
        master.connectNode(newId, KnownDeviceType::Unknown);

        ap04UiIsActive = false;
        ap04PageLoaded = false;
        ap04Dev = nullptr;

        selectedNodeId = newId;
    }

    // Master tick
    master.update();

    // Ping scan tick (fixed baud, no baud switching)
    if (pingScanRunning) {
        // Abort if user changed baud during scan
        if (activeBaud != pingScanBaudSnapshot) {
            Serial.println("[PING] Abort: baud changed during scan");
            pingScanRunning = false;
        } else {
            const uint32_t t = millis();
            if (pingScanNode <= 32 && (t - pingScanLastSendMs) >= 60) {
                // SDO read request to 0x1000:00 (device type)
                uint8_t sdo[8] = { 0x40, 0x00, 0x10, 0x00, 0,0,0,0 };
                const uint32_t cobId = 0x600 + pingScanNode;
                canDriver.sendFrame(cobId, sdo, 8);
                pingScanLastSendMs = t;
                pingScanNode++;
            }

            if (pingScanNode > 32) {
                // done (give a little time for late responses)
                static uint32_t doneDue = 0;
                if (doneDue == 0) doneDue = t + 250;
                if ((int32_t)(t - doneDue) >= 0) {
                    Serial.printf("[PING] Done. Found %u nodes: ", (unsigned)pingScanFoundCount);
                    for (uint8_t i = 1; i <= 32; i++) {
                        if (pingScanFound[i]) Serial.printf("%u ", (unsigned)i);
                    }
                    Serial.println();
                    pingScanRunning = false;
                    doneDue = 0;

                    // Show freshly discovered nodes in the start-menu list
                    lvgl_port_lock(-1);
                    refreshStartMenuList();
                    lvgl_port_unlock();
                }
            }
        }
    }

    // Dunker DS402 drive tick (statusword poll + controlword writes)
    if (dunkerUiIsActive && dunkerDev) {
        dunkerDev->update();
    }

    // Identify SDO client tick
    identifySdo.update();

    // Identify-All sequencer (non-blocking): for each found node read 1018h:01..04
    if (identifyAllRunning && !identifySdo.isBusy() && (int32_t)(millis() - identifyAllNextMs) >= 0) {
        identifySdo.setNodeId(identifyAllNode);

        switch (identifyAllStep) {
            case 0: {
                identifyAllStep = 1;
                // Clear any previous identity so a timed-out/aborted read can't
                // reuse stale values and misclassify (e.g. after a node swap).
                auto& dn0 = master.node(identifyAllNode);
                dn0.vendorId = dn0.productCode = dn0.revision = dn0.serial = 0;
                dn0.sdoOk = false;
                dn0.identifyInProgress = true;
                Serial.printf("[IDENT-ALL] Node %u: read 1018h:01 Vendor ID\n", (unsigned)identifyAllNode);
                identifySdo.readAsync(0x1018, 0x01, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) master.node(identifyAllNode).vendorId = v;
                });
                identifyAllNextMs = millis() + 20;
                break;
            }
            case 1:
                identifyAllStep = 2;
                Serial.printf("[IDENT-ALL] Node %u: read 1018h:02 Product Code\n", (unsigned)identifyAllNode);
                identifySdo.readAsync(0x1018, 0x02, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) master.node(identifyAllNode).productCode = v;
                });
                identifyAllNextMs = millis() + 20;
                break;
            case 2:
                identifyAllStep = 3;
                Serial.printf("[IDENT-ALL] Node %u: read 1018h:03 Revision\n", (unsigned)identifyAllNode);
                identifySdo.readAsync(0x1018, 0x03, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) master.node(identifyAllNode).revision = v;
                });
                identifyAllNextMs = millis() + 20;
                break;
            case 3:
                identifyAllStep = 4;
                Serial.printf("[IDENT-ALL] Node %u: read 1018h:04 Serial\n", (unsigned)identifyAllNode);
                identifySdo.readAsync(0x1018, 0x04, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) master.node(identifyAllNode).serial = v;
                });
                identifyAllNextMs = millis() + 20;
                break;
            default: {
                // Finalize this node: classify + log
                auto& dn = master.node(identifyAllNode);
                dn.sdoOk = (dn.vendorId != 0 || dn.productCode != 0);
                dn.known = classifyByIdentity(dn.vendorId, dn.productCode);
                dn.identifyInProgress = false;

                const char* typeStr =
                    (dn.known == KnownDeviceType::SIKO_AP04)   ? "SIKO AP04"   :
                    (dn.known == KnownDeviceType::SIKO_AP10)   ? "SIKO AP10"   :
                    (dn.known == KnownDeviceType::Dunker_75CI) ? "Dunker 75CI" : "Unknown";

                Serial.printf("[IDENT-ALL] Node %u: vendor=0x%08lX product=0x%08lX rev=0x%08lX serial=0x%08lX -> %s\n",
                              (unsigned)identifyAllNode,
                              (unsigned long)dn.vendorId,
                              (unsigned long)dn.productCode,
                              (unsigned long)dn.revision,
                              (unsigned long)dn.serial,
                              typeStr);

                // Dunker: print a HEURISTIC (unverified) BG-series decode next to the
                // raw product code, so the scheme can be validated on real hardware.
                if (dn.known == KnownDeviceType::Dunker_75CI) {
                    char hint[40];
                    dunkerDecodeHint(dn.productCode, hint, sizeof(hint));
                    Serial.printf("[IDENT-ALL] Node %u: Dunker BG-code hint (UNVERIFIED) = %s\n",
                                  (unsigned)identifyAllNode, hint);
                }

                // Reflect the new classification in the list immediately
                lvgl_port_lock(-1);
                refreshStartMenuList();
                lvgl_port_unlock();

                // Advance to the next found node, or finish
                uint8_t next = 0;
                for (uint8_t i = (uint8_t)(identifyAllNode + 1); i <= 32; i++) {
                    if (pingScanFound[i]) { next = i; break; }
                }
                if (next) {
                    identifyAllNode = next;
                    identifyAllStep = 0;
                    identifyAllNextMs = millis() + 30;
                } else {
                    identifyAllRunning = false;
                    Serial.println("[IDENT-ALL] Done (all found nodes identified)");
                }
                break;
            }
        }
    }

    // Standard node-detail read sequencer: 1018h identity + 1001h error register
    stdSdo.update();
    if (stdReadRunning && !stdSdo.isBusy() && (int32_t)(millis() - stdReadNextMs) >= 0) {
        switch (stdReadStep) {
            case 0: {
                stdReadStep = 1;
                // Clear previous identity so a failed read can't reuse stale values.
                auto& dns = master.node(stdReadNode);
                dns.vendorId = dns.productCode = dns.revision = dns.serial = 0;
                dns.sdoOk = false;
                stdSdo.readAsync(0x1018, 0x01, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) master.node(stdReadNode).vendorId = v;
                });
                stdReadNextMs = millis() + 20;
                break;
            }
            case 1:
                stdReadStep = 2;
                stdSdo.readAsync(0x1018, 0x02, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) master.node(stdReadNode).productCode = v;
                });
                stdReadNextMs = millis() + 20;
                break;
            case 2:
                stdReadStep = 3;
                stdSdo.readAsync(0x1018, 0x03, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) master.node(stdReadNode).revision = v;
                });
                stdReadNextMs = millis() + 20;
                break;
            case 3:
                stdReadStep = 4;
                stdSdo.readAsync(0x1018, 0x04, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) master.node(stdReadNode).serial = v;
                });
                stdReadNextMs = millis() + 20;
                break;
            case 4:
                stdReadStep = 5;
                stdSdo.readAsync(0x1001, 0x00, [](SdoResult r, uint32_t v){
                    if (r == SDO_OK) { stdErrReg = (uint8_t)(v & 0xFF); stdErrRegValid = true; }
                });
                stdReadNextMs = millis() + 20;
                break;
            default: {
                auto& dn = master.node(stdReadNode);
                dn.sdoOk = (dn.vendorId != 0 || dn.productCode != 0);
                dn.known = classifyByIdentity(dn.vendorId, dn.productCode);
                Serial.printf("[STD] Node %u: vendor=0x%08lX product=0x%08lX rev=0x%08lX ser=0x%08lX err1001=%s0x%02X\n",
                              (unsigned)stdReadNode,
                              (unsigned long)dn.vendorId, (unsigned long)dn.productCode,
                              (unsigned long)dn.revision, (unsigned long)dn.serial,
                              stdErrRegValid ? "" : "(invalid) ", (unsigned)stdErrReg);
                stdReadRunning = false;

                // Refresh the detail view (and list) with freshly read values
                lvgl_port_lock(-1);
                refreshNodeDetail();
                refreshStartMenuList();
                lvgl_port_unlock();
                break;
            }
        }
    }

    // Deferred UI navigation (LVGL)
    if (navPendingUiSwitch) {
        navPendingUiSwitch = false;
        lvgl_port_lock(-1);
        refreshStartMenuList();
        lv_scr_load_anim(startUi.screen(), LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
        if (navOpenNodeDetail && selectedNodeId >= 1 && selectedNodeId <= 127) {
            refreshNodeDetail();
            nodeUi.load();
        }
        lvgl_port_unlock();
    }

    // Prevent AP04 keep-alive while navigating
    static uint32_t lastAp04StartKickMs = 0;
    if (!navPendingDisconnect && !navPendingUiSwitch && !ap04PendingReconnect) {
        if (master.mode() == MasterMode::OnlineKnown && selectedNodeId >= 1 && selectedNodeId <= 127) {
            const auto& dn = master.node(selectedNodeId);
            if (dn.known == KnownDeviceType::SIKO_AP04) {
                const uint32_t nowMs = millis();
                if (nowMs - lastAp04StartKickMs > 1500) {
                    lastAp04StartKickMs = nowMs;
                    if (dn.lastHbState == 0x7F || dn.lastHbState == 0x00) {
                        Serial.println("[MASTER] AP04: Kick NMT START (keep OP)");
                        master.nmtStart();
                    }
                }
            }
        }
    }

    // Auto-switch to AP04 page (create once)
    if (!ap04PageLoaded && selectedNodeId >= 1 && selectedNodeId <= 127) {
        const auto& dn = master.node(selectedNodeId);
        if (!dn.identifyInProgress && dn.known == KnownDeviceType::SIKO_AP04) {
            ap04PageLoaded = true;
            ap04UiIsActive = true;

            if (!ap04Dev) {
                ap04Dev = new SikoAP04(&canDriver, selectedNodeId);
                ap04Dev->setScaleFactor(0.1f);
            }

            lvgl_port_lock(-1);

            ap04Ui.setAP04(ap04Dev);
            ap04Ui.setConfigCallback([](const AP10UI_Config& cfg) {
                // AP04: Set Node-ID from AP04 page
                if (cfg.toolsApply && cfg.toolsNewNodeId >= 1 && cfg.toolsNewNodeId <= 127) {
                    Serial.printf("[UI] AP04: Set Node-ID request -> %u\n", (unsigned)cfg.toolsNewNodeId);

                    master.ap04SetNodeIdRobust(cfg.toolsNewNodeId);

                    selectedNodeId = cfg.toolsNewNodeId;

                    ap04PendingReconnect = true;
                    ap04ReconnectNewNodeId = cfg.toolsNewNodeId;
                    ap04ReconnectDueMs = millis() + 1200;

                    return;
                }

                if (!cfg.navBackToMain && !cfg.navOpenNodeDetail) return;

                Serial.println("[UI] Home: request navigation");

                ap04UiIsActive = false;
                ap04PageLoaded = false;
                ap04Dev = nullptr;

                navOpenNodeDetail = cfg.navOpenNodeDetail;
                navPendingDisconnect = true;
                navPendingUiSwitch = true;
            });

            ap04Ui.create(selectedNodeId);
            master.nmtStart();

            lvgl_port_unlock();
        }
    }

    // Drive AP04 device update while page is active
    if (ap04UiIsActive && ap04Dev) {
        ap04Dev->update();
    }

    // Auto-switch to Dunker DS402 page (create once)
    if (!dunkerPageLoaded && selectedNodeId >= 1 && selectedNodeId <= 127) {
        const auto& dn = master.node(selectedNodeId);
        if (!dn.identifyInProgress && dn.known == KnownDeviceType::Dunker_75CI) {
            dunkerPageLoaded = true;

            // Reuse one heap object across connects (rebind -> no leak, no RX race)
            if (!dunkerDev) dunkerDev = new DunkerDrive(&canDriver, selectedNodeId);
            else            dunkerDev->rebind(selectedNodeId);

            lvgl_port_lock(-1);

            Dunker_Callbacks dcbs;
            dcbs.onBack = [](){
                Serial.println("[UI] Dunker: back to main");
                dunkerUiIsActive = false;   // stop RX processing BEFORE leaving
                dunkerPageLoaded = false;
                // Clear the selection so the auto-route block does not immediately
                // recreate the Dunker page (and leak LVGL screens) on the same pass.
                selectedNodeId = 0;
                navPendingDisconnect = true;
                navPendingUiSwitch = true;
            };
            dcbs.onCommand = [](DunkerUiCmd c){
                if (!dunkerDev) return;
                switch (c) {
                    case DunkerUiCmd::FaultReset:      dunkerDev->cmdFaultReset();      break;
                    case DunkerUiCmd::Shutdown:        dunkerDev->cmdShutdown();        break;
                    case DunkerUiCmd::SwitchOn:        dunkerDev->cmdSwitchOn();        break;
                    case DunkerUiCmd::EnableOperation: dunkerDev->cmdEnableOperation(); break;
                    case DunkerUiCmd::DisableVoltage:  dunkerDev->cmdDisableVoltage();  break;
                    case DunkerUiCmd::Halt:            dunkerDev->halt();               break;
                }
            };
            // M5: motion
            dcbs.onSetMode = [](int8_t mode){ if (dunkerDev) dunkerDev->setMode(mode); };
            dcbs.onGoto    = [](int32_t pos, uint32_t vel){ if (dunkerDev) dunkerDev->gotoPosition(pos, vel); };
            dcbs.onJog     = [](int dir, uint32_t speed){ if (dunkerDev) dunkerDev->jog(dir, speed); };
            // M6: I/O + brake
            dcbs.onSetOutput = [](uint8_t bit, bool on){ if (dunkerDev) dunkerDev->setOutputBit(bit, on); };
            dcbs.onBrake     = [](bool release){ if (dunkerDev) dunkerDev->setBrake(release); };
            dunkerUi.setCallbacks(dcbs);
            dunkerUi.create(selectedNodeId);
            dunkerUi.load();

            dunkerUiIsActive = true;   // activate RX processing only after full setup
            master.nmtStart();

            lvgl_port_unlock();
        }
    }

    // UI update
    if (now - lastUiUpdate >= Config::UI_UPDATE_MS) {
        lastUiUpdate = now;

        if (!navPendingDisconnect && !navPendingUiSwitch && !ap04PendingReconnect) {
            lvgl_port_lock(-1);
            refreshNodeDetail();
            if (ap04UiIsActive) refreshAp04();
            if (dunkerUiIsActive && dunkerDev) dunkerUi.update(dunkerDev->getData());
            lvgl_port_unlock();
        }
    }

    // Status print (+ sniffer health)
    if (now - lastStatusPrint >= Config::STATUS_PRINT_MS) {
        lastStatusPrint = now;

        Serial.printf("[STATUS] mode=%u baud=%lu scan=%s sniffer=%s drops=%lu age=%lu ms\n",
                      (unsigned)master.mode(),
                      (unsigned long)activeBaud,
                      scanRunning ? "RUN" : "STOP",
                      snifferEnabled ? "ON" : "OFF",
                      (unsigned long)sniffer.getDroppedCount(),
                      (unsigned long)sniffer.getLastTrafficAgeMs());
    }

    // Phase B UI integration will toggle snifferEnabled.

    delay(10);
}
