/**
 * @file ap10_ui_fixed.h
 * @brief LVGL UI fuer SIKO AP10 / AP04 Positionsanzeige
 *
 * FIXES:
 * [FIX #1] btnResetCommCb: LV_EVENT_VALUE_CHANGED – "Nein" loest keinen Reset aus
 * [FIX #2] btnZeroCb: LV_EVENT_VALUE_CHANGED – "Nein" loest kein Zero aus
 * [FIX #3] Beide Dialoge pruefen btnTxt != nullptr vor strcmp
 * [FIX #4] AP04 "0-Setzen" ruft setZeroLocal() – KEIN SDO-Write!
 *          Die SIKO-Anzeige zeigt weiterhin ihren Wert (korrekt).
 *          Der ESP32 setzt intern einen Software-Offset.
 * [FIX #5] Min/Max liest aus AP04Data.minMm/maxMm (immer mit aktuellem scaleFactor)
 * [FIX #6] Statusbyte-Bits (inPos, istLtSoll, batteryWarn, etc.) werden
 *          im Info-Panel angezeigt – mit farbigen Indikatoren
 */

#pragma once

#include <Arduino.h>
#include <lvgl.h>
#include "../devices/siko_ap10.h"
#include "../devices/siko_ap04.h"

// ============================================================================
// Farben
// ============================================================================
#define AP10UI_COLOR_BG          lv_color_hex(0x1A1A2E)
#define AP10UI_COLOR_CARD        lv_color_hex(0x16213E)
#define AP10UI_COLOR_ACCENT      lv_color_hex(0x0F3460)
#define AP10UI_COLOR_GREEN       lv_color_hex(0x00C851)
#define AP10UI_COLOR_RED         lv_color_hex(0xFF4444)
#define AP10UI_COLOR_YELLOW      lv_color_hex(0xFFBB33)
#define AP10UI_COLOR_WHITE       lv_color_hex(0xFFFFFF)
#define AP10UI_COLOR_GRAY        lv_color_hex(0x888888)
#define AP10UI_COLOR_BLUE_BTN    lv_color_hex(0x0066CC)
#define AP10UI_COLOR_ORANGE_BTN  lv_color_hex(0xCC6600)
#define AP10UI_COLOR_DARK_BTN    lv_color_hex(0x444444)
#define AP10UI_COLOR_GREEN_BTN   lv_color_hex(0x007744)
#define AP10UI_COLOR_CFG_BG      lv_color_hex(0x0D0D1A)
#define AP10UI_COLOR_CFG_CARD    lv_color_hex(0x12122A)
#define AP10UI_COLOR_DIM         lv_color_hex(0x333355)

// ============================================================================
// Konfigurations-Callback
// ============================================================================
struct AP10UI_Config {
    uint8_t  nodeId;
    uint32_t baudrate;
    float    scaleFactor;

    // [NEU] Tools/Scanner: erkannte Node neu konfigurieren
    bool     toolsApply = false;      ///< true = Tools-Seite hat eine Aktion angefordert
    uint8_t  toolsTargetNodeId = 0;   ///< erkannte/ausgewaehlte Node
    // Tools/Scanner: fuer spaetere Erweiterung (LSS / herstellerspezifisch)
    uint8_t  toolsNewNodeId = 0;      ///< neue Node-ID (falls unterstuetzt)
    uint32_t toolsNewBaudrate = 0;    ///< neue Baudrate (falls unterstuetzt)
     // Navigation / App-Callbacks
    bool navBackToMain = false;
    bool navOpenNodeDetail = false;

};
using AP10UI_ConfigCallback = std::function<void(const AP10UI_Config&)>;

// ============================================================================
// CAN-Statistiken
// ============================================================================
struct AP10UI_CanStats {
    uint32_t rxCount  = 0;
    uint32_t txCount  = 0;
    uint32_t errCount = 0;
    uint32_t recoveryCount = 0;
    uint32_t syncTx   = 0;
    uint32_t pdoCount = 0;
};

// ============================================================================
// AP10_UI Klasse
// ============================================================================
class AP10_UI {
public:
    // ---------------------------------------------------------------
    // [NEU] Node-Scanner Daten von aussen fuettern (Sketch/CAN-RX)
    // ---------------------------------------------------------------
    void toolsMarkNodeHeartbeat(uint8_t nodeId, uint8_t hbState) {
        if (nodeId < 1 || nodeId > 127) return;
        m_nodeSeen[nodeId] = true;
        m_nodeLastHbState[nodeId] = hbState;
        m_nodeLastSeenMs[nodeId] = millis();
    }

    AP10_UI() { s_instance = this; }

    void setAP10(SikoAP10* ap10) { m_ap10 = ap10; }
    void setAP04(SikoAP04* ap04) { m_ap04 = ap04; }
    void setConfigCallback(AP10UI_ConfigCallback cb) { m_configCallback = cb; }

    // -----------------------------------------------------------------------
    // UI aufbauen
    // -----------------------------------------------------------------------
    void create(uint8_t nodeId = 1) {
        m_nodeId = nodeId;

        // Tool-/Scanner-Status initialisieren
        for (int i = 0; i < 128; i++) {
            m_nodeSeen[i] = false;
            m_nodeLastHbState[i] = 0xFF;
            m_nodeLastSeenMs[i] = 0;
        }

        m_screen = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(m_screen, AP10UI_COLOR_BG, 0);
        lv_obj_set_style_bg_opa(m_screen, LV_OPA_COVER, 0);
        lv_obj_clear_flag(m_screen, LV_OBJ_FLAG_SCROLLABLE);

        buildHeader();
        buildPositionCard();
        buildInfoCard();
        buildStatusCard();   // [FIX #6] Statusbyte-Karte
        buildButtons();
        buildConfigScreen();
        buildToolsScreen();  // [NEU] CANopen Tools (Node-Scanner)

        lv_scr_load(m_screen);
        Serial.println("[UI] AP10_UI created");
    }

    // -----------------------------------------------------------------------
    // AP10 Hauptanzeige aktualisieren
    // -----------------------------------------------------------------------
    void update(const AP10Data& data) {
        if (!m_screen) return;

        if (!data.connected) {
            lv_obj_set_style_bg_color(m_statusDot, AP10UI_COLOR_RED, 0);
            lv_label_set_text(m_statusLabel, "Getrennt");
            lv_label_set_text(m_nmtLabel, "Warte auf Boot-Up...");
            lv_label_set_text(m_positionLabel, "---");
            lv_label_set_text(m_positionUnit, "");
        } else if (!data.operational) {
            lv_obj_set_style_bg_color(m_statusDot, AP10UI_COLOR_YELLOW, 0);
            lv_label_set_text(m_statusLabel, "Pre-Operational");
            lv_label_set_text(m_nmtLabel, "Initialisierung...");
            lv_label_set_text(m_positionLabel, "---");
            lv_label_set_text(m_positionUnit, "");
        } else {
            if (data.hasError) {
                lv_obj_set_style_bg_color(m_statusDot, AP10UI_COLOR_RED, 0);
                lv_label_set_text(m_statusLabel, "FEHLER");
            } else if (data.batteryLow) {
                lv_obj_set_style_bg_color(m_statusDot, AP10UI_COLOR_YELLOW, 0);
                lv_label_set_text(m_statusLabel, "Batterie schwach!");
            } else {
                lv_obj_set_style_bg_color(m_statusDot, AP10UI_COLOR_GREEN, 0);
                lv_label_set_text(m_statusLabel, "Operational");
            }
            lv_label_set_text(m_nmtLabel, data.inPosition ? "IN POSITION" : "");

            char buf[32];
            snprintf(buf, sizeof(buf), "%.1f", data.positionMm);
            lv_label_set_text(m_positionLabel, buf);
            lv_label_set_text(m_positionUnit, "mm");
        }

        // Min/Max/Hub
        if (data.positionMin != INT32_MAX && data.positionMax != INT32_MIN) {
            char buf[48];
            snprintf(buf, sizeof(buf), "Min: %ld", (long)data.positionMin);
            lv_label_set_text(m_minLabel, buf);
            snprintf(buf, sizeof(buf), "Max: %ld", (long)data.positionMax);
            lv_label_set_text(m_maxLabel, buf);
            const int32_t range = data.positionMax - data.positionMin;
            snprintf(buf, sizeof(buf), "Hub: %ld", (long)range);
            lv_label_set_text(m_rangeLabel, buf);
        }

        // Geschwindigkeit
        {
            char buf[48];
            snprintf(buf, sizeof(buf), "v: %.1f mm/s", data.velocityMmPerSec);
            lv_label_set_text(m_velocityLabel, buf);
        }

        // Update-Alter
        {
            char buf[48];
            if (data.lastUpdateMs == 0) {
                snprintf(buf, sizeof(buf), "Kein Signal");
            } else {
                const uint32_t age = millis() - data.lastUpdateMs;
                snprintf(buf, sizeof(buf), "Update: %lu ms", (unsigned long)age);
            }
            lv_label_set_text(m_heartbeatLabel, buf);
        }

        // PDO-Zaehler
        {
            char buf[48];
            snprintf(buf, sizeof(buf), "PDOs: %lu", (unsigned long)data.updateCount);
            lv_label_set_text(m_updateCountLabel, buf);
        }

        // Seriennummer
        if (data.deviceInfoRead) {
            char buf[48];
            snprintf(buf, sizeof(buf), "S/N: %lu", (unsigned long)data.serialNumber);
            lv_label_set_text(m_serialLabel, buf);
        }
    }

    // -----------------------------------------------------------------------
    // [FIX #5 + #6] AP04 Anzeige aktualisieren
    // -----------------------------------------------------------------------
    void updateAP04(const AP04Data& d) {
        if (!m_screen) return;

        // [NEU] Tools-Screen live aktualisieren (Node-Scanner)
        toolsUpdateUi();

        // --- Verbindungsstatus ---
        if (!d.connected) {
            lv_obj_set_style_bg_color(m_statusDot, AP10UI_COLOR_RED, 0);
            lv_label_set_text(m_statusLabel, "Getrennt");
            lv_label_set_text(m_nmtLabel, "Warte auf Boot-Up...");
            lv_label_set_text(m_positionLabel, "---");
            lv_label_set_text(m_positionUnit, "");
        } else {
            lv_obj_set_style_bg_color(m_statusDot,
                d.batteryEmpty ? AP10UI_COLOR_RED :
                d.batteryWarn  ? AP10UI_COLOR_YELLOW :
                                 AP10UI_COLOR_GREEN, 0);
            lv_label_set_text(m_statusLabel,
                d.batteryEmpty ? "Batterie LEER!" :
                d.batteryWarn  ? "Batterie Warnung" :
                                 "Operational");
            lv_label_set_text(m_nmtLabel, d.inPos ? "IN POSITION" : "");

            char buf[32];
            snprintf(buf, sizeof(buf), "%.1f", d.positionMm);
            lv_label_set_text(m_positionLabel, buf);
            lv_label_set_text(m_positionUnit, "mm");
        }

        // --- [FIX #5] Min/Max – direkt aus AP04Data (immer mit aktuellem scaleFactor) ---
        if (d.hasMinMax) {
            char buf[48];
            snprintf(buf, sizeof(buf), "Min: %.2f mm", d.minMm);
            lv_label_set_text(m_minLabel, buf);
            snprintf(buf, sizeof(buf), "Max: %.2f mm", d.maxMm);
            lv_label_set_text(m_maxLabel, buf);
            snprintf(buf, sizeof(buf), "Hub: %.2f mm", d.maxMm - d.minMm);
            lv_label_set_text(m_rangeLabel, buf);
        } else {
            lv_label_set_text(m_minLabel, "Min: ---");
            lv_label_set_text(m_maxLabel, "Max: ---");
            lv_label_set_text(m_rangeLabel, "Hub: ---");
        }

        // PDO-Zaehler
        {
            char buf[48];
            snprintf(buf, sizeof(buf), "PDOs: %lu", (unsigned long)d.updateCount);
            lv_label_set_text(m_updateCountLabel, buf);
        }

        // Status-Byte Rohwert
        {
            char buf[48];
            snprintf(buf, sizeof(buf), "Status: 0x%02X", (unsigned)d.statusByte);
            lv_label_set_text(m_heartbeatLabel, buf);
        }

        // Software-Offset anzeigen
        {
            char buf[48];
            snprintf(buf, sizeof(buf), "Offset: %ld", (long)d.zeroOffset);
            lv_label_set_text(m_serialLabel, buf);
        }

        // [FIX #6] Statusbyte-Bits als farbige Indikatoren aktualisieren
        updateStatusBits(d);
    }

    // -----------------------------------------------------------------------
    // CAN-Statistiken aktualisieren (Konfig-Screen)
    // -----------------------------------------------------------------------
    void updateCanStats(const AP10UI_CanStats& stats) {
        if (!m_cfgScreen || !m_cfgLblRx) return;

        char buf[64];
        snprintf(buf, sizeof(buf), "CAN RX:    %lu", (unsigned long)stats.rxCount);
        lv_label_set_text(m_cfgLblRx, buf);
        snprintf(buf, sizeof(buf), "CAN TX:    %lu", (unsigned long)stats.txCount);
        lv_label_set_text(m_cfgLblTx, buf);
        snprintf(buf, sizeof(buf), "CAN Err:   %lu", (unsigned long)stats.errCount);
        lv_label_set_text(m_cfgLblErr, buf);
        snprintf(buf, sizeof(buf), "Recovery:  %lu", (unsigned long)stats.recoveryCount);
        lv_label_set_text(m_cfgLblRecovery, buf);
        snprintf(buf, sizeof(buf), "SYNC TX:   %lu", (unsigned long)stats.syncTx);
        lv_label_set_text(m_cfgLblSync, buf);
        snprintf(buf, sizeof(buf), "PDO-Upd:   %lu", (unsigned long)stats.pdoCount);
        lv_label_set_text(m_cfgLblPdo, buf);
    }

private:
    // -----------------------------------------------------------------------
    // LVGL Objekte – Haupt-Screen
    // -----------------------------------------------------------------------
    lv_obj_t* m_screen           = nullptr;
    uint8_t   m_nodeId           = 1;

    // -----------------------------------------------------------------------
    // [NEU] Node-Scanner Daten (1..127)
    // -----------------------------------------------------------------------
    bool     m_nodeSeen[128];
    uint8_t  m_nodeLastHbState[128];
    uint32_t m_nodeLastSeenMs[128];

    uint8_t  m_toolsSelectedNode = 1;

    lv_obj_t* m_toolsScreen      = nullptr;
    lv_obj_t* m_toolsDropNode    = nullptr;
    lv_obj_t* m_toolsLblState    = nullptr;
    lv_obj_t* m_toolsLblLastSeen = nullptr;
    lv_obj_t* m_toolsDropNewNode = nullptr;
    lv_obj_t* m_toolsDropNewBaud = nullptr;

    lv_obj_t* m_positionLabel    = nullptr;
    lv_obj_t* m_positionUnit     = nullptr;
    lv_obj_t* m_statusDot        = nullptr;
    lv_obj_t* m_statusLabel      = nullptr;
    lv_obj_t* m_nmtLabel         = nullptr;

    lv_obj_t* m_minLabel         = nullptr;
    lv_obj_t* m_maxLabel         = nullptr;
    lv_obj_t* m_rangeLabel       = nullptr;
    lv_obj_t* m_velocityLabel    = nullptr;
    lv_obj_t* m_heartbeatLabel   = nullptr;
    lv_obj_t* m_updateCountLabel = nullptr;
    lv_obj_t* m_serialLabel      = nullptr;
    lv_obj_t* m_nodeIdLabel      = nullptr;

    lv_obj_t* m_btnZero          = nullptr;
    lv_obj_t* m_btnResetMinMax   = nullptr;
    lv_obj_t* m_btnResetComm     = nullptr;
    // [NEW] AP04: Set Node-ID Dialog + Button
    lv_obj_t* m_btnSetNodeId     = nullptr;
    lv_obj_t* m_mboxSetNodeId    = nullptr;
    lv_obj_t* m_taSetNodeId      = nullptr;
    lv_obj_t* m_kbSetNodeId      = nullptr;

    // [FIX #6] Statusbyte-Indikatoren
    lv_obj_t* m_dotInPos         = nullptr;
    lv_obj_t* m_dotIstLtSoll     = nullptr;
    lv_obj_t* m_dotBattWarn      = nullptr;
    lv_obj_t* m_dotBattEmpty     = nullptr;
    lv_obj_t* m_dotChainSet      = nullptr;
    lv_obj_t* m_dotArrowGt       = nullptr;
    lv_obj_t* m_dotArrowLt       = nullptr;

    // -----------------------------------------------------------------------
    // LVGL Objekte – Konfigurations-Screen
    // -----------------------------------------------------------------------
    lv_obj_t* m_cfgScreen        = nullptr;
    lv_obj_t* m_cfgRollerNodeId  = nullptr;
    lv_obj_t* m_cfgDropBaud      = nullptr;
    lv_obj_t* m_cfgSpinScale     = nullptr;
    lv_obj_t* m_cfgLblRx         = nullptr;
    lv_obj_t* m_cfgLblTx         = nullptr;
    lv_obj_t* m_cfgLblErr        = nullptr;
    lv_obj_t* m_cfgLblRecovery   = nullptr;
    lv_obj_t* m_cfgLblSync       = nullptr;
    lv_obj_t* m_cfgLblPdo        = nullptr;

    // Geraete-Referenzen & Callbacks
    SikoAP10*             m_ap10           = nullptr;
    SikoAP04*             m_ap04           = nullptr;
    AP10UI_ConfigCallback m_configCallback = nullptr;

    inline static AP10_UI* s_instance = nullptr;

    // -----------------------------------------------------------------------
    // Hilfsfunktionen
    // -----------------------------------------------------------------------
    static lv_obj_t* makeLabel(lv_obj_t* parent, const char* text,
                                lv_color_t color, const lv_font_t* font,
                                int x, int y)
    {
        lv_obj_t* lbl = lv_label_create(parent);
        lv_label_set_text(lbl, text);
        lv_obj_set_style_text_color(lbl, color, 0);
        lv_obj_set_style_text_font(lbl, font, 0);
        lv_obj_set_pos(lbl, x, y);
        return lbl;
    }

    // Kleiner runder Indikator-Punkt
    static lv_obj_t* makeDot(lv_obj_t* parent, lv_color_t color, int x, int y) {
        lv_obj_t* dot = lv_obj_create(parent);
        lv_obj_set_size(dot, 14, 14);
        lv_obj_set_pos(dot, x, y);
        lv_obj_set_style_bg_color(dot, color, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_set_style_radius(dot, 7, 0);
        return dot;
    }

    // Dot-Farbe setzen: aktiv=Farbe, inaktiv=dunkel
    static void setDot(lv_obj_t* dot, bool active, lv_color_t activeColor) {
        if (!dot) return;
        lv_obj_set_style_bg_color(dot, active ? activeColor : AP10UI_COLOR_DIM, 0);
    }

    // -----------------------------------------------------------------------
    // [FIX #6] Statusbyte-Bits aktualisieren
    // -----------------------------------------------------------------------
    void updateStatusBits(const AP04Data& d) {
        setDot(m_dotInPos,     d.inPos,       AP10UI_COLOR_GREEN);
        setDot(m_dotIstLtSoll, d.istLtSoll,   AP10UI_COLOR_YELLOW);
        setDot(m_dotBattWarn,  d.batteryWarn,  AP10UI_COLOR_YELLOW);
        setDot(m_dotBattEmpty, d.batteryEmpty, AP10UI_COLOR_RED);
        setDot(m_dotChainSet,  d.chainSet,     AP10UI_COLOR_BLUE_BTN);
        setDot(m_dotArrowGt,   d.arrowGt,      AP10UI_COLOR_WHITE);
        setDot(m_dotArrowLt,   d.arrowLt,      AP10UI_COLOR_WHITE);
    }

    // -----------------------------------------------------------------------
    // Header
    // -----------------------------------------------------------------------
    void buildHeader() {
        lv_obj_t* header = lv_obj_create(m_screen);
        lv_obj_set_size(header, 800, 60);
        lv_obj_set_pos(header, 0, 0);
        lv_obj_set_style_bg_color(header, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(header, 0, 0);
        lv_obj_set_style_radius(header, 0, 0);
        lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* title = lv_label_create(header);
        lv_label_set_text(title, "SIKO AP04 - Absolute Positionsanzeige");
        lv_obj_set_style_text_color(title, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_LEFT_MID, 15, 0);

        m_nodeIdLabel = lv_label_create(header);
        char buf[64];
        snprintf(buf, sizeof(buf), "Node-ID: %u  |  250 kBit/s", (unsigned)m_nodeId);
        lv_label_set_text(m_nodeIdLabel, buf);
        lv_obj_set_style_text_color(m_nodeIdLabel, AP10UI_COLOR_GRAY, 0);
        lv_obj_set_style_text_font(m_nodeIdLabel, &lv_font_montserrat_14, 0);
        lv_obj_align(m_nodeIdLabel, LV_ALIGN_RIGHT_MID, -125, 0);

        // [NEU] Tools-Button (Lupe) -> CANopen Tools / Node-Scanner
        lv_obj_t* btnTools = lv_btn_create(header);
        lv_obj_set_size(btnTools, 50, 40);
        lv_obj_align(btnTools, LV_ALIGN_RIGHT_MID, -65, 0);
        lv_obj_set_style_bg_color(btnTools, AP10UI_COLOR_DARK_BTN, 0);
        lv_obj_set_style_radius(btnTools, 8, 0);
        lv_obj_add_event_cb(btnTools, btnToolsCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* toolsIcon = lv_label_create(btnTools);
        lv_label_set_text(toolsIcon, LV_SYMBOL_SETTINGS);
        lv_obj_set_style_text_color(toolsIcon, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(toolsIcon, &lv_font_montserrat_20, 0);
        lv_obj_center(toolsIcon);

        // Zahnrad-Button -> Konfigurations-Screen
        lv_obj_t* btnCfg = lv_btn_create(header);
        lv_obj_set_size(btnCfg, 50, 40);
        lv_obj_align(btnCfg, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(btnCfg, AP10UI_COLOR_DARK_BTN, 0);
        lv_obj_set_style_radius(btnCfg, 8, 0);
        lv_obj_add_event_cb(btnCfg, btnConfigCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* cfgIcon = lv_label_create(btnCfg);
        lv_label_set_text(cfgIcon, LV_SYMBOL_SETTINGS);
        lv_obj_set_style_text_color(cfgIcon, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(cfgIcon, &lv_font_montserrat_20, 0);
        lv_obj_center(cfgIcon);
    }

    // -----------------------------------------------------------------------
    // Positions-Karte (links oben)
    // -----------------------------------------------------------------------
    void buildPositionCard() {
        lv_obj_t* card = lv_obj_create(m_screen);
        lv_obj_set_size(card, 390, 300);
        lv_obj_set_pos(card, 10, 70);
        lv_obj_set_style_bg_color(card, AP10UI_COLOR_CARD, 0);
        lv_obj_set_style_border_color(card, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        m_statusDot = lv_obj_create(card);
        lv_obj_set_size(m_statusDot, 20, 20);
        lv_obj_set_pos(m_statusDot, 10, 10);
        lv_obj_set_style_bg_color(m_statusDot, AP10UI_COLOR_RED, 0);
        lv_obj_set_style_border_width(m_statusDot, 0, 0);
        lv_obj_set_style_radius(m_statusDot, 10, 0);

        m_statusLabel = lv_label_create(card);
        lv_label_set_text(m_statusLabel, "Getrennt");
        lv_obj_set_style_text_color(m_statusLabel, AP10UI_COLOR_GRAY, 0);
        lv_obj_set_style_text_font(m_statusLabel, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(m_statusLabel, 38, 12);

        m_nmtLabel = lv_label_create(card);
        lv_label_set_text(m_nmtLabel, "Warte auf Boot-Up...");
        lv_obj_set_style_text_color(m_nmtLabel, AP10UI_COLOR_YELLOW, 0);
        lv_obj_set_style_text_font(m_nmtLabel, &lv_font_montserrat_14, 0);
        lv_obj_align(m_nmtLabel, LV_ALIGN_TOP_MID, 0, 10);

        lv_obj_t* posTitle = lv_label_create(card);
        lv_label_set_text(posTitle, "IST-Position (ESP32-relativ)");
        lv_obj_set_style_text_color(posTitle, AP10UI_COLOR_GRAY, 0);
        lv_obj_set_style_text_font(posTitle, &lv_font_montserrat_14, 0);
        lv_obj_align(posTitle, LV_ALIGN_TOP_MID, 0, 50);

        m_positionLabel = lv_label_create(card);
        lv_label_set_text(m_positionLabel, "---");
        lv_obj_set_style_text_color(m_positionLabel, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(m_positionLabel, &lv_font_montserrat_48, 0);
        lv_obj_align(m_positionLabel, LV_ALIGN_CENTER, 0, 10);

        m_positionUnit = lv_label_create(card);
        lv_label_set_text(m_positionUnit, "");
        lv_obj_set_style_text_color(m_positionUnit, AP10UI_COLOR_GRAY, 0);
        lv_obj_set_style_text_font(m_positionUnit, &lv_font_montserrat_16, 0);
        lv_obj_align(m_positionUnit, LV_ALIGN_BOTTOM_MID, 0, -20);
    }

    // -----------------------------------------------------------------------
    // Info-Karte (rechts oben): Min/Max/Hub/PDO/Offset
    // -----------------------------------------------------------------------
    void buildInfoCard() {
        lv_obj_t* card = lv_obj_create(m_screen);
        lv_obj_set_size(card, 200, 300);
        lv_obj_set_pos(card, 410, 70);
        lv_obj_set_style_bg_color(card, AP10UI_COLOR_CARD, 0);
        lv_obj_set_style_border_color(card, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* t = lv_label_create(card);
        lv_label_set_text(t, "Messwerte");
        lv_obj_set_style_text_color(t, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 5);

        m_minLabel = makeLabel(card, "Min: ---",
                               AP10UI_COLOR_GREEN, &lv_font_montserrat_14, 8, 35);
        m_maxLabel = makeLabel(card, "Max: ---",
                               AP10UI_COLOR_RED, &lv_font_montserrat_14, 8, 60);
        m_rangeLabel = makeLabel(card, "Hub: ---",
                                 AP10UI_COLOR_YELLOW, &lv_font_montserrat_14, 8, 85);

        // Trennlinie
        lv_obj_t* sep = lv_obj_create(card);
        lv_obj_set_size(sep, 180, 1);
        lv_obj_set_pos(sep, 5, 112);
        lv_obj_set_style_bg_color(sep, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(sep, 0, 0);

        m_velocityLabel = makeLabel(card, "v: 0.0 mm/s",
                                    AP10UI_COLOR_WHITE, &lv_font_montserrat_14, 8, 120);
        m_heartbeatLabel = makeLabel(card, "Status: 0x00",
                                     AP10UI_COLOR_GRAY, &lv_font_montserrat_12, 8, 148);
        m_updateCountLabel = makeLabel(card, "PDOs: 0",
                                       AP10UI_COLOR_GRAY, &lv_font_montserrat_12, 8, 168);
        m_serialLabel = makeLabel(card, "Offset: 0",
                                  AP10UI_COLOR_GRAY, &lv_font_montserrat_12, 8, 188);
    }

    // -----------------------------------------------------------------------
    // [FIX #6] Statusbyte-Karte (ganz rechts)
    // -----------------------------------------------------------------------
    void buildStatusCard() {
        lv_obj_t* card = lv_obj_create(m_screen);
        lv_obj_set_size(card, 175, 300);
        lv_obj_set_pos(card, 618, 70);
        lv_obj_set_style_bg_color(card, AP10UI_COLOR_CARD, 0);
        lv_obj_set_style_border_color(card, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* t = lv_label_create(card);
        lv_label_set_text(t, "AP04 Status");
        lv_obj_set_style_text_color(t, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(t, &lv_font_montserrat_14, 0);
        lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 5);

        // Jede Zeile: Dot + Label
        // y-Positionen: 35, 65, 95, 125, 155, 185, 215
        struct BitRow { const char* name; lv_obj_t** dot; lv_color_t color; int y; };
        // Wir bauen die Zeilen manuell:

        int y = 35;
        const int dy = 30;

        // IN-POS
        m_dotInPos = makeDot(card, AP10UI_COLOR_DIM, 8, y);
        makeLabel(card, "IN-POS",    AP10UI_COLOR_WHITE, &lv_font_montserrat_14, 30, y);
        y += dy;

        // IST < SOLL
        m_dotIstLtSoll = makeDot(card, AP10UI_COLOR_DIM, 8, y);
        makeLabel(card, "IST<SOLL",  AP10UI_COLOR_WHITE, &lv_font_montserrat_14, 30, y);
        y += dy;

        // Batt. Warn
        m_dotBattWarn = makeDot(card, AP10UI_COLOR_DIM, 8, y);
        makeLabel(card, "Batt.Warn", AP10UI_COLOR_WHITE, &lv_font_montserrat_14, 30, y);
        y += dy;

        // Batt. Leer
        m_dotBattEmpty = makeDot(card, AP10UI_COLOR_DIM, 8, y);
        makeLabel(card, "Batt.Leer", AP10UI_COLOR_WHITE, &lv_font_montserrat_14, 30, y);
        y += dy;

        // Kettenmass
        m_dotChainSet = makeDot(card, AP10UI_COLOR_DIM, 8, y);
        makeLabel(card, "Kette",     AP10UI_COLOR_WHITE, &lv_font_montserrat_14, 30, y);
        y += dy;

        // Pfeil >
        m_dotArrowGt = makeDot(card, AP10UI_COLOR_DIM, 8, y);
        makeLabel(card, "> LED",     AP10UI_COLOR_WHITE, &lv_font_montserrat_14, 30, y);
        y += dy;

        // Pfeil <
        m_dotArrowLt = makeDot(card, AP10UI_COLOR_DIM, 8, y);
        makeLabel(card, "< LED",     AP10UI_COLOR_WHITE, &lv_font_montserrat_14, 30, y);
    }

    // -----------------------------------------------------------------------
    // Buttons (unten)
    // -----------------------------------------------------------------------
    void buildButtons() {
        // 0-Setzen
        m_btnZero = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnZero, 230, 70);
        lv_obj_set_pos(m_btnZero, 10, 385);
        lv_obj_set_style_bg_color(m_btnZero, AP10UI_COLOR_BLUE_BTN, 0);
        lv_obj_set_style_radius(m_btnZero, 10, 0);
        lv_obj_add_event_cb(m_btnZero, btnZeroCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* lbl = lv_label_create(m_btnZero);
        lv_label_set_text(lbl, LV_SYMBOL_REFRESH "  0-Setzen");
        lv_obj_set_style_text_color(lbl, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl);

        // Min/Max Reset
        m_btnResetMinMax = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnResetMinMax, 230, 70);
        lv_obj_set_pos(m_btnResetMinMax, 255, 385);
        lv_obj_set_style_bg_color(m_btnResetMinMax, AP10UI_COLOR_ORANGE_BTN, 0);
        lv_obj_set_style_radius(m_btnResetMinMax, 10, 0);
        lv_obj_add_event_cb(m_btnResetMinMax, btnResetMinMaxCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* lbl2 = lv_label_create(m_btnResetMinMax);
        lv_label_set_text(lbl2, LV_SYMBOL_TRASH "  Min/Max Reset");
        lv_obj_set_style_text_color(lbl2, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(lbl2, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl2);

        // Reset-Com
        m_btnResetComm = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnResetComm, 295, 70);
        lv_obj_set_pos(m_btnResetComm, 498, 385);
        lv_obj_set_style_bg_color(m_btnResetComm, AP10UI_COLOR_DARK_BTN, 0);
        lv_obj_set_style_radius(m_btnResetComm, 10, 0);
        lv_obj_add_event_cb(m_btnResetComm, btnResetCommCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* lbl3 = lv_label_create(m_btnResetComm);
        lv_label_set_text(lbl3, LV_SYMBOL_WARNING "  Reset-Com");
        lv_obj_set_style_text_color(lbl3, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(lbl3, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl3);

    // [NEW] AP04: Set Node-ID (nur wenn AP04 aktiv)
    if (m_ap04) {
        m_btnSetNodeId = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnSetNodeId, 230, 70);
        // Position: oberhalb der Button-Reihe, damit nichts überlappt
        lv_obj_set_pos(m_btnSetNodeId, 10, 305);
        lv_obj_set_style_bg_color(m_btnSetNodeId, AP10UI_COLOR_GREEN_BTN, 0);
        lv_obj_set_style_radius(m_btnSetNodeId, 10, 0);
        lv_obj_add_event_cb(m_btnSetNodeId, btnSetNodeIdCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* lbl4 = lv_label_create(m_btnSetNodeId);
        lv_label_set_text(lbl4, LV_SYMBOL_EDIT "  Set Node-ID");
        lv_obj_set_style_text_color(lbl4, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(lbl4, &lv_font_montserrat_18, 0);
        lv_obj_center(lbl4);
    }

        // Footer
        lv_obj_t* info = lv_label_create(m_screen);
        char infoBuf[96];
        snprintf(infoBuf, sizeof(infoBuf),
                 "SIKO AP04 | CANopen | Node-ID %u | 250 kBit/s | 0-Setzen = Software-Offset",
                 (unsigned)m_nodeId);
        lv_label_set_text(info, infoBuf);
        lv_obj_set_style_text_color(info, AP10UI_COLOR_GRAY, 0);
        lv_obj_set_style_text_font(info, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(info, 10, 462);
    }

    // -----------------------------------------------------------------------
    // Konfigurations-Screen
    // -----------------------------------------------------------------------
    void buildToolsScreen() {
        // ================================================================
        // CANopen Tools / Node-Scanner (NMT)
        // ================================================================
        m_toolsScreen = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(m_toolsScreen, AP10UI_COLOR_CFG_BG, 0);
        lv_obj_set_style_bg_opa(m_toolsScreen, LV_OPA_COVER, 0);
        lv_obj_clear_flag(m_toolsScreen, LV_OBJ_FLAG_SCROLLABLE);

        // Header
        lv_obj_t* hdr = lv_obj_create(m_toolsScreen);
        lv_obj_set_size(hdr, 800, 60);
        lv_obj_set_pos(hdr, 0, 0);
        lv_obj_set_style_bg_color(hdr, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(hdr, 0, 0);
        lv_obj_set_style_radius(hdr, 0, 0);
        lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* hdrTitle = lv_label_create(hdr);
        lv_label_set_text(hdrTitle, LV_SYMBOL_SETTINGS "  CANopen Tools (NMT)");
        lv_obj_set_style_text_color(hdrTitle, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(hdrTitle, &lv_font_montserrat_20, 0);
        lv_obj_align(hdrTitle, LV_ALIGN_LEFT_MID, 15, 0);

        lv_obj_t* btnBack = lv_btn_create(hdr);
        lv_obj_set_size(btnBack, 110, 40);
        lv_obj_align(btnBack, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(btnBack, AP10UI_COLOR_DARK_BTN, 0);
        lv_obj_set_style_radius(btnBack, 8, 0);
        lv_obj_add_event_cb(btnBack, btnToolsBackCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* backLbl = lv_label_create(btnBack);
        lv_label_set_text(backLbl, LV_SYMBOL_LEFT "  Zurueck");
        lv_obj_set_style_text_color(backLbl, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(backLbl, &lv_font_montserrat_16, 0);
        lv_obj_center(backLbl);

        // Hinweis
        lv_obj_t* warn = lv_label_create(m_toolsScreen);
        lv_label_set_text(warn,
            "Node-Scanner + NMT Tools.\n"
            "Hinweis: Node-ID/Baudrate sind nicht per NMT aenderbar (dafuer braucht man i.d.R. LSS).\n"
            "AP04 laut Handbuch: kein LSS.");
        lv_obj_set_style_text_color(warn, AP10UI_COLOR_YELLOW, 0);
        lv_obj_set_style_text_font(warn, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(warn, 10, 70);

        // Karte links: Node Auswahl + Status
        lv_obj_t* cardLeft = lv_obj_create(m_toolsScreen);
        lv_obj_set_size(cardLeft, 380, 360);
        lv_obj_set_pos(cardLeft, 10, 120);
        lv_obj_set_style_bg_color(cardLeft, AP10UI_COLOR_CFG_CARD, 0);
        lv_obj_set_style_border_color(cardLeft, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(cardLeft, 2, 0);
        lv_obj_set_style_radius(cardLeft, 12, 0);
        lv_obj_clear_flag(cardLeft, LV_OBJ_FLAG_SCROLLABLE);

        makeLabel(cardLeft, "Erkannte Node:", AP10UI_COLOR_GRAY, &lv_font_montserrat_16, 10, 20);

        // Dropdown Node 1..127
        static char nodeOpts[1024];
        nodeOpts[0] = '\0';
        for (int i = 1; i <= 127; i++) {
            char tmp[8];
            snprintf(tmp, sizeof(tmp), "%d", i);
            strncat(nodeOpts, tmp, sizeof(nodeOpts) - strlen(nodeOpts) - 1);
            if (i < 127) strncat(nodeOpts, "\n", sizeof(nodeOpts) - strlen(nodeOpts) - 1);
        }

        m_toolsDropNode = lv_dropdown_create(cardLeft);
        lv_dropdown_set_options(m_toolsDropNode, nodeOpts);
        lv_dropdown_set_selected(m_toolsDropNode, 0);
        lv_obj_set_size(m_toolsDropNode, 180, 44);
        lv_obj_set_pos(m_toolsDropNode, 180, 12);
        lv_obj_set_style_bg_color(m_toolsDropNode, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_text_color(m_toolsDropNode, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(m_toolsDropNode, &lv_font_montserrat_16, 0);

        m_toolsLblState = makeLabel(cardLeft, "State: ---", AP10UI_COLOR_WHITE, &lv_font_montserrat_16, 10, 80);
        m_toolsLblLastSeen = makeLabel(cardLeft, "Last seen: ---", AP10UI_COLOR_WHITE, &lv_font_montserrat_16, 10, 110);

        // Karte rechts: Neue Parameter
        lv_obj_t* cardRight = lv_obj_create(m_toolsScreen);
        lv_obj_set_size(cardRight, 400, 360);
        lv_obj_set_pos(cardRight, 400, 120);
        lv_obj_set_style_bg_color(cardRight, AP10UI_COLOR_CFG_CARD, 0);
        lv_obj_set_style_border_color(cardRight, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(cardRight, 2, 0);
        lv_obj_set_style_radius(cardRight, 12, 0);
        lv_obj_clear_flag(cardRight, LV_OBJ_FLAG_SCROLLABLE);

        makeLabel(cardRight, "Neue Node-ID:", AP10UI_COLOR_GRAY, &lv_font_montserrat_16, 10, 20);
        m_toolsDropNewNode = lv_dropdown_create(cardRight);
        lv_dropdown_set_options(m_toolsDropNewNode, nodeOpts);
        lv_dropdown_set_selected(m_toolsDropNewNode, 0);
        lv_obj_set_size(m_toolsDropNewNode, 180, 44);
        lv_obj_set_pos(m_toolsDropNewNode, 200, 12);
        lv_obj_set_style_bg_color(m_toolsDropNewNode, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_text_color(m_toolsDropNewNode, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(m_toolsDropNewNode, &lv_font_montserrat_16, 0);

        makeLabel(cardRight, "Neue Baudrate:", AP10UI_COLOR_GRAY, &lv_font_montserrat_16, 10, 80);
        m_toolsDropNewBaud = lv_dropdown_create(cardRight);
        lv_dropdown_set_options(m_toolsDropNewBaud,
            "125 kBit/s\n"
            "250 kBit/s\n"
            "500 kBit/s\n"
            "1 MBit/s\n"
            "100 kBit/s\n"
            "50 kBit/s\n"
            "20 kBit/s\n"
            "10 kBit/s");
        lv_dropdown_set_selected(m_toolsDropNewBaud, 1);
        lv_obj_set_size(m_toolsDropNewBaud, 180, 44);
        lv_obj_set_pos(m_toolsDropNewBaud, 200, 72);
        lv_obj_set_style_bg_color(m_toolsDropNewBaud, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_text_color(m_toolsDropNewBaud, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(m_toolsDropNewBaud, &lv_font_montserrat_16, 0);

        lv_obj_t* btnApply = lv_btn_create(cardRight);
        lv_obj_set_size(btnApply, 370, 60);
        lv_obj_set_pos(btnApply, 15, 150);
        lv_obj_set_style_bg_color(btnApply, AP10UI_COLOR_GREEN_BTN, 0);
        lv_obj_set_style_radius(btnApply, 10, 0);
        lv_obj_add_event_cb(btnApply, btnToolsApplyCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* applyLbl = lv_label_create(btnApply);
        lv_label_set_text(applyLbl, LV_SYMBOL_OK "  Hinweis / Aktionen");
        lv_obj_set_style_text_color(applyLbl, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(applyLbl, &lv_font_montserrat_18, 0);
        lv_obj_center(applyLbl);

        lv_obj_t* hint = lv_label_create(cardRight);
        lv_label_set_text(hint,
            "Node-ID/Baudrate: nicht per NMT.\n"
            "Fuer universell: spaeter LSS/Fastscan (wenn Geraet es kann)\n"
            "oder herstellerspezifische SDO-Parameter.");
        lv_obj_set_style_text_color(hint, AP10UI_COLOR_GRAY, 0);
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(hint, 15, 230);
    }

    // -----------------------------------------------------------------------
    // [NEU] Tools helper: UI aktualisieren (Status/LastSeen)
    // -----------------------------------------------------------------------
    void toolsUpdateUi() {
        if (!m_toolsScreen || !m_toolsDropNode || !m_toolsLblState || !m_toolsLblLastSeen) return;

        const uint16_t idx = lv_dropdown_get_selected(m_toolsDropNode);
        const uint8_t nid = (uint8_t)(idx + 1);
        m_toolsSelectedNode = nid;

        const uint8_t st = m_nodeLastHbState[nid];
        const uint32_t lastMs = m_nodeLastSeenMs[nid];

        const char* stTxt = "---";
        if (!m_nodeSeen[nid]) {
            stTxt = "nicht gesehen";
        } else {
            if (st == 0x00) stTxt = "Boot-Up";
            else if (st == 0x7F) stTxt = "Pre-Operational";
            else if (st == 0x05) stTxt = "Operational";
            else if (st == 0x04) stTxt = "Stopped";
            else stTxt = "unknown";
        }

        char buf[64];
        snprintf(buf, sizeof(buf), "State: %s (0x%02X)", stTxt, (unsigned)st);
        lv_label_set_text(m_toolsLblState, buf);

        if (!m_nodeSeen[nid] || lastMs == 0) {
            lv_label_set_text(m_toolsLblLastSeen, "Last seen: ---");
        } else {
            const uint32_t age = millis() - lastMs;
            snprintf(buf, sizeof(buf), "Last seen: %lu ms", (unsigned long)age);
            lv_label_set_text(m_toolsLblLastSeen, buf);
        }
    }

    void buildConfigScreen() {
        m_cfgScreen = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(m_cfgScreen, AP10UI_COLOR_CFG_BG, 0);
        lv_obj_set_style_bg_opa(m_cfgScreen, LV_OPA_COVER, 0);
        lv_obj_clear_flag(m_cfgScreen, LV_OBJ_FLAG_SCROLLABLE);

        // Header
        lv_obj_t* hdr = lv_obj_create(m_cfgScreen);
        lv_obj_set_size(hdr, 800, 60);
        lv_obj_set_pos(hdr, 0, 0);
        lv_obj_set_style_bg_color(hdr, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(hdr, 0, 0);
        lv_obj_set_style_radius(hdr, 0, 0);
        lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* hdrTitle = lv_label_create(hdr);
        lv_label_set_text(hdrTitle, LV_SYMBOL_SETTINGS "  Konfiguration");
        lv_obj_set_style_text_color(hdrTitle, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(hdrTitle, &lv_font_montserrat_20, 0);
        lv_obj_align(hdrTitle, LV_ALIGN_LEFT_MID, 15, 0);

        lv_obj_t* btnBack = lv_btn_create(hdr);
        lv_obj_set_size(btnBack, 110, 40);
        lv_obj_align(btnBack, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(btnBack, AP10UI_COLOR_DARK_BTN, 0);
        lv_obj_set_style_radius(btnBack, 8, 0);
        lv_obj_add_event_cb(btnBack, btnBackCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* backLbl = lv_label_create(btnBack);
        lv_label_set_text(backLbl, LV_SYMBOL_LEFT "  Zurueck");
        lv_obj_set_style_text_color(backLbl, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(backLbl, &lv_font_montserrat_16, 0);
        lv_obj_center(backLbl);

        // ---- Linke Spalte: Einstellungen ----
        lv_obj_t* cardLeft = lv_obj_create(m_cfgScreen);
        lv_obj_set_size(cardLeft, 370, 390);
        lv_obj_set_pos(cardLeft, 10, 70);
        lv_obj_set_style_bg_color(cardLeft, AP10UI_COLOR_CFG_CARD, 0);
        lv_obj_set_style_border_color(cardLeft, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(cardLeft, 2, 0);
        lv_obj_set_style_radius(cardLeft, 12, 0);
        lv_obj_clear_flag(cardLeft, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* settTitle = lv_label_create(cardLeft);
        lv_label_set_text(settTitle, "Einstellungen");
        lv_obj_set_style_text_color(settTitle, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(settTitle, &lv_font_montserrat_18, 0);
        lv_obj_align(settTitle, LV_ALIGN_TOP_MID, 0, 8);

        // Node-ID
        makeLabel(cardLeft, "Node-ID (1-127):", AP10UI_COLOR_GRAY, &lv_font_montserrat_16, 10, 45);

        static char nodeIdOptions[1024];
        nodeIdOptions[0] = '\0';
        for (int i = 1; i <= 127; i++) {
            char tmp[8];
            snprintf(tmp, sizeof(tmp), "%d", i);
            strncat(nodeIdOptions, tmp, sizeof(nodeIdOptions) - strlen(nodeIdOptions) - 1);
            if (i < 127) strncat(nodeIdOptions, "\n", sizeof(nodeIdOptions) - strlen(nodeIdOptions) - 1);
        }

        m_cfgRollerNodeId = lv_roller_create(cardLeft);
        lv_roller_set_options(m_cfgRollerNodeId, nodeIdOptions, LV_ROLLER_MODE_NORMAL);
        lv_roller_set_visible_row_count(m_cfgRollerNodeId, 3);
        lv_roller_set_selected(m_cfgRollerNodeId, (uint16_t)(m_nodeId - 1), LV_ANIM_OFF);
        lv_obj_set_size(m_cfgRollerNodeId, 120, 90);
        lv_obj_set_pos(m_cfgRollerNodeId, 220, 38);
        lv_obj_set_style_text_font(m_cfgRollerNodeId, &lv_font_montserrat_16, 0);
        lv_obj_set_style_bg_color(m_cfgRollerNodeId, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_text_color(m_cfgRollerNodeId, AP10UI_COLOR_WHITE, 0);

        // Baudrate
        makeLabel(cardLeft, "Baudrate:", AP10UI_COLOR_GRAY, &lv_font_montserrat_16, 10, 148);

        m_cfgDropBaud = lv_dropdown_create(cardLeft);
        lv_dropdown_set_options(m_cfgDropBaud,
                                "125 kBit/s\n"
                                "250 kBit/s\n"
                                "500 kBit/s\n"
                                "1 MBit/s");
        lv_dropdown_set_selected(m_cfgDropBaud, 1);
        lv_obj_set_size(m_cfgDropBaud, 180, 44);
        lv_obj_set_pos(m_cfgDropBaud, 160, 142);
        lv_obj_set_style_bg_color(m_cfgDropBaud, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_text_color(m_cfgDropBaud, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(m_cfgDropBaud, &lv_font_montserrat_16, 0);

        // Skalierungsfaktor
        makeLabel(cardLeft, "Skalierung\n(mm/Digit):", AP10UI_COLOR_GRAY, &lv_font_montserrat_16, 10, 205);
        makeLabel(cardLeft, "z.B. 0.10 = 1 Digit -> 0.1 mm", AP10UI_COLOR_GRAY, &lv_font_montserrat_12, 10, 270);

        m_cfgSpinScale = lv_spinbox_create(cardLeft);
        lv_spinbox_set_range(m_cfgSpinScale, 1, 999);
        lv_spinbox_set_digit_format(m_cfgSpinScale, 3, 2);
        lv_spinbox_set_value(m_cfgSpinScale, 10);
        lv_spinbox_set_step(m_cfgSpinScale, 1);
        lv_obj_set_size(m_cfgSpinScale, 160, 44);
        lv_obj_set_pos(m_cfgSpinScale, 160, 200);
        lv_obj_set_style_bg_color(m_cfgSpinScale, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_text_color(m_cfgSpinScale, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(m_cfgSpinScale, &lv_font_montserrat_20, 0);

        lv_obj_t* btnPlus = lv_btn_create(cardLeft);
        lv_obj_set_size(btnPlus, 44, 44);
        lv_obj_set_pos(btnPlus, 325, 200);
        lv_obj_set_style_bg_color(btnPlus, AP10UI_COLOR_GREEN_BTN, 0);
        lv_obj_add_event_cb(btnPlus, [](lv_event_t*) {
            if (s_instance && s_instance->m_cfgSpinScale)
                lv_spinbox_increment(s_instance->m_cfgSpinScale);
        }, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* plusLbl = lv_label_create(btnPlus);
        lv_label_set_text(plusLbl, LV_SYMBOL_PLUS);
        lv_obj_set_style_text_color(plusLbl, AP10UI_COLOR_WHITE, 0);
        lv_obj_center(plusLbl);

        lv_obj_t* btnMinus = lv_btn_create(cardLeft);
        lv_obj_set_size(btnMinus, 44, 44);
        lv_obj_set_pos(btnMinus, 110, 200);
        lv_obj_set_style_bg_color(btnMinus, AP10UI_COLOR_RED, 0);
        lv_obj_add_event_cb(btnMinus, [](lv_event_t*) {
            if (s_instance && s_instance->m_cfgSpinScale)
                lv_spinbox_decrement(s_instance->m_cfgSpinScale);
        }, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* minusLbl = lv_label_create(btnMinus);
        lv_label_set_text(minusLbl, LV_SYMBOL_MINUS);
        lv_obj_set_style_text_color(minusLbl, AP10UI_COLOR_WHITE, 0);
        lv_obj_center(minusLbl);

        // Uebernehmen
        lv_obj_t* btnApply = lv_btn_create(cardLeft);
        lv_obj_set_size(btnApply, 340, 55);
        lv_obj_set_pos(btnApply, 10, 315);
        lv_obj_set_style_bg_color(btnApply, AP10UI_COLOR_GREEN_BTN, 0);
        lv_obj_set_style_radius(btnApply, 10, 0);
        lv_obj_add_event_cb(btnApply, btnApplyCb, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* applyLbl = lv_label_create(btnApply);
        lv_label_set_text(applyLbl, LV_SYMBOL_OK "  Uebernehmen & Neustart CAN");
        lv_obj_set_style_text_color(applyLbl, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(applyLbl, &lv_font_montserrat_16, 0);
        lv_obj_center(applyLbl);

        // ---- Rechte Spalte: CAN-Info ----
        lv_obj_t* cardRight = lv_obj_create(m_cfgScreen);
        lv_obj_set_size(cardRight, 390, 390);
        lv_obj_set_pos(cardRight, 400, 70);
        lv_obj_set_style_bg_color(cardRight, AP10UI_COLOR_CFG_CARD, 0);
        lv_obj_set_style_border_color(cardRight, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(cardRight, 2, 0);
        lv_obj_set_style_radius(cardRight, 12, 0);
        lv_obj_clear_flag(cardRight, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* infoTitle = lv_label_create(cardRight);
        lv_label_set_text(infoTitle, "CAN-Telegramm Info (Live)");
        lv_obj_set_style_text_color(infoTitle, AP10UI_COLOR_WHITE, 0);
        lv_obj_set_style_text_font(infoTitle, &lv_font_montserrat_18, 0);
        lv_obj_align(infoTitle, LV_ALIGN_TOP_MID, 0, 8);

        lv_obj_t* sep = lv_obj_create(cardRight);
        lv_obj_set_size(sep, 360, 1);
        lv_obj_set_pos(sep, 10, 38);
        lv_obj_set_style_bg_color(sep, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(sep, 0, 0);

        lv_obj_t* lblProto = lv_label_create(cardRight);
        lv_label_set_text(lblProto,
            "COB-ID Uebersicht:\n"
            "  0x000        NMT Kommando\n"
            "  0x080        SYNC\n"
            "  0x180+NodeID TPDO1 (Position)\n"
            "  0x280+NodeID TPDO2 (sync)\n"
            "  0x580+NodeID SDO Response\n"
            "  0x600+NodeID SDO Request\n"
            "  0x700+NodeID Heartbeat/Boot-Up\n\n"
            "NMT: 01=Start  02=Stop  80=PreOp\n"
            "SDO Write: 23h  SDO Read: 40h");
        lv_obj_set_style_text_color(lblProto, AP10UI_COLOR_GRAY, 0);
        lv_obj_set_style_text_font(lblProto, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(lblProto, 10, 48);

        lv_obj_t* sep2 = lv_obj_create(cardRight);
        lv_obj_set_size(sep2, 360, 1);
        lv_obj_set_pos(sep2, 10, 230);
        lv_obj_set_style_bg_color(sep2, AP10UI_COLOR_ACCENT, 0);
        lv_obj_set_style_border_width(sep2, 0, 0);

        makeLabel(cardRight, "Live-Statistiken:", AP10UI_COLOR_WHITE, &lv_font_montserrat_16, 10, 240);

        m_cfgLblRx       = makeLabel(cardRight, "CAN RX:    0", AP10UI_COLOR_GREEN,    &lv_font_montserrat_16, 10, 265);
        m_cfgLblTx       = makeLabel(cardRight, "CAN TX:    0", AP10UI_COLOR_BLUE_BTN, &lv_font_montserrat_16, 10, 287);
        m_cfgLblErr      = makeLabel(cardRight, "CAN Err:   0", AP10UI_COLOR_RED,      &lv_font_montserrat_16, 10, 309);
        m_cfgLblRecovery = makeLabel(cardRight, "Recovery:  0", AP10UI_COLOR_ORANGE_BTN, &lv_font_montserrat_16, 10, 331);
        m_cfgLblSync     = makeLabel(cardRight, "SYNC TX:   0", AP10UI_COLOR_YELLOW,   &lv_font_montserrat_16, 10, 353);
        m_cfgLblPdo      = makeLabel(cardRight, "PDO-Upd:   0", AP10UI_COLOR_WHITE,    &lv_font_montserrat_16, 10, 375);

        // Footer
        lv_obj_t* cfgFooter = lv_label_create(m_cfgScreen);
        lv_label_set_text(cfgFooter,
            "Hinweis: Nach 'Uebernehmen' wird CAN neu gestartet. Verbindung kurz unterbrochen.");
        lv_obj_set_style_text_color(cfgFooter, AP10UI_COLOR_GRAY, 0);
        lv_obj_set_style_text_font(cfgFooter, &lv_font_montserrat_12, 0);
        lv_obj_set_pos(cfgFooter, 10, 465);
    }

    // -----------------------------------------------------------------------
    // Hilfsfunktion: Baudrate-Index -> uint32_t
    // -----------------------------------------------------------------------
    static uint32_t baudrateFromIndex(uint16_t idx) {
        switch (idx) {
            case 0:  return 125000;
            case 1:  return 250000;
            case 2:  return 500000;
            case 3:  return 1000000;
            default: return 250000;
        }
    }

    static uint32_t baudrateFromToolsIndex(uint16_t idx) {
        // identisch, aber getrennt damit Tools-UI spaeter eigene Auswahl haben kann
        return baudrateFromIndex(idx);
    }

    // -----------------------------------------------------------------------
    // Button Callbacks
    // -----------------------------------------------------------------------

    static void btnConfigCb(lv_event_t*) {
        if (!s_instance || !s_instance->m_cfgScreen) return;
        lv_scr_load_anim(s_instance->m_cfgScreen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }

    // [NEU] Tools/Node-Scanner oeffnen
    static void btnToolsCb(lv_event_t*) {
        if (!s_instance || !s_instance->m_toolsScreen) return;
        lv_scr_load_anim(s_instance->m_toolsScreen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }

    static void btnBackCb(lv_event_t*) {
        if (!s_instance || !s_instance->m_screen) return;
        lv_scr_load_anim(s_instance->m_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    }

    // [NEU] Tools-Screen -> Hauptscreen
    static void btnToolsBackCb(lv_event_t*) {
        if (!s_instance || !s_instance->m_screen) return;
        lv_scr_load_anim(s_instance->m_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
    }

    static void btnApplyCb(lv_event_t*) {
        if (!s_instance) return;

        uint16_t rollerIdx = lv_roller_get_selected(s_instance->m_cfgRollerNodeId);
        uint8_t  newNodeId = (uint8_t)(rollerIdx + 1);
        uint16_t baudIdx   = lv_dropdown_get_selected(s_instance->m_cfgDropBaud);
        uint32_t newBaud   = baudrateFromIndex(baudIdx);
        int32_t  spinVal   = lv_spinbox_get_value(s_instance->m_cfgSpinScale);
        float    newScale  = (float)spinVal / 100.0f;

        Serial.printf("[UI] Konfig: NodeID=%u Baud=%lu Scale=%.2f\n",
                      (unsigned)newNodeId, (unsigned long)newBaud, newScale);

        if (s_instance->m_nodeIdLabel) {
            char buf[64];
            const char* baudStr = "?";
            switch (newBaud) {
                case 125000:  baudStr = "125 kBit/s"; break;
                case 250000:  baudStr = "250 kBit/s"; break;
                case 500000:  baudStr = "500 kBit/s"; break;
                case 1000000: baudStr = "1 MBit/s";   break;
            }
            snprintf(buf, sizeof(buf), "Node-ID: %u  |  %s", (unsigned)newNodeId, baudStr);
            lv_label_set_text(s_instance->m_nodeIdLabel, buf);
        }

        if (s_instance->m_configCallback) {
            AP10UI_Config cfg;
            cfg.nodeId      = newNodeId;
            cfg.baudrate    = newBaud;
            cfg.scaleFactor = newScale;
            // Tools-Felder deaktiviert
            cfg.toolsApply = false;
            cfg.toolsTargetNodeId = 0;
            cfg.toolsNewNodeId = 0;
            cfg.toolsNewBaudrate = 0;
            s_instance->m_configCallback(cfg);
            
        }

        if (s_instance->m_screen) {
            lv_scr_load_anim(s_instance->m_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
        }
    }

    // ---- [FIX #2 + #4] 0-Setzen: NUR Software-Offset, KEIN SDO ----
    static void btnZeroCb(lv_event_t*) {
        if (!s_instance) return;

        if (s_instance->m_ap04) {
            static const char* btns[] = {"Ja", "Nein", ""};
            lv_obj_t* mbox = lv_msgbox_create(nullptr,
                "Nullpunkt setzen?",
                "ESP32 Software-Offset auf aktuelle Position setzen.\n"
                "Die SIKO-Anzeige zeigt weiterhin ihren eigenen Wert.\n"
                "Der ESP32 rechnet intern relativ zu diesem Punkt.\n\n"
                "Fortfahren?",
                btns, false);
            lv_obj_center(mbox);

            // WICHTIG: lv_event_get_target() kann ein Child der MsgBox sein.
            // Darum den echten MsgBox-Pointer als user_data verwenden.
            lv_obj_add_event_cb(mbox, [](lv_event_t* ev) {
                lv_obj_t* box = (lv_obj_t*)lv_event_get_user_data(ev);
                if (!box) return;

                const char* txt = lv_msgbox_get_active_btn_text(box);
                if (txt && strcmp(txt, "Ja") == 0) {
                    Serial.println("[UI] AP04 Zero: setZeroLocal()");
                    if (s_instance && s_instance->m_ap04) {
                        s_instance->m_ap04->setZeroLocal();
                    }
                } else {
                    Serial.println("[UI] AP04 Zero: abgebrochen");
                }
                lv_msgbox_close(box);
            }, LV_EVENT_VALUE_CHANGED, mbox);
            return;
        }

        if (s_instance->m_ap10) {
            s_instance->m_ap10->setZero();
        }
    }

    // ---- Min/Max Reset ----
    static void btnResetMinMaxCb(lv_event_t*) {
        if (!s_instance) return;
        Serial.println("[UI] Min/Max Reset");
        if (s_instance->m_ap10) s_instance->m_ap10->resetMinMax();
        if (s_instance->m_ap04) s_instance->m_ap04->resetMinMax();
    }

    // ---- [FIX #1] Reset-Com: LV_EVENT_VALUE_CHANGED ----
    // [NEU] Tools: Anwenden (NMT Tools / Hinweis fuer Node-ID & Baudrate)
    static void btnToolsApplyCb(lv_event_t*) {
        if (!s_instance) return;

        // AP04: laut Handbuch kein LSS -> Node-ID/Baudrate nicht per CANopen Standard aenderbar.
        // Wir loesen hier daher KEINE Umkonfiguration aus, sondern zeigen einen klaren Hinweis.
        static const char* btns[] = {"OK", ""};
        lv_obj_t* mbox = lv_msgbox_create(nullptr,
            "Nicht unterstuetzt",
            "Dieses Geraet unterstuetzt kein LSS.\n\n"
            "Node-ID und Baudrate lassen sich nicht per NMT aendern.\n"
            "Bitte am Geraet (Setup/DIP) einstellen oder herstellerspezifisch per SDO,\n"
            "falls vom jeweiligen Geraet unterstuetzt.",
            btns, false);
        lv_obj_center(mbox);
    }

    // [NEW] AP04: Set Node-ID Dialog (SDO 5F0Ah wird in der App ausgeführt)
    static void btnSetNodeIdCb(lv_event_t*) {
        if (!s_instance || !s_instance->m_ap04) return;

        static const char* btns[] = {"OK", "Abbrechen", ""};
        lv_obj_t* mbox = lv_msgbox_create(nullptr,
            "AP04 Node-ID setzen",
            "Neue Node-ID eingeben (1..127).\n\n"
            "Nach OK: SDO 5F0Ah schreiben + ResetComm.\n"
            "Geraet rebootet kurz.",
            btns, false);
        lv_obj_center(mbox);

        s_instance->m_mboxSetNodeId = mbox;

        // Textarea
        lv_obj_t* ta = lv_textarea_create(mbox);
        s_instance->m_taSetNodeId = ta;
        lv_obj_set_width(ta, 140);
        lv_textarea_set_one_line(ta, true);
        lv_textarea_set_accepted_chars(ta, "0123456789");
        lv_textarea_set_max_length(ta, 3);
        char b[8];
        snprintf(b, sizeof(b), "%u", (unsigned)s_instance->m_nodeId);
        lv_textarea_set_text(ta, b);
        lv_obj_align(ta, LV_ALIGN_BOTTOM_LEFT, 16, -220);

        // Keyboard
        lv_obj_t* kb = lv_keyboard_create(mbox);
        s_instance->m_kbSetNodeId = kb;
        lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
        lv_keyboard_set_textarea(kb, ta);
        lv_obj_set_size(kb, 520, 180);
        lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, -20);

        // OK/Cancel
        lv_obj_add_event_cb(mbox, [](lv_event_t* ev) {
            lv_obj_t* box = (lv_obj_t*)lv_event_get_user_data(ev);
            if (!box || !s_instance) return;

            const char* txt = lv_msgbox_get_active_btn_text(box);
            if (txt && strcmp(txt, "OK") == 0) {
                const char* s = s_instance->m_taSetNodeId ? lv_textarea_get_text(s_instance->m_taSetNodeId) : nullptr;
                int v = s ? atoi(s) : 0;

                if (v < 1 || v > 127) {
                    Serial.println("[UI] AP04 SetNodeId: invalid range");
                    return;
                }

                Serial.printf("[UI] AP04 SetNodeId: request %d\n", v);

                if (s_instance->m_configCallback) {
                    AP10UI_Config cfg;
                    cfg.nodeId = s_instance->m_nodeId;
                    cfg.baudrate = 0;
                    cfg.scaleFactor = 0.0f;
                    cfg.toolsApply = true;
                    cfg.toolsTargetNodeId = s_instance->m_nodeId;
                    cfg.toolsNewNodeId = (uint8_t)v;
                    cfg.toolsNewBaudrate = 0;
                    s_instance->m_configCallback(cfg);
                }
            } else {
                Serial.println("[UI] AP04 SetNodeId: abgebrochen");
            }

            s_instance->m_mboxSetNodeId = nullptr;
            s_instance->m_taSetNodeId = nullptr;
            s_instance->m_kbSetNodeId = nullptr;
            lv_msgbox_close(box);
        }, LV_EVENT_VALUE_CHANGED, mbox);
    }

    static void btnResetCommCb(lv_event_t*) {
        if (!s_instance || !s_instance->m_ap04) return;

        static const char* btns[] = {"Ja", "Nein", ""};
        lv_obj_t* mbox = lv_msgbox_create(nullptr,
            "Reset Communication?",
            "AP04 CANopen Reset Communication + NMT Start\n"
            "(Recovery nach Verbindungsabbruch)\n\n"
            "Fortfahren?",
            btns, false);
        lv_obj_center(mbox);

        // [FIX #1] LV_EVENT_VALUE_CHANGED – "Nein" loest KEINEN Reset aus
        // WICHTIG: lv_event_get_target() kann ein Child der MsgBox sein.
        // Darum den echten MsgBox-Pointer als user_data verwenden.
        lv_obj_add_event_cb(mbox, [](lv_event_t* ev) {
            lv_obj_t* box = (lv_obj_t*)lv_event_get_user_data(ev);
            if (!box) return;

            const char* txt = lv_msgbox_get_active_btn_text(box);
            if (txt && strcmp(txt, "Ja") == 0) {
                Serial.println("[UI] Reset-Com bestaetigt");
                if (s_instance && s_instance->m_ap04) {
                    s_instance->m_ap04->requestResetComm();
                }
            } else {
                Serial.println("[UI] Reset-Com abgebrochen – KEIN Reset!");
            }
            lv_msgbox_close(box);
        }, LV_EVENT_VALUE_CHANGED, mbox);
    }
};
