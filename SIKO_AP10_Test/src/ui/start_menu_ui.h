#pragma once

/*
 * Start Menu UI (LVGL)
 *
 * Purpose:
 * - Boot into a neutral screen (no auto-connect, no recovery)
 * - Provide Scan + Node list + Select/Connect hooks
 * - Provide Sniffer ON/OFF toggle (new)
 */

#include <Arduino.h>
#include <lvgl.h>
#include <functional>

struct StartMenu_NodeRow {
    uint8_t  nodeId = 0;
    const char* stateText = "---";
    uint32_t lastSeenMs = 0;
    const char* typeText = "Unknown";
};

struct StartMenu_Callbacks {
    std::function<void()> onScanAuto = nullptr;

    // Fixed baud quick-set (sniffer / discovery)
    std::function<void(uint32_t baud)> onSetFixedBaud = nullptr;

    // NMT quick actions
    std::function<void()> onNmtStartBroadcast = nullptr;

    // Loopback / No-ACK test mode
    std::function<void(bool enabled)> onLoopbackToggle = nullptr;

    // Sniffer
    std::function<void(bool enabled)> onSnifferToggle = nullptr;

    // Navigation
    std::function<void(uint8_t nodeId)> onOpenNode = nullptr;

    // Quick actions
    std::function<void(uint8_t nodeId)> onConnectNode = nullptr;

    // Dunker / DS402 quick actions
    std::function<void()> onDunkerReadStatus = nullptr;
    std::function<void()> onDunkerFaultReset = nullptr;
    std::function<void()> onDunkerEnable = nullptr;

    // SDO ping scan (fixed baud, no baud switching)
    std::function<void()> onPingScan1_32 = nullptr;
};

class StartMenuUI {
public:
    void setCallbacks(const StartMenu_Callbacks& cbs) { m_cbs = cbs; }

    lv_obj_t* screen() const { return m_screen; }

    void create() {
        m_screen = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(m_screen, lv_color_hex(0x0D0D1A), 0);
        lv_obj_set_style_bg_opa(m_screen, LV_OPA_COVER, 0);
        lv_obj_clear_flag(m_screen, LV_OBJ_FLAG_SCROLLABLE);

        // Header
        lv_obj_t* hdr = lv_obj_create(m_screen);
        lv_obj_set_size(hdr, 800, 60);
        lv_obj_set_pos(hdr, 0, 0);
        lv_obj_set_style_bg_color(hdr, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(hdr, 0, 0);
        lv_obj_set_style_radius(hdr, 0, 0);
        lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* title = lv_label_create(hdr);
        lv_label_set_text(title, "CANopen Master - Start");
        lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_LEFT_MID, 15, 0);

        // Scan button
        lv_obj_t* btnScanAuto = lv_btn_create(m_screen);
        lv_obj_set_size(btnScanAuto, 260, 60);
        lv_obj_set_pos(btnScanAuto, 10, 80);
        lv_obj_set_style_bg_color(btnScanAuto, lv_color_hex(0x007744), 0);
        lv_obj_set_style_radius(btnScanAuto, 10, 0);
        lv_obj_add_event_cb(btnScanAuto, StartMenuUI::onScanAutoClicked, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* lblAuto = lv_label_create(btnScanAuto);
        lv_label_set_text(lblAuto, LV_SYMBOL_REFRESH "  Auto-Scan Baud");
        lv_obj_set_style_text_color(lblAuto, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(lblAuto, &lv_font_montserrat_18, 0);
        lv_obj_center(lblAuto);

        // Sniffer toggle button (new)
        m_btnSniffer = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnSniffer, 220, 60);
        lv_obj_set_pos(m_btnSniffer, 10, 150);
        lv_obj_set_style_radius(m_btnSniffer, 10, 0);
        lv_obj_add_event_cb(m_btnSniffer, StartMenuUI::onSnifferClicked, LV_EVENT_CLICKED, nullptr);

        m_lblSniffer = lv_label_create(m_btnSniffer);
        lv_obj_set_style_text_color(m_lblSniffer, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(m_lblSniffer, &lv_font_montserrat_18, 0);
        lv_obj_center(m_lblSniffer);
        setSnifferUi(false);

        // Fixed baud quick buttons (125k / 250k / 500k)
        lv_obj_t* lblBaud = lv_label_create(m_screen);
        lv_label_set_text(lblBaud, "Fix Baud:");
        lv_obj_set_style_text_color(lblBaud, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(lblBaud, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(lblBaud, 250, 160);

        m_btnBaud125 = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnBaud125, 110, 44);
        lv_obj_set_pos(m_btnBaud125, 320, 150);
        lv_obj_set_style_bg_color(m_btnBaud125, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(m_btnBaud125, 10, 0);
        lv_obj_add_event_cb(m_btnBaud125, StartMenuUI::onBaud125Clicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* l125 = lv_label_create(m_btnBaud125);
        lv_label_set_text(l125, "125k");
        lv_obj_set_style_text_color(l125, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(l125);

        m_btnBaud250 = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnBaud250, 110, 44);
        lv_obj_set_pos(m_btnBaud250, 440, 150);
        lv_obj_set_style_bg_color(m_btnBaud250, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(m_btnBaud250, 10, 0);
        lv_obj_add_event_cb(m_btnBaud250, StartMenuUI::onBaud250Clicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* l250 = lv_label_create(m_btnBaud250);
        lv_label_set_text(l250, "250k");
        lv_obj_set_style_text_color(l250, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(l250);

        m_btnBaud500 = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnBaud500, 110, 44);
        lv_obj_set_pos(m_btnBaud500, 560, 150);
        lv_obj_set_style_bg_color(m_btnBaud500, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(m_btnBaud500, 10, 0);
        lv_obj_add_event_cb(m_btnBaud500, StartMenuUI::onBaud500Clicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* l500 = lv_label_create(m_btnBaud500);
        lv_label_set_text(l500, "500k");
        lv_obj_set_style_text_color(l500, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(l500);

        // NMT Start Broadcast button
        m_btnNmtStartAll = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnNmtStartAll, 220, 44);
        lv_obj_set_pos(m_btnNmtStartAll, 680, 150);
        lv_obj_set_style_bg_color(m_btnNmtStartAll, lv_color_hex(0x0066CC), 0);
        lv_obj_set_style_radius(m_btnNmtStartAll, 10, 0);
        lv_obj_add_event_cb(m_btnNmtStartAll, StartMenuUI::onNmtStartAllClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* lnmt = lv_label_create(m_btnNmtStartAll);
        lv_label_set_text(lnmt, "NMT START ALL");
        lv_obj_set_style_text_color(lnmt, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(lnmt);

        // Loopback toggle (NO_ACK)
        m_btnLoopback = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnLoopback, 220, 44);
        lv_obj_set_pos(m_btnLoopback, 680, 200);
        lv_obj_set_style_bg_color(m_btnLoopback, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(m_btnLoopback, 10, 0);
        lv_obj_add_event_cb(m_btnLoopback, StartMenuUI::onLoopbackClicked, LV_EVENT_CLICKED, nullptr);
        m_lblLoopback = lv_label_create(m_btnLoopback);
        lv_label_set_text(m_lblLoopback, "Loopback: OFF");
        lv_obj_set_style_text_color(m_lblLoopback, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(m_lblLoopback);

        // Dunker DS402 quick buttons (Node 1)
        m_btnDunkerStatus = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnDunkerStatus, 220, 44);
        lv_obj_set_pos(m_btnDunkerStatus, 560, 80);
        lv_obj_set_style_bg_color(m_btnDunkerStatus, lv_color_hex(0x4A2C82), 0);
        lv_obj_set_style_radius(m_btnDunkerStatus, 10, 0);
        lv_obj_add_event_cb(m_btnDunkerStatus, StartMenuUI::onDunkerStatusClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* lds = lv_label_create(m_btnDunkerStatus);
        lv_label_set_text(lds, "DUNKER STATUS");
        lv_obj_set_style_text_color(lds, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(lds);

        m_btnDunkerFaultReset = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnDunkerFaultReset, 220, 44);
        lv_obj_set_pos(m_btnDunkerFaultReset, 560, 130);
        lv_obj_set_style_bg_color(m_btnDunkerFaultReset, lv_color_hex(0x993333), 0);
        lv_obj_set_style_radius(m_btnDunkerFaultReset, 10, 0);
        lv_obj_add_event_cb(m_btnDunkerFaultReset, StartMenuUI::onDunkerFaultResetClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* lfr = lv_label_create(m_btnDunkerFaultReset);
        lv_label_set_text(lfr, "FAULT RESET");
        lv_obj_set_style_text_color(lfr, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(lfr);

        m_btnDunkerEnable = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnDunkerEnable, 220, 44);
        lv_obj_set_pos(m_btnDunkerEnable, 560, 180);
        lv_obj_set_style_bg_color(m_btnDunkerEnable, lv_color_hex(0x0066CC), 0);
        lv_obj_set_style_radius(m_btnDunkerEnable, 10, 0);
        lv_obj_add_event_cb(m_btnDunkerEnable, StartMenuUI::onDunkerEnableClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* len = lv_label_create(m_btnDunkerEnable);
        lv_label_set_text(len, "ENABLE DS402");
        lv_obj_set_style_text_color(len, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(len);

        // SDO ping scan (fixed baud)
        m_btnPingScan = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnPingScan, 220, 44);
        lv_obj_set_pos(m_btnPingScan, 560, 230);
        lv_obj_set_style_bg_color(m_btnPingScan, lv_color_hex(0x007744), 0);
        lv_obj_set_style_radius(m_btnPingScan, 10, 0);
        lv_obj_add_event_cb(m_btnPingScan, StartMenuUI::onPingScanClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* lps = lv_label_create(m_btnPingScan);
        lv_label_set_text(lps, "PING SCAN 1..32");
        lv_obj_set_style_text_color(lps, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(lps);

        m_lblActiveBaud = lv_label_create(m_screen);
        lv_label_set_text(m_lblActiveBaud, "Active: ---");
        lv_obj_set_style_text_color(m_lblActiveBaud, lv_color_hex(0xAAAAAA), 0);
        lv_obj_set_style_text_font(m_lblActiveBaud, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblActiveBaud, 250, 190);

        setActiveBaudUi(0);

        // Hint (shortened)
        lv_obj_t* hint = lv_label_create(m_screen);
        lv_label_set_text(hint,
            "Scan oder Fix Baud -> Sniffer ON.\n"
            "Dunker: Status/Reset/Enable (Node 1)."
        );
        lv_obj_set_style_text_color(hint, lv_color_hex(0xFFBB33), 0);
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(hint, 290, 80);

        // Node list container (moved down to make space for DS402 buttons)
        m_list = lv_list_create(m_screen);
        lv_obj_set_size(m_list, 780, 210);
        lv_obj_set_pos(m_list, 10, 390);
        lv_obj_set_style_bg_color(m_list, lv_color_hex(0x12122A), 0);
        lv_obj_set_style_border_color(m_list, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(m_list, 2, 0);
        lv_obj_set_style_radius(m_list, 12, 0);

        s_inst = this;
        lv_scr_load(m_screen);
    }

    void setNodes(const StartMenu_NodeRow* rows, size_t count) {
        if (!m_list) return;
        lv_obj_clean(m_list);
        for (size_t i = 0; i < count; i++) {
            char buf[96];
            snprintf(buf, sizeof(buf), "Node %u | %s | last %lu ms | %s",
                     (unsigned)rows[i].nodeId,
                     rows[i].stateText ? rows[i].stateText : "---",
                     (unsigned long)rows[i].lastSeenMs,
                     rows[i].typeText ? rows[i].typeText : "Unknown");

            lv_obj_t* btn = lv_list_add_btn(m_list, LV_SYMBOL_DIRECTORY, buf);
            lv_obj_set_height(btn, 44);

            lv_obj_add_event_cb(btn, StartMenuUI::onNodeBtnClicked, LV_EVENT_CLICKED, (void*)(uintptr_t)rows[i].nodeId);

            lv_obj_t* cbtn = lv_btn_create(btn);
            lv_obj_set_size(cbtn, 110, 34);
            lv_obj_align(cbtn, LV_ALIGN_RIGHT_MID, -8, 0);
            lv_obj_set_style_bg_color(cbtn, lv_color_hex(0x007744), 0);
            lv_obj_set_style_radius(cbtn, 8, 0);
            lv_obj_add_event_cb(cbtn, StartMenuUI::onConnectBtnClicked, LV_EVENT_CLICKED, (void*)(uintptr_t)rows[i].nodeId);

            lv_obj_t* clbl = lv_label_create(cbtn);
            lv_label_set_text(clbl, "CONNECT");
            lv_obj_set_style_text_color(clbl, lv_color_hex(0xFFFFFF), 0);
            lv_obj_center(clbl);
        }
    }

    void setSnifferEnabled(bool en) {
        m_snifferEnabled = en;
        setSnifferUi(en);
    }

    void setActiveBaud(uint32_t baud) {
        m_activeBaud = baud;
        setActiveBaudUi(baud);
    }

private:
    void setSnifferUi(bool en) {
        if (!m_btnSniffer || !m_lblSniffer) return;
        lv_obj_set_style_bg_color(m_btnSniffer, en ? lv_color_hex(0xCC6600) : lv_color_hex(0x444444), 0);
        lv_label_set_text(m_lblSniffer, en ? (LV_SYMBOL_EYE_OPEN "  Sniffer: ON")
                                          : (LV_SYMBOL_EYE_CLOSE "  Sniffer: OFF"));
    }

    void setActiveBaudUi(uint32_t baud) {
        if (!m_lblActiveBaud) return;
        char buf[64];
        if (baud == 0) snprintf(buf, sizeof(buf), "Active: ---");
        else snprintf(buf, sizeof(buf), "Active: %lu kBit/s", (unsigned long)(baud / 1000));
        lv_label_set_text(m_lblActiveBaud, buf);

        // Highlight the selected baud button
        auto setBtn = [](lv_obj_t* b, bool sel){
            if (!b) return;
            lv_obj_set_style_bg_color(b, sel ? lv_color_hex(0x007744) : lv_color_hex(0x444444), 0);
        };
        setBtn(m_btnBaud125, baud == 125000);
        setBtn(m_btnBaud250, baud == 250000);
        setBtn(m_btnBaud500, baud == 500000);
    }

    static void onScanAutoClicked(lv_event_t*) {
        Serial.println("[UI] ScanAuto clicked");
        if (s_inst && s_inst->m_cbs.onScanAuto) s_inst->m_cbs.onScanAuto();
    }

    static void onSnifferClicked(lv_event_t*) {
        if (!s_inst) return;
        s_inst->m_snifferEnabled = !s_inst->m_snifferEnabled;
        s_inst->setSnifferUi(s_inst->m_snifferEnabled);
        Serial.printf("[UI] Sniffer toggled: %s\n", s_inst->m_snifferEnabled ? "ON" : "OFF");
        if (s_inst->m_cbs.onSnifferToggle) s_inst->m_cbs.onSnifferToggle(s_inst->m_snifferEnabled);
    }

    static void onBaud125Clicked(lv_event_t*) {
        if (!s_inst) return;
        s_inst->m_activeBaud = 125000;
        s_inst->setActiveBaudUi(s_inst->m_activeBaud);
        Serial.println("[UI] Fix baud clicked: 125k");
        if (s_inst->m_cbs.onSetFixedBaud) s_inst->m_cbs.onSetFixedBaud(125000);
    }

    static void onBaud250Clicked(lv_event_t*) {
        if (!s_inst) return;
        s_inst->m_activeBaud = 250000;
        s_inst->setActiveBaudUi(s_inst->m_activeBaud);
        Serial.println("[UI] Fix baud clicked: 250k");
        if (s_inst->m_cbs.onSetFixedBaud) s_inst->m_cbs.onSetFixedBaud(250000);
    }

    static void onBaud500Clicked(lv_event_t*) {
        if (!s_inst) return;
        s_inst->m_activeBaud = 500000;
        s_inst->setActiveBaudUi(s_inst->m_activeBaud);
        Serial.println("[UI] Fix baud clicked: 500k");
        if (s_inst->m_cbs.onSetFixedBaud) s_inst->m_cbs.onSetFixedBaud(500000);
    }

    static void onNmtStartAllClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] NMT START ALL clicked");
        if (s_inst->m_cbs.onNmtStartBroadcast) s_inst->m_cbs.onNmtStartBroadcast();
    }

    static void onLoopbackClicked(lv_event_t*) {
        if (!s_inst) return;
        s_inst->m_loopbackEnabled = !s_inst->m_loopbackEnabled;
        lv_obj_set_style_bg_color(s_inst->m_btnLoopback,
                                  s_inst->m_loopbackEnabled ? lv_color_hex(0xCC6600) : lv_color_hex(0x444444), 0);
        lv_label_set_text(s_inst->m_lblLoopback, s_inst->m_loopbackEnabled ? "Loopback: ON" : "Loopback: OFF");
        Serial.printf("[UI] Loopback toggled: %s\n", s_inst->m_loopbackEnabled ? "ON" : "OFF");
        if (s_inst->m_cbs.onLoopbackToggle) s_inst->m_cbs.onLoopbackToggle(s_inst->m_loopbackEnabled);
    }

    static void onNodeBtnClicked(lv_event_t* ev) {
        const uint8_t nid = (uint8_t)(uintptr_t)lv_event_get_user_data(ev);
        Serial.printf("[UI] Node row clicked: %u\n", (unsigned)nid);
        if (s_inst && s_inst->m_cbs.onOpenNode) s_inst->m_cbs.onOpenNode(nid);
    }

    static void onConnectBtnClicked(lv_event_t* ev) {
        const uint8_t nid = (uint8_t)(uintptr_t)lv_event_get_user_data(ev);
        Serial.printf("[UI] CONNECT clicked: %u\n", (unsigned)nid);
        if (s_inst && s_inst->m_cbs.onConnectNode) s_inst->m_cbs.onConnectNode(nid);
    }

    static void onDunkerStatusClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] Dunker STATUS clicked");
        if (s_inst->m_cbs.onDunkerReadStatus) s_inst->m_cbs.onDunkerReadStatus();
    }

    static void onDunkerFaultResetClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] Dunker FAULT RESET clicked");
        if (s_inst->m_cbs.onDunkerFaultReset) s_inst->m_cbs.onDunkerFaultReset();
    }

    static void onDunkerEnableClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] Dunker ENABLE DS402 clicked");
        if (s_inst->m_cbs.onDunkerEnable) s_inst->m_cbs.onDunkerEnable();
    }

    static void onPingScanClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] PING SCAN 1..32 clicked");
        if (s_inst->m_cbs.onPingScan1_32) s_inst->m_cbs.onPingScan1_32();
    }

    lv_obj_t* m_screen = nullptr;
    lv_obj_t* m_list = nullptr;

    // sniffer UI
    lv_obj_t* m_btnSniffer = nullptr;
    lv_obj_t* m_lblSniffer = nullptr;
    bool m_snifferEnabled = false;

    // fixed baud UI
    lv_obj_t* m_btnBaud125 = nullptr;
    lv_obj_t* m_btnBaud250 = nullptr;
    lv_obj_t* m_btnBaud500 = nullptr;
    lv_obj_t* m_lblActiveBaud = nullptr;
    uint32_t m_activeBaud = 0;

    // NMT
    lv_obj_t* m_btnNmtStartAll = nullptr;

    // Loopback
    lv_obj_t* m_btnLoopback = nullptr;
    lv_obj_t* m_lblLoopback = nullptr;
    bool m_loopbackEnabled = false;

    // Dunker buttons
    lv_obj_t* m_btnDunkerStatus = nullptr;
    lv_obj_t* m_btnDunkerFaultReset = nullptr;
    lv_obj_t* m_btnDunkerEnable = nullptr;
    lv_obj_t* m_btnPingScan = nullptr;

    StartMenu_Callbacks m_cbs;

    inline static StartMenuUI* s_inst = nullptr;
};
