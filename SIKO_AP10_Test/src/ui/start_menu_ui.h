#pragma once

/*
 * Start Menu UI (LVGL) - two-screen version
 *
 * Screen 1: Start
 *  - Auto-Scan Baud
 *  - Node list with row click + CONNECT
 *  - Button: TOOLS
 *
 * Screen 2: Tools
 *  - Sniffer toggle
 *  - Fix baud buttons (125k/250k/500k) + Active baud label
 *  - NMT START ALL
 *  - Loopback NO_ACK toggle
 *  - SDO Ping Scan 1..32 (does NOT change baud)
 *  - BACK
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
    // Start screen
    std::function<void()> onScanAuto = nullptr;

    // Tools screen
    std::function<void(uint32_t baud)> onSetFixedBaud = nullptr; // 125k/250k/500k
    std::function<void()> onNmtStartBroadcast = nullptr;
    std::function<void(bool enabled)> onLoopbackToggle = nullptr; // NO_ACK
    std::function<void(bool enabled)> onSnifferToggle = nullptr;
    std::function<void()> onPingScan1_32 = nullptr;
    std::function<void()> onIdentifyAll = nullptr;

    // Navigation / actions
    std::function<void(uint8_t nodeId)> onOpenNode = nullptr;
    std::function<void(uint8_t nodeId)> onConnectNode = nullptr;
};

class StartMenuUI {
public:
    void setCallbacks(const StartMenu_Callbacks& cbs) { m_cbs = cbs; }

    lv_obj_t* screenStart() const { return m_screenStart; }
    lv_obj_t* screenTools() const { return m_screenTools; }

    // For compatibility with existing code that expects screen()
    lv_obj_t* screen() const { return m_screenStart ? m_screenStart : m_screenTools; }

    void create() {
        createStartScreen();
        createToolsScreen();

        s_inst = this;
        lv_scr_load(m_screenStart);
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

    void setLoopbackEnabled(bool en) {
        m_loopbackEnabled = en;
        setLoopbackUi(en);
    }

    void setActiveBaud(uint32_t baud) {
        m_activeBaud = baud;
        setActiveBaudUi(baud);
    }

private:
    // -------------------- UI helpers --------------------

    static void styleScreen(lv_obj_t* s) {
        lv_obj_set_style_bg_color(s, lv_color_hex(0x0D0D1A), 0);
        lv_obj_set_style_bg_opa(s, LV_OPA_COVER, 0);
        lv_obj_clear_flag(s, LV_OBJ_FLAG_SCROLLABLE);
    }

    static lv_obj_t* makeHeader(lv_obj_t* parent, const char* titleText) {
        lv_obj_t* hdr = lv_obj_create(parent);
        lv_obj_set_size(hdr, 800, 60);
        lv_obj_set_pos(hdr, 0, 0);
        lv_obj_set_style_bg_color(hdr, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(hdr, 0, 0);
        lv_obj_set_style_radius(hdr, 0, 0);
        lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* title = lv_label_create(hdr);
        lv_label_set_text(title, titleText);
        lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_LEFT_MID, 15, 0);
        return hdr;
    }

    static lv_obj_t* makeButton(lv_obj_t* parent, int x, int y, int w, int h,
                               uint32_t color, const char* text,
                               lv_event_cb_t cb, void* userData = nullptr) {
        lv_obj_t* b = lv_btn_create(parent);
        lv_obj_set_size(b, w, h);
        lv_obj_set_pos(b, x, y);
        lv_obj_set_style_bg_color(b, lv_color_hex(color), 0);
        lv_obj_set_style_radius(b, 10, 0);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, userData);

        lv_obj_t* l = lv_label_create(b);
        lv_label_set_text(l, text);
        lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_18, 0);
        lv_obj_center(l);
        return b;
    }

    void setSnifferUi(bool en) {
        if (!m_btnSniffer || !m_lblSniffer) return;
        lv_obj_set_style_bg_color(m_btnSniffer, en ? lv_color_hex(0xCC6600) : lv_color_hex(0x444444), 0);
        lv_label_set_text(m_lblSniffer, en ? (LV_SYMBOL_EYE_OPEN "  Sniffer: ON")
                                          : (LV_SYMBOL_EYE_CLOSE "  Sniffer: OFF"));
    }

    void setLoopbackUi(bool en) {
        if (!m_btnLoopback || !m_lblLoopback) return;
        lv_obj_set_style_bg_color(m_btnLoopback, en ? lv_color_hex(0xCC6600) : lv_color_hex(0x444444), 0);
        lv_label_set_text(m_lblLoopback, en ? "Loopback: ON" : "Loopback: OFF");
    }

    void setActiveBaudUi(uint32_t baud) {
        if (!m_lblActiveBaud) return;
        char buf[64];
        if (baud == 0) snprintf(buf, sizeof(buf), "Active: ---");
        else snprintf(buf, sizeof(buf), "Active: %lu kBit/s", (unsigned long)(baud / 1000));
        lv_label_set_text(m_lblActiveBaud, buf);

        auto setBtn = [](lv_obj_t* b, bool sel){
            if (!b) return;
            lv_obj_set_style_bg_color(b, sel ? lv_color_hex(0x007744) : lv_color_hex(0x444444), 0);
        };
        setBtn(m_btnBaud125, baud == 125000);
        setBtn(m_btnBaud250, baud == 250000);
        setBtn(m_btnBaud500, baud == 500000);
    }

    // -------------------- Screen builders --------------------

    void createStartScreen() {
        m_screenStart = lv_obj_create(nullptr);
        styleScreen(m_screenStart);
        makeHeader(m_screenStart, "CANopen Master - Start");

        // Auto scan
        makeButton(m_screenStart, 10, 80, 260, 60, 0x007744,
                   LV_SYMBOL_REFRESH "  Auto-Scan Baud",
                   StartMenuUI::onScanAutoClicked);

        // Tools nav
        makeButton(m_screenStart, 290, 80, 220, 60, 0x0066CC,
                   LV_SYMBOL_SETTINGS "  TOOLS",
                   StartMenuUI::onOpenToolsClicked);

        // Hint
        lv_obj_t* hint = lv_label_create(m_screenStart);
        lv_label_set_text(hint,
            "Start: Scan oder Node auswaehlen.\n"
            "Tools: Sniffer, Fix Baud, NMT, Loopback, Ping Scan."
        );
        lv_obj_set_style_text_color(hint, lv_color_hex(0xFFBB33), 0);
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(hint, 530, 82);

        // Live status line (scan / ping / identify feedback)
        m_lblStatus = lv_label_create(m_screenStart);
        lv_label_set_text(m_lblStatus, LV_SYMBOL_BELL "  Bereit.");
        lv_obj_set_style_text_color(m_lblStatus, lv_color_hex(0x33DD66), 0);
        lv_obj_set_style_text_font(m_lblStatus, &lv_font_montserrat_18, 0);
        lv_obj_set_pos(m_lblStatus, 12, 150);
        lv_label_set_long_mode(m_lblStatus, LV_LABEL_LONG_DOT);
        lv_obj_set_width(m_lblStatus, 776);

        // Node list
        m_list = lv_list_create(m_screenStart);
        lv_obj_set_size(m_list, 780, 296);
        lv_obj_set_pos(m_list, 10, 182);
        lv_obj_set_style_bg_color(m_list, lv_color_hex(0x12122A), 0);
        lv_obj_set_style_border_color(m_list, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(m_list, 2, 0);
        lv_obj_set_style_radius(m_list, 12, 0);
    }

public:
    // Set the live status line (call under the LVGL lock). color: 0=info,1=ok,2=warn.
    void setStatus(const char* text, uint8_t kind = 0) {
        if (!m_lblStatus || !text) return;
        const uint32_t col = (kind == 1) ? 0x33DD66 : (kind == 2) ? 0xFF6666 : 0xFFBB33;
        lv_obj_set_style_text_color(m_lblStatus, lv_color_hex(col), 0);
        lv_label_set_text(m_lblStatus, text);
    }

private:

    void createToolsScreen() {
        m_screenTools = lv_obj_create(nullptr);
        styleScreen(m_screenTools);
        makeHeader(m_screenTools, "CANopen Master - Tools");

        // BACK
        makeButton(m_screenTools, 10, 80, 180, 60, 0x444444,
                   LV_SYMBOL_LEFT "  BACK",
                   StartMenuUI::onBackToStartClicked);

        // Sniffer toggle
        m_btnSniffer = lv_btn_create(m_screenTools);
        lv_obj_set_size(m_btnSniffer, 240, 60);
        lv_obj_set_pos(m_btnSniffer, 210, 80);
        lv_obj_set_style_radius(m_btnSniffer, 10, 0);
        lv_obj_add_event_cb(m_btnSniffer, StartMenuUI::onSnifferClicked, LV_EVENT_CLICKED, nullptr);
        m_lblSniffer = lv_label_create(m_btnSniffer);
        lv_obj_set_style_text_color(m_lblSniffer, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(m_lblSniffer, &lv_font_montserrat_18, 0);
        lv_obj_center(m_lblSniffer);
        setSnifferUi(false);

        // Fixed baud buttons
        lv_obj_t* lblBaud = lv_label_create(m_screenTools);
        lv_label_set_text(lblBaud, "Fix Baud:");
        lv_obj_set_style_text_color(lblBaud, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(lblBaud, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(lblBaud, 10, 160);

        m_btnBaud125 = makeButton(m_screenTools, 90, 150, 110, 44, 0x444444, "125k", StartMenuUI::onBaud125Clicked);
        m_btnBaud250 = makeButton(m_screenTools, 210, 150, 110, 44, 0x444444, "250k", StartMenuUI::onBaud250Clicked);
        m_btnBaud500 = makeButton(m_screenTools, 330, 150, 110, 44, 0x444444, "500k", StartMenuUI::onBaud500Clicked);

        m_lblActiveBaud = lv_label_create(m_screenTools);
        lv_label_set_text(m_lblActiveBaud, "Active: ---");
        lv_obj_set_style_text_color(m_lblActiveBaud, lv_color_hex(0xAAAAAA), 0);
        lv_obj_set_style_text_font(m_lblActiveBaud, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblActiveBaud, 460, 160);

        // NMT start all
        makeButton(m_screenTools, 10, 210, 220, 44, 0x0066CC, "NMT START ALL", StartMenuUI::onNmtStartAllClicked);

        // Loopback
        m_btnLoopback = lv_btn_create(m_screenTools);
        lv_obj_set_size(m_btnLoopback, 220, 44);
        lv_obj_set_pos(m_btnLoopback, 240, 210);
        lv_obj_set_style_bg_color(m_btnLoopback, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(m_btnLoopback, 10, 0);
        lv_obj_add_event_cb(m_btnLoopback, StartMenuUI::onLoopbackClicked, LV_EVENT_CLICKED, nullptr);
        m_lblLoopback = lv_label_create(m_btnLoopback);
        lv_label_set_text(m_lblLoopback, "Loopback: OFF");
        lv_obj_set_style_text_color(m_lblLoopback, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(m_lblLoopback);

        // Ping scan
        makeButton(m_screenTools, 470, 210, 220, 44, 0x007744, "PING SCAN 1..32", StartMenuUI::onPingScanClicked);

        // Identify all found nodes
        makeButton(m_screenTools, 470, 260, 220, 44, 0x4A2C82, "IDENTIFY ALL", StartMenuUI::onIdentifyClicked);

        lv_obj_t* hint = lv_label_create(m_screenTools);
        lv_label_set_text(hint,
            "Ping Scan: SDO read 0x1000 auf Node 1..32 (ohne Baudwechsel).\n"
            "Identify All: liest 0x1018 (Vendor/Product/Rev/Serial) aller gefundenen Nodes."
        );
        lv_obj_set_style_text_color(hint, lv_color_hex(0xFFBB33), 0);
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(hint, 10, 270);
    }

    // -------------------- Event handlers --------------------

    static void onScanAutoClicked(lv_event_t*) {
        Serial.println("[UI] ScanAuto clicked");
        if (s_inst && s_inst->m_cbs.onScanAuto) s_inst->m_cbs.onScanAuto();
    }

    static void onOpenToolsClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] Open TOOLS");
        lv_scr_load_anim(s_inst->m_screenTools, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }

    static void onBackToStartClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] Back to START");
        lv_scr_load_anim(s_inst->m_screenStart, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 0, false);
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
        s_inst->setLoopbackUi(s_inst->m_loopbackEnabled);
        Serial.printf("[UI] Loopback toggled: %s\n", s_inst->m_loopbackEnabled ? "ON" : "OFF");
        if (s_inst->m_cbs.onLoopbackToggle) s_inst->m_cbs.onLoopbackToggle(s_inst->m_loopbackEnabled);
    }

    static void onPingScanClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] PING SCAN 1..32 clicked");
        if (s_inst->m_cbs.onPingScan1_32) s_inst->m_cbs.onPingScan1_32();
    }

    static void onIdentifyClicked(lv_event_t*) {
        if (!s_inst) return;
        Serial.println("[UI] IDENTIFY ALL clicked");
        if (s_inst->m_cbs.onIdentifyAll) s_inst->m_cbs.onIdentifyAll();
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

private:
    StartMenu_Callbacks m_cbs;

    // Screens
    lv_obj_t* m_screenStart = nullptr;
    lv_obj_t* m_screenTools = nullptr;

    // Start screen widgets
    lv_obj_t* m_list = nullptr;
    lv_obj_t* m_lblStatus = nullptr;

    // Tools widgets
    lv_obj_t* m_btnSniffer = nullptr;
    lv_obj_t* m_lblSniffer = nullptr;
    bool m_snifferEnabled = false;

    lv_obj_t* m_btnBaud125 = nullptr;
    lv_obj_t* m_btnBaud250 = nullptr;
    lv_obj_t* m_btnBaud500 = nullptr;
    lv_obj_t* m_lblActiveBaud = nullptr;
    uint32_t m_activeBaud = 0;

    lv_obj_t* m_btnLoopback = nullptr;
    lv_obj_t* m_lblLoopback = nullptr;
    bool m_loopbackEnabled = false;

    inline static StartMenuUI* s_inst = nullptr;
};
