#pragma once

/*
 * Node Detail UI (Standard CANopen + basic config)
 *
 * Provides:
 * - Node info (NMT state, last seen)
 * - NMT controls: Start/PreOp/Stop/ResetNode/ResetComm
 * - Back button
 *
 * No SDO explorer yet (next step).
 */

#include <Arduino.h>
#include <lvgl.h>
#include <functional>

struct NodeDetail_Info {
    uint8_t nodeId = 0;
    const char* stateText = "---";
    uint32_t lastSeenMs = 0;
    const char* typeText = "Unknown";
};

struct NodeDetail_Callbacks {
    std::function<void()> onBack = nullptr;

    std::function<void(uint8_t nodeId)> onNmtStart = nullptr;
    std::function<void(uint8_t nodeId)> onNmtPreOp = nullptr;
    std::function<void(uint8_t nodeId)> onNmtStop = nullptr;
    std::function<void(uint8_t nodeId)> onNmtResetNode = nullptr;
    std::function<void(uint8_t nodeId)> onNmtResetComm = nullptr;

    // Config (AP04): set Node-ID via SDO + ResetComm
    std::function<void(uint8_t nodeId, uint8_t newNodeId)> onSetNodeId = nullptr;
};

class NodeDetailUI {
public:
    void setCallbacks(const NodeDetail_Callbacks& cbs) { m_cbs = cbs; }

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
        lv_label_set_text(title, "CANopen Node - Details");
        lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_LEFT_MID, 15, 0);

        lv_obj_t* btnBack = lv_btn_create(hdr);
        lv_obj_set_size(btnBack, 110, 40);
        lv_obj_align(btnBack, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(btnBack, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(btnBack, 8, 0);
        lv_obj_add_event_cb(btnBack, NodeDetailUI::onBackClicked, LV_EVENT_CLICKED, nullptr);

        lv_obj_t* backLbl = lv_label_create(btnBack);
        lv_label_set_text(backLbl, LV_SYMBOL_LEFT "  Zurueck");
        lv_obj_set_style_text_color(backLbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(backLbl);

        // Info card
        lv_obj_t* card = lv_obj_create(m_screen);
        lv_obj_set_size(card, 780, 110);
        lv_obj_set_pos(card, 10, 80);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x12122A), 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        m_lblNode = lv_label_create(card);
        lv_label_set_text(m_lblNode, "Node: ---");
        lv_obj_set_style_text_color(m_lblNode, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(m_lblNode, &lv_font_montserrat_18, 0);
        lv_obj_set_pos(m_lblNode, 12, 12);

        m_lblState = lv_label_create(card);
        lv_label_set_text(m_lblState, "State: ---");
        lv_obj_set_style_text_color(m_lblState, lv_color_hex(0xFFBB33), 0);
        lv_obj_set_style_text_font(m_lblState, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(m_lblState, 12, 44);

        m_lblLast = lv_label_create(card);
        lv_label_set_text(m_lblLast, "Last seen: ---");
        lv_obj_set_style_text_color(m_lblLast, lv_color_hex(0xAAAAAA), 0);
        lv_obj_set_style_text_font(m_lblLast, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblLast, 12, 72);

        // NMT buttons
        int x = 10;
        int y = 210;
        const int w = 250;
        const int h = 70;
        const int gap = 15;

        m_btnStart = makeBtn("NMT START", lv_color_hex(0x007744), x, y, w, h, NodeDetailUI::onStartClicked);
        m_btnPreOp = makeBtn("NMT PRE-OP", lv_color_hex(0xCC6600), x + (w + gap), y, w, h, NodeDetailUI::onPreOpClicked);
        m_btnStop  = makeBtn("NMT STOP", lv_color_hex(0x444444), x + 2*(w + gap), y, w, h, NodeDetailUI::onStopClicked);

        y += h + gap;
        m_btnResetNode = makeBtn("RESET NODE", lv_color_hex(0x666666), x, y, w, h, NodeDetailUI::onResetNodeClicked);
        m_btnResetComm = makeBtn("RESET COMM", lv_color_hex(0x666666), x + (w + gap), y, w, h, NodeDetailUI::onResetCommClicked);

        // Config row (AP04 / generic): Node-ID set
        y += h + gap;
        m_taNodeId = lv_textarea_create(m_screen);
        lv_obj_set_size(m_taNodeId, 160, 52);
        lv_obj_set_pos(m_taNodeId, x, y + 9);
        lv_textarea_set_one_line(m_taNodeId, true);
        lv_textarea_set_accepted_chars(m_taNodeId, "0123456789");
        lv_textarea_set_max_length(m_taNodeId, 3);
        lv_textarea_set_text(m_taNodeId, "1");
        lv_textarea_set_placeholder_text(m_taNodeId, "Node-ID");

        // On-screen numeric keypad (simple)
        m_kb = lv_keyboard_create(m_screen);
        lv_keyboard_set_mode(m_kb, LV_KEYBOARD_MODE_NUMBER);
        lv_keyboard_set_textarea(m_kb, m_taNodeId);
        lv_obj_set_width(m_kb, 780);
        lv_obj_set_pos(m_kb, 10, 520);

        m_btnSetNodeId = makeBtn("SET NODE-ID", lv_color_hex(0x1D4ED8), x + 180, y, 250, h, NodeDetailUI::onSetNodeIdClicked);

        s_inst = this;
    }

    void load() {
        if (m_screen) lv_scr_load_anim(m_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }

    void setInfo(const NodeDetail_Info& info) {
        m_info = info;
        if (m_lblNode) {
            char b[64];
            snprintf(b, sizeof(b), "Node: %u | Type: %s", (unsigned)info.nodeId, info.typeText ? info.typeText : "Unknown");
            lv_label_set_text(m_lblNode, b);
        }
        if (m_lblState) {
            char b[64];
            snprintf(b, sizeof(b), "State: %s", info.stateText ? info.stateText : "---");
            lv_label_set_text(m_lblState, b);
        }
        if (m_lblLast) {
            char b[64];
            if (info.lastSeenMs == 0) snprintf(b, sizeof(b), "Last seen: ---");
            else snprintf(b, sizeof(b), "Last seen: %lu ms", (unsigned long)info.lastSeenMs);
            lv_label_set_text(m_lblLast, b);
        }
    }

private:
    static void onBackClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onBack) s_inst->m_cbs.onBack();
    }
    static void onStartClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onNmtStart) s_inst->m_cbs.onNmtStart(s_inst->m_info.nodeId);
    }
    static void onPreOpClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onNmtPreOp) s_inst->m_cbs.onNmtPreOp(s_inst->m_info.nodeId);
    }
    static void onStopClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onNmtStop) s_inst->m_cbs.onNmtStop(s_inst->m_info.nodeId);
    }
    static void onResetNodeClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onNmtResetNode) s_inst->m_cbs.onNmtResetNode(s_inst->m_info.nodeId);
    }
    static void onResetCommClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onNmtResetComm) s_inst->m_cbs.onNmtResetComm(s_inst->m_info.nodeId);
    }

    static void onSetNodeIdClicked(lv_event_t*) {
        if (!s_inst) return;
        if (!s_inst->m_cbs.onSetNodeId) return;
        if (!s_inst->m_taNodeId) return;

        const char* t = lv_textarea_get_text(s_inst->m_taNodeId);
        int v = t ? atoi(t) : 0;
        if (v < 1) v = 1;
        if (v > 127) v = 127;
        s_inst->m_cbs.onSetNodeId(s_inst->m_info.nodeId, (uint8_t)v);
    }

    lv_obj_t* makeBtn(const char* text, lv_color_t bg, int x, int y, int w, int h,
                      lv_event_cb_t cb) {
        lv_obj_t* b = lv_btn_create(m_screen);
        lv_obj_set_size(b, w, h);
        lv_obj_set_pos(b, x, y);
        lv_obj_set_style_bg_color(b, bg, 0);
        lv_obj_set_style_radius(b, 10, 0);
        lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* l = lv_label_create(b);
        lv_label_set_text(l, text);
        lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_18, 0);
        lv_obj_center(l);
        return b;
    }

    lv_obj_t* m_screen = nullptr;
    NodeDetail_Info m_info;
    NodeDetail_Callbacks m_cbs;

    lv_obj_t* m_lblNode = nullptr;
    lv_obj_t* m_lblState = nullptr;
    lv_obj_t* m_lblLast = nullptr;

    lv_obj_t* m_btnStart = nullptr;
    lv_obj_t* m_btnPreOp = nullptr;
    lv_obj_t* m_btnStop  = nullptr;
    lv_obj_t* m_btnResetNode = nullptr;
    lv_obj_t* m_btnResetComm = nullptr;

    // Config widgets
    lv_obj_t* m_taNodeId = nullptr;
    lv_obj_t* m_btnSetNodeId = nullptr;
    lv_obj_t* m_kb = nullptr;

    inline static NodeDetailUI* s_inst = nullptr;
};
