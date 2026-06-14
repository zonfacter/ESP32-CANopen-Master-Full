/**
 * @file dunker_ui.h
 * @brief LVGL-Seite fuer Dunkermotoren DS402-Antriebe (Milestone 4, v1)
 *
 * Zeigt:
 *   - Statusword (0x6041) roh + dekodierter CiA-402-State
 *   - Flags: Fault / Operation enabled / Warning
 *   - Letztes gesendetes Controlword
 *
 * Aktionen (Controlword 0x6040):
 *   - FAULT RESET (0x0080)
 *   - SHUTDOWN (0x0006)
 *   - SWITCH ON (0x0007)
 *   - ENABLE OP (0x000F)
 *   - DISABLE VOLTAGE (0x0000)
 *
 * SPEICHERN ALS: src/ui/dunker_ui.h
 */

#pragma once

#include <Arduino.h>
#include <lvgl.h>
#include <functional>

#include "../devices/dunker_drive.h"

enum class DunkerUiCmd : uint8_t {
    FaultReset,
    Shutdown,
    SwitchOn,
    EnableOperation,
    DisableVoltage,
};

struct Dunker_Callbacks {
    std::function<void()>            onBack    = nullptr;
    std::function<void(DunkerUiCmd)> onCommand = nullptr;
};

class DunkerUI {
public:
    void setCallbacks(const Dunker_Callbacks& cbs) { m_cbs = cbs; }
    lv_obj_t* screen() const { return m_screen; }

    void create(uint8_t nodeId) {
        m_nodeId = nodeId;

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
        char tbuf[48];
        snprintf(tbuf, sizeof(tbuf), "Dunker DS402 - Node %u", (unsigned)nodeId);
        lv_label_set_text(title, tbuf);
        lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_LEFT_MID, 15, 0);

        lv_obj_t* btnBack = lv_btn_create(hdr);
        lv_obj_set_size(btnBack, 110, 40);
        lv_obj_align(btnBack, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(btnBack, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(btnBack, 8, 0);
        lv_obj_add_event_cb(btnBack, DunkerUI::onBackClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* backLbl = lv_label_create(btnBack);
        lv_label_set_text(backLbl, LV_SYMBOL_LEFT "  Zurueck");
        lv_obj_set_style_text_color(backLbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(backLbl);

        // Status card
        lv_obj_t* card = lv_obj_create(m_screen);
        lv_obj_set_size(card, 780, 170);
        lv_obj_set_pos(card, 10, 80);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x12122A), 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        m_lblStatusword = lv_label_create(card);
        lv_label_set_text(m_lblStatusword, "Statusword 6041h: ---");
        lv_obj_set_style_text_color(m_lblStatusword, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(m_lblStatusword, &lv_font_montserrat_18, 0);
        lv_obj_set_pos(m_lblStatusword, 14, 12);

        m_lblState = lv_label_create(card);
        lv_label_set_text(m_lblState, "State: ---");
        lv_obj_set_style_text_color(m_lblState, lv_color_hex(0xFFBB33), 0);
        lv_obj_set_style_text_font(m_lblState, &lv_font_montserrat_24, 0);
        lv_obj_set_pos(m_lblState, 14, 48);

        m_lblFlags = lv_label_create(card);
        lv_label_set_text(m_lblFlags, "Fault: -   OpEnabled: -   Warning: -");
        lv_obj_set_style_text_color(m_lblFlags, lv_color_hex(0xCCCCCC), 0);
        lv_obj_set_style_text_font(m_lblFlags, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblFlags, 14, 100);

        m_lblCw = lv_label_create(card);
        lv_label_set_text(m_lblCw, "Last Controlword: ---");
        lv_obj_set_style_text_color(m_lblCw, lv_color_hex(0x8888AA), 0);
        lv_obj_set_style_text_font(m_lblCw, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblCw, 14, 136);

        // Action buttons (one Controlword each)
        const int y = 270;
        const int w = 185;
        const int h = 80;
        makeBtn("FAULT\nRESET", lv_color_hex(0xB23A48), 10,  y, w, h, DunkerUI::onFaultResetClicked);
        makeBtn("SHUTDOWN",     lv_color_hex(0x666666), 205, y, w, h, DunkerUI::onShutdownClicked);
        makeBtn("SWITCH ON",    lv_color_hex(0xCC6600), 400, y, w, h, DunkerUI::onSwitchOnClicked);
        makeBtn("ENABLE\nOP",   lv_color_hex(0x007744), 595, y, w, h, DunkerUI::onEnableOpClicked);

        // Second row: safe stop
        makeBtn("DISABLE VOLTAGE", lv_color_hex(0x444444), 10, y + h + 15, 380, 60,
                DunkerUI::onDisableVoltageClicked);

        lv_obj_t* hint = lv_label_create(m_screen);
        lv_label_set_text(hint,
            "DS402-Reihenfolge: FAULT RESET -> SHUTDOWN -> SWITCH ON -> ENABLE OP");
        lv_obj_set_style_text_color(hint, lv_color_hex(0xFFBB33), 0);
        lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(hint, 410, 430);

        s_inst = this;
    }

    void load() {
        if (m_screen) lv_scr_load_anim(m_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }

    void update(const DunkerData& d) {
        if (m_lblStatusword) {
            char b[48];
            if (d.statuswordValid) snprintf(b, sizeof(b), "Statusword 6041h: 0x%04X", (unsigned)d.statusword);
            else                   snprintf(b, sizeof(b), "Statusword 6041h: ---");
            lv_label_set_text(m_lblStatusword, b);
        }
        if (m_lblState) {
            char b[48];
            snprintf(b, sizeof(b), "State: %s", DunkerDrive::stateName(d.state));
            lv_label_set_text(m_lblState, b);
            uint32_t col = 0xFFBB33; // default amber
            if (d.fault || d.state == DS402State::FaultReactionActive) col = 0xFF4444; // red
            else if (d.operationEnabled)                               col = 0x33DD66; // green
            lv_obj_set_style_text_color(m_lblState, lv_color_hex(col), 0);
        }
        if (m_lblFlags) {
            char b[64];
            snprintf(b, sizeof(b), "Fault: %s   OpEnabled: %s   Warning: %s",
                     d.fault ? "YES" : "no",
                     d.operationEnabled ? "YES" : "no",
                     d.warning ? "YES" : "no");
            lv_label_set_text(m_lblFlags, b);
        }
        if (m_lblCw) {
            char b[48];
            snprintf(b, sizeof(b), "Last Controlword: 0x%04X  | %s",
                     (unsigned)d.lastControlword, d.connected ? "online" : "OFFLINE");
            lv_label_set_text(m_lblCw, b);
        }
    }

private:
    lv_obj_t* makeBtn(const char* text, lv_color_t bg, int x, int y, int w, int h, lv_event_cb_t cb) {
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

    static void emit(DunkerUiCmd c) {
        if (s_inst && s_inst->m_cbs.onCommand) s_inst->m_cbs.onCommand(c);
    }

    static void onBackClicked(lv_event_t*)          { if (s_inst && s_inst->m_cbs.onBack) s_inst->m_cbs.onBack(); }
    static void onFaultResetClicked(lv_event_t*)    { emit(DunkerUiCmd::FaultReset); }
    static void onShutdownClicked(lv_event_t*)      { emit(DunkerUiCmd::Shutdown); }
    static void onSwitchOnClicked(lv_event_t*)      { emit(DunkerUiCmd::SwitchOn); }
    static void onEnableOpClicked(lv_event_t*)      { emit(DunkerUiCmd::EnableOperation); }
    static void onDisableVoltageClicked(lv_event_t*){ emit(DunkerUiCmd::DisableVoltage); }

    Dunker_Callbacks m_cbs;
    uint8_t m_nodeId = 0;

    lv_obj_t* m_screen = nullptr;
    lv_obj_t* m_lblStatusword = nullptr;
    lv_obj_t* m_lblState = nullptr;
    lv_obj_t* m_lblFlags = nullptr;
    lv_obj_t* m_lblCw = nullptr;

    inline static DunkerUI* s_inst = nullptr;
};
