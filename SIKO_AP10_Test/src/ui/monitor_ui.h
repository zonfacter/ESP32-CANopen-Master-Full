/**
 * @file monitor_ui.h
 * @brief Live CAN traffic monitor (LVGL) - renders the SnifferManager buffer.
 *
 * Decoupled from SnifferManager: the app builds a text log (newest first) and
 * pushes it via setLog(); this screen just displays it + offers Sniffer ON/OFF
 * and Clear. Reachable from the start menu.
 *
 * SPEICHERN ALS: src/ui/monitor_ui.h
 */

#pragma once

#include <Arduino.h>
#include <lvgl.h>
#include <functional>

struct Monitor_Callbacks {
    std::function<void()>     onBack          = nullptr;
    std::function<void(bool)> onToggleSniffer = nullptr;
    std::function<void()>     onClear         = nullptr;
};

class MonitorUI {
public:
    void setCallbacks(const Monitor_Callbacks& cbs) { m_cbs = cbs; }
    lv_obj_t* screen() const { return m_screen; }

    void create() {
        m_screen = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(m_screen, lv_color_hex(0x0D0D1A), 0);
        lv_obj_set_style_bg_opa(m_screen, LV_OPA_COVER, 0);
        lv_obj_clear_flag(m_screen, LV_OBJ_FLAG_SCROLLABLE);

        // Header
        lv_obj_t* hdr = lv_obj_create(m_screen);
        lv_obj_set_size(hdr, 800, 56);
        lv_obj_set_pos(hdr, 0, 0);
        lv_obj_set_style_bg_color(hdr, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(hdr, 0, 0);
        lv_obj_set_style_radius(hdr, 0, 0);
        lv_obj_clear_flag(hdr, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t* title = lv_label_create(hdr);
        lv_label_set_text(title, LV_SYMBOL_EYE_OPEN "  CAN Monitor - Live");
        lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_LEFT_MID, 15, 0);

        lv_obj_t* btnBack = lv_btn_create(hdr);
        lv_obj_set_size(btnBack, 110, 38);
        lv_obj_align(btnBack, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(btnBack, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(btnBack, 8, 0);
        lv_obj_add_event_cb(btnBack, MonitorUI::onBackClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* backLbl = lv_label_create(btnBack);
        lv_label_set_text(backLbl, LV_SYMBOL_LEFT "  Zurueck");
        lv_obj_set_style_text_color(backLbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(backLbl);

        // Controls row
        m_btnSniffer = lv_btn_create(m_screen);
        lv_obj_set_size(m_btnSniffer, 200, 44);
        lv_obj_set_pos(m_btnSniffer, 10, 66);
        lv_obj_set_style_radius(m_btnSniffer, 10, 0);
        lv_obj_add_event_cb(m_btnSniffer, MonitorUI::onSnifferClicked, LV_EVENT_CLICKED, nullptr);
        m_lblSniffer = lv_label_create(m_btnSniffer);
        lv_obj_set_style_text_color(m_lblSniffer, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(m_lblSniffer, &lv_font_montserrat_18, 0);
        lv_obj_center(m_lblSniffer);
        setSnifferUi(false);

        lv_obj_t* btnClear = lv_btn_create(m_screen);
        lv_obj_set_size(btnClear, 150, 44);
        lv_obj_set_pos(btnClear, 224, 66);
        lv_obj_set_style_bg_color(btnClear, lv_color_hex(0x884444), 0);
        lv_obj_set_style_radius(btnClear, 10, 0);
        lv_obj_add_event_cb(btnClear, MonitorUI::onClearClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* clr = lv_label_create(btnClear);
        lv_label_set_text(clr, LV_SYMBOL_TRASH "  Clear");
        lv_obj_set_style_text_color(clr, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(clr);

        m_lblCount = lv_label_create(m_screen);
        lv_label_set_text(m_lblCount, "0 Frames");
        lv_obj_set_style_text_color(m_lblCount, lv_color_hex(0xAAAAAA), 0);
        lv_obj_set_style_text_font(m_lblCount, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblCount, 390, 78);

        // Log container (scrollable both ways; lines do not wrap)
        lv_obj_t* cont = lv_obj_create(m_screen);
        lv_obj_set_size(cont, 780, 348);
        lv_obj_set_pos(cont, 10, 122);
        lv_obj_set_style_bg_color(cont, lv_color_hex(0x12122A), 0);
        lv_obj_set_style_border_color(cont, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(cont, 2, 0);
        lv_obj_set_style_radius(cont, 10, 0);
        lv_obj_set_style_pad_all(cont, 8, 0);
        lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_AUTO);

        m_lblLog = lv_label_create(cont);
        lv_label_set_text(m_lblLog, "(Sniffer aus - ON druecken)");
        lv_obj_set_style_text_color(m_lblLog, lv_color_hex(0xCCCCCC), 0);
        lv_obj_set_style_text_font(m_lblLog, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblLog, 0, 0);

        s_inst = this;
    }

    void load() {
        if (m_screen) lv_scr_load_anim(m_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }

    void setLog(const char* text, uint16_t count) {
        if (m_lblLog && text) lv_label_set_text(m_lblLog, text);
        if (m_lblCount) {
            char b[24];
            snprintf(b, sizeof(b), "%u Frames", (unsigned)count);
            lv_label_set_text(m_lblCount, b);
        }
    }

    void setSnifferUi(bool en) {
        m_sniffOn = en;
        if (!m_btnSniffer || !m_lblSniffer) return;
        lv_obj_set_style_bg_color(m_btnSniffer, en ? lv_color_hex(0xCC6600) : lv_color_hex(0x444444), 0);
        lv_label_set_text(m_lblSniffer, en ? (LV_SYMBOL_EYE_OPEN "  Sniffer: ON")
                                           : (LV_SYMBOL_EYE_CLOSE "  Sniffer: OFF"));
    }

private:
    static void onBackClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onBack) s_inst->m_cbs.onBack();
    }
    static void onSnifferClicked(lv_event_t*) {
        if (!s_inst) return;
        s_inst->setSnifferUi(!s_inst->m_sniffOn);
        if (s_inst->m_cbs.onToggleSniffer) s_inst->m_cbs.onToggleSniffer(s_inst->m_sniffOn);
    }
    static void onClearClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onClear) s_inst->m_cbs.onClear();
    }

    Monitor_Callbacks m_cbs;
    lv_obj_t* m_screen = nullptr;
    lv_obj_t* m_btnSniffer = nullptr;
    lv_obj_t* m_lblSniffer = nullptr;
    lv_obj_t* m_lblCount = nullptr;
    lv_obj_t* m_lblLog = nullptr;
    bool m_sniffOn = false;

    inline static MonitorUI* s_inst = nullptr;
};
