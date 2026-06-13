/**
 * @file ap04_ui.h
 * @brief Minimal-LVGL UI für SIKO AP04 Test
 * @date 2026
 */

#pragma once

#include <Arduino.h>
#include <lvgl.h>

#include "../devices/siko_ap04.h"
#include <functional>

struct AP04UI_Callbacks {
    std::function<void()> onBack = nullptr;
};

class AP04_UI {
public:
    AP04_UI() = default;

    void setAP04(SikoAP04* dev) { m_dev = dev; }
    void setCallbacks(const AP04UI_Callbacks& cbs) { m_cbs = cbs; }

    void create(uint8_t nodeId)
    {
        (void)nodeId;
        m_screen = lv_obj_create(NULL);
        lv_obj_clear_flag(m_screen, LV_OBJ_FLAG_SCROLLABLE);
        lv_scr_load(m_screen);

        // Titel + Back
        lv_obj_t* title = lv_label_create(m_screen);
        lv_label_set_text(title, "SIKO AP04 – CANopen");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

        lv_obj_t* btnBack = lv_btn_create(m_screen);
        lv_obj_set_size(btnBack, 120, 46);
        lv_obj_align(btnBack, LV_ALIGN_TOP_LEFT, 10, 10);
        lv_obj_add_event_cb(btnBack, AP04_UI::onBackClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* backLbl = lv_label_create(btnBack);
        lv_label_set_text(backLbl, LV_SYMBOL_LEFT " Zurueck");
        lv_obj_center(backLbl);

        s_inst = this;

        // Connection
        m_lblConn = lv_label_create(m_screen);
        lv_label_set_text(m_lblConn, "Verbunden: --- | Operational: ---");
        lv_obj_set_style_text_font(m_lblConn, &lv_font_montserrat_18, 0);
        lv_obj_align(m_lblConn, LV_ALIGN_TOP_MID, 0, 50);

        // Position groß
        m_lblPos = lv_label_create(m_screen);
        lv_label_set_text(m_lblPos, "0");
        lv_obj_set_style_text_font(m_lblPos, &lv_font_montserrat_48, 0);
        lv_obj_align(m_lblPos, LV_ALIGN_CENTER, 0, -10);

        // Statusbyte / Updates
        m_lblMeta = lv_label_create(m_screen);
        lv_label_set_text(m_lblMeta, "Status: 0x00 | Updates: 0");
        lv_obj_set_style_text_font(m_lblMeta, &lv_font_montserrat_18, 0);
        lv_obj_align(m_lblMeta, LV_ALIGN_BOTTOM_MID, 0, -20);
    }

    void update(const AP04Data& d)
    {
        if (!m_screen) return;

        // Verbunden/Operational
        char bufConn[96];
        snprintf(bufConn, sizeof(bufConn), "Verbunden: %s | Operational: %s",
                 d.connected ? "JA" : "NEIN",
                 d.operational ? "JA" : "NEIN");
        lv_label_set_text(m_lblConn, bufConn);

        // Position
        char bufPos[32];
        snprintf(bufPos, sizeof(bufPos), "%.1f", d.positionMm);
        lv_label_set_text(m_lblPos, bufPos);

        // Meta
        char bufMeta[96];
        snprintf(bufMeta, sizeof(bufMeta), "Status: 0x%02X | Updates: %lu",
                 (unsigned)d.statusByte, (unsigned long)d.updateCount);
        lv_label_set_text(m_lblMeta, bufMeta);
    }

private:
    static void onBackClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onBack) s_inst->m_cbs.onBack();
    }

    SikoAP04* m_dev = nullptr;
    AP04UI_Callbacks m_cbs;
    inline static AP04_UI* s_inst = nullptr;

    lv_obj_t* m_screen = nullptr;
    lv_obj_t* m_lblConn = nullptr;
    lv_obj_t* m_lblPos  = nullptr;
    lv_obj_t* m_lblMeta = nullptr;
};
