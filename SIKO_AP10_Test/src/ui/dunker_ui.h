/**
 * @file dunker_ui.h
 * @brief LVGL-Seite fuer Dunkermotoren DS402-Antriebe (Milestones 4-6)
 *
 * Tabview mit drei Reitern:
 *   - "DS402"  (M4): Statusword/State/Flags + Controlword-Kommandos
 *   - "Motion" (M5): Mode-Wahl, Istwerte, Zielposition + Profilgeschwindigkeit,
 *                    GO (New-Setpoint), Halt, Jog +/- (Profile Velocity)
 *   - "I/O"    (M6): Digitale Eingaenge (60FD), Ausgaenge (60FE:01),
 *                    Bremse (herstellerspezifisch - Objekt aus EDS)
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
    Halt,
};

struct Dunker_Callbacks {
    std::function<void()>                  onBack      = nullptr;
    std::function<void(DunkerUiCmd)>       onCommand   = nullptr; // M4 controlword
    std::function<void(int8_t mode)>       onSetMode   = nullptr; // M5
    std::function<void(int32_t, uint32_t)> onGoto      = nullptr; // M5 (pos, vel)
    std::function<void(int, uint32_t)>     onJog       = nullptr; // M5 (dir, speed)
    std::function<void(uint8_t, bool)>     onSetOutput = nullptr; // M6 (bit, on)
    std::function<void(bool)>              onBrake     = nullptr; // M6 (release)
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
        lv_obj_set_size(hdr, 800, 56);
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
        lv_obj_set_size(btnBack, 110, 38);
        lv_obj_align(btnBack, LV_ALIGN_RIGHT_MID, -10, 0);
        lv_obj_set_style_bg_color(btnBack, lv_color_hex(0x444444), 0);
        lv_obj_set_style_radius(btnBack, 8, 0);
        lv_obj_add_event_cb(btnBack, DunkerUI::onBackClicked, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* backLbl = lv_label_create(btnBack);
        lv_label_set_text(backLbl, LV_SYMBOL_LEFT "  Zurueck");
        lv_obj_set_style_text_color(backLbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(backLbl);

        // Tabview
        lv_obj_t* tv = lv_tabview_create(m_screen, LV_DIR_TOP, 44);
        lv_obj_set_pos(tv, 0, 56);
        lv_obj_set_size(tv, 800, 424);
        lv_obj_set_style_bg_color(tv, lv_color_hex(0x0D0D1A), 0);

        lv_obj_t* tabDs   = lv_tabview_add_tab(tv, "DS402");
        lv_obj_t* tabMot  = lv_tabview_add_tab(tv, "Motion");
        lv_obj_t* tabIo   = lv_tabview_add_tab(tv, "I/O");

        buildDs402Tab(tabDs);
        buildMotionTab(tabMot);
        buildIoTab(tabIo);

        s_inst = this;
    }

    void load() {
        if (m_screen) lv_scr_load_anim(m_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }

    void update(const DunkerData& d) {
        char b[80];

        // --- DS402 tab ---
        if (m_lblStatusword) {
            if (d.statuswordValid) snprintf(b, sizeof(b), "Statusword 6041h: 0x%04X", (unsigned)d.statusword);
            else                   snprintf(b, sizeof(b), "Statusword 6041h: ---");
            lv_label_set_text(m_lblStatusword, b);
        }
        if (m_lblState) {
            snprintf(b, sizeof(b), "State: %s", DunkerDrive::stateName(d.state));
            lv_label_set_text(m_lblState, b);
            uint32_t col = 0xFFBB33;
            if (d.fault || d.state == DS402State::FaultReactionActive) col = 0xFF4444;
            else if (d.operationEnabled)                               col = 0x33DD66;
            lv_obj_set_style_text_color(m_lblState, lv_color_hex(col), 0);
        }
        if (m_lblFlags) {
            snprintf(b, sizeof(b), "Fault: %s   OpEnabled: %s   Warning: %s   | %s",
                     d.fault ? "YES" : "no",
                     d.operationEnabled ? "YES" : "no",
                     d.warning ? "YES" : "no",
                     d.connected ? "online" : "OFFLINE");
            lv_label_set_text(m_lblFlags, b);
        }
        if (m_lblCw) {
            snprintf(b, sizeof(b), "Last Controlword: 0x%04X", (unsigned)d.lastControlword);
            lv_label_set_text(m_lblCw, b);
        }

        // --- Motion tab ---
        if (m_lblMode) {
            if (d.modeValid) snprintf(b, sizeof(b), "Mode 6061h: %d (%s)", (int)d.mode, DunkerDrive::modeName(d.mode));
            else             snprintf(b, sizeof(b), "Mode 6061h: ---");
            lv_label_set_text(m_lblMode, b);
        }
        if (m_lblActPos) {
            if (d.posValid) snprintf(b, sizeof(b), "Act. Pos 6064h: %ld", (long)d.positionActual);
            else            snprintf(b, sizeof(b), "Act. Pos 6064h: ---");
            lv_label_set_text(m_lblActPos, b);
        }
        if (m_lblActVel) {
            if (d.velValid) snprintf(b, sizeof(b), "Act. Vel 606Ch: %ld", (long)d.velocityActual);
            else            snprintf(b, sizeof(b), "Act. Vel 606Ch: ---");
            lv_label_set_text(m_lblActVel, b);
        }

        // --- I/O tab ---
        if (m_lblDi) {
            if (d.diValid) snprintf(b, sizeof(b), "Dig. Inputs 60FDh: 0x%08lX", (unsigned long)d.digitalInputs);
            else           snprintf(b, sizeof(b), "Dig. Inputs 60FDh: ---");
            lv_label_set_text(m_lblDi, b);
        }
        if (m_lblBrake) {
            snprintf(b, sizeof(b), "Bremse: %s",
                     d.brakeConfigured ? "Objekt konfiguriert" : "nicht konfiguriert (EDS noetig)");
            lv_label_set_text(m_lblBrake, b);
        }
    }

private:
    // ---------------- Tab builders ----------------
    void buildDs402Tab(lv_obj_t* t) {
        m_lblStatusword = mkLabel(t, "Statusword 6041h: ---", 8, 8, 0xFFFFFF, &lv_font_montserrat_18);
        m_lblState      = mkLabel(t, "State: ---",            8, 44, 0xFFBB33, &lv_font_montserrat_24);
        m_lblFlags      = mkLabel(t, "Fault: -  OpEnabled: -  Warning: -", 8, 92, 0xCCCCCC, &lv_font_montserrat_14);
        m_lblCw         = mkLabel(t, "Last Controlword: ---", 8, 120, 0x8888AA, &lv_font_montserrat_14);

        const int y = 160; const int w = 150; const int h = 64;
        mkBtn(t, "FAULT\nRESET", 0xB23A48, 8,   y, w, h, DunkerUI::onFaultResetClicked);
        mkBtn(t, "SHUTDOWN",     0x666666, 166, y, w, h, DunkerUI::onShutdownClicked);
        mkBtn(t, "SWITCH ON",    0xCC6600, 324, y, w, h, DunkerUI::onSwitchOnClicked);
        mkBtn(t, "ENABLE\nOP",   0x007744, 482, y, w, h, DunkerUI::onEnableOpClicked);
        mkBtn(t, "HALT",         0x884444, 8,   y + h + 12, w, 48, DunkerUI::onHaltClicked);
        mkBtn(t, "DISABLE VOLT", 0x444444, 166, y + h + 12, 300, 48, DunkerUI::onDisableVoltageClicked);
    }

    void buildMotionTab(lv_obj_t* t) {
        m_lblMode   = mkLabel(t, "Mode 6061h: ---",     8,   8, 0xFFBB33, &lv_font_montserrat_18);
        m_lblActPos = mkLabel(t, "Act. Pos 6064h: ---", 400, 8, 0xFFFFFF, &lv_font_montserrat_18);
        m_lblActVel = mkLabel(t, "Act. Vel 606Ch: ---", 400, 40, 0xCCCCCC, &lv_font_montserrat_14);

        // Mode buttons
        mkBtn(t, "PP",     0x0066CC, 8,   44, 110, 44, DunkerUI::onModePpClicked);
        mkBtn(t, "PV",     0x0066CC, 128, 44, 110, 44, DunkerUI::onModePvClicked);
        mkBtn(t, "HOMING", 0x0066CC, 248, 44, 120, 44, DunkerUI::onModeHomeClicked);

        // Target position + velocity inputs
        mkLabel(t, "Target:", 8, 108, 0xFFFFFF, &lv_font_montserrat_14);
        m_taTarget = mkTextArea(t, "0", "0123456789-", 80, 102, 180);
        mkLabel(t, "Vel:", 280, 108, 0xFFFFFF, &lv_font_montserrat_14);
        m_taVel = mkTextArea(t, "0", "0123456789", 330, 102, 150);

        mkBtn(t, "GO", 0x007744, 500, 100, 130, 50, DunkerUI::onGoClicked);
        mkBtn(t, "STOP", 0x884444, 640, 100, 120, 50, DunkerUI::onHaltClicked);

        // Jog (momentary: press = move, release = stop)
        lv_obj_t* jm = mkBtn(t, "JOG -", 0xCC6600, 8,   170, 175, 70, nullptr);
        lv_obj_add_event_cb(jm, DunkerUI::onJogMinus, LV_EVENT_PRESSED,  nullptr);
        lv_obj_add_event_cb(jm, DunkerUI::onJogStop,  LV_EVENT_RELEASED, nullptr);
        lv_obj_t* jp = mkBtn(t, "JOG +", 0xCC6600, 193, 170, 175, 70, nullptr);
        lv_obj_add_event_cb(jp, DunkerUI::onJogPlus, LV_EVENT_PRESSED,  nullptr);
        lv_obj_add_event_cb(jp, DunkerUI::onJogStop, LV_EVENT_RELEASED, nullptr);

        // Shared numeric keyboard (overlay on the screen, hidden until focus)
        m_kb = lv_keyboard_create(m_screen);
        lv_keyboard_set_mode(m_kb, LV_KEYBOARD_MODE_NUMBER);
        lv_obj_set_size(m_kb, 800, 240);
        lv_obj_align(m_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_add_flag(m_kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(m_kb, DunkerUI::onKbReady, LV_EVENT_READY,  nullptr);
        lv_obj_add_event_cb(m_kb, DunkerUI::onKbReady, LV_EVENT_CANCEL, nullptr);

        lv_obj_add_event_cb(m_taTarget, DunkerUI::onTaFocused, LV_EVENT_FOCUSED, nullptr);
        lv_obj_add_event_cb(m_taVel,    DunkerUI::onTaFocused, LV_EVENT_FOCUSED, nullptr);
    }

    void buildIoTab(lv_obj_t* t) {
        m_lblDi = mkLabel(t, "Dig. Inputs 60FDh: ---", 8, 8, 0xFFFFFF, &lv_font_montserrat_18);

        mkLabel(t, "Outputs 60FE:01", 8, 48, 0xCCCCCC, &lv_font_montserrat_14);
        for (uint8_t bit = 0; bit < 4; bit++) {
            char lbl[8];
            snprintf(lbl, sizeof(lbl), "OUT%u", (unsigned)bit);
            // Proven pattern: pass the bit index via the event user-data (no
            // dependency on lv_obj_set_user_data / lv_obj_has_state).
            lv_obj_t* b = lv_btn_create(t);
            lv_obj_set_size(b, 110, 54);
            lv_obj_set_pos(b, 8 + bit * 120, 76);
            lv_obj_set_style_bg_color(b, lv_color_hex(0x555577), 0);
            lv_obj_set_style_radius(b, 10, 0);
            lv_obj_add_event_cb(b, DunkerUI::onOutputClicked, LV_EVENT_CLICKED, (void*)(uintptr_t)bit);
            lv_obj_t* l = lv_label_create(b);
            lv_label_set_text(l, lbl);
            lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_text_font(l, &lv_font_montserrat_18, 0);
            lv_obj_center(l);
            m_outBtn[bit] = b;
        }

        mkLabel(t, "Bremse (herstellerspezifisch - Objekt aus EDS)", 8, 150, 0xFFBB33, &lv_font_montserrat_14);
        mkBtn(t, "BRAKE RELEASE", 0x007744, 8,   176, 200, 60, DunkerUI::onBrakeReleaseClicked);
        mkBtn(t, "BRAKE ENGAGE",  0x884444, 218, 176, 200, 60, DunkerUI::onBrakeEngageClicked);
        m_lblBrake = mkLabel(t, "Bremse: ---", 8, 246, 0x8888AA, &lv_font_montserrat_14);
    }

    // ---------------- Widget helpers ----------------
    lv_obj_t* mkLabel(lv_obj_t* parent, const char* txt, int x, int y, uint32_t color, const lv_font_t* font) {
        lv_obj_t* l = lv_label_create(parent);
        lv_label_set_text(l, txt);
        lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
        lv_obj_set_style_text_font(l, font, 0);
        lv_obj_set_pos(l, x, y);
        return l;
    }

    lv_obj_t* mkBtn(lv_obj_t* parent, const char* text, uint32_t bg, int x, int y, int w, int h, lv_event_cb_t cb) {
        lv_obj_t* b = lv_btn_create(parent);
        lv_obj_set_size(b, w, h);
        lv_obj_set_pos(b, x, y);
        lv_obj_set_style_bg_color(b, lv_color_hex(bg), 0);
        lv_obj_set_style_radius(b, 10, 0);
        if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, nullptr);
        lv_obj_t* l = lv_label_create(b);
        lv_label_set_text(l, text);
        lv_obj_set_style_text_color(l, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_18, 0);
        lv_obj_center(l);
        return b;
    }

    lv_obj_t* mkTextArea(lv_obj_t* parent, const char* init, const char* accepted, int x, int y, int w) {
        lv_obj_t* ta = lv_textarea_create(parent);
        lv_obj_set_size(ta, w, 46);
        lv_obj_set_pos(ta, x, y);
        lv_textarea_set_one_line(ta, true);
        lv_textarea_set_accepted_chars(ta, accepted);
        lv_textarea_set_max_length(ta, 11);
        lv_textarea_set_text(ta, init);
        return ta;
    }

    static long taLong(lv_obj_t* ta) {
        if (!ta) return 0;
        const char* s = lv_textarea_get_text(ta);
        return s ? atol(s) : 0;
    }

    // ---------------- Event handlers ----------------
    static void emit(DunkerUiCmd c) { if (s_inst && s_inst->m_cbs.onCommand) s_inst->m_cbs.onCommand(c); }

    static void onBackClicked(lv_event_t*)           { if (s_inst && s_inst->m_cbs.onBack) s_inst->m_cbs.onBack(); }
    static void onFaultResetClicked(lv_event_t*)     { emit(DunkerUiCmd::FaultReset); }
    static void onShutdownClicked(lv_event_t*)       { emit(DunkerUiCmd::Shutdown); }
    static void onSwitchOnClicked(lv_event_t*)       { emit(DunkerUiCmd::SwitchOn); }
    static void onEnableOpClicked(lv_event_t*)       { emit(DunkerUiCmd::EnableOperation); }
    static void onDisableVoltageClicked(lv_event_t*) { emit(DunkerUiCmd::DisableVoltage); }
    static void onHaltClicked(lv_event_t*)           { emit(DunkerUiCmd::Halt); }

    static void onModePpClicked(lv_event_t*)   { if (s_inst && s_inst->m_cbs.onSetMode) s_inst->m_cbs.onSetMode(DS402_MODE::PROFILE_POSITION); }
    static void onModePvClicked(lv_event_t*)   { if (s_inst && s_inst->m_cbs.onSetMode) s_inst->m_cbs.onSetMode(DS402_MODE::PROFILE_VELOCITY); }
    static void onModeHomeClicked(lv_event_t*) { if (s_inst && s_inst->m_cbs.onSetMode) s_inst->m_cbs.onSetMode(DS402_MODE::HOMING); }

    static void onGoClicked(lv_event_t*) {
        if (!s_inst || !s_inst->m_cbs.onGoto) return;
        const int32_t  pos = (int32_t)taLong(s_inst->m_taTarget);
        const uint32_t vel = (uint32_t)taLong(s_inst->m_taVel);
        s_inst->m_cbs.onGoto(pos, vel);
    }

    static void onJogPlus(lv_event_t*)  { jog(+1); }
    static void onJogMinus(lv_event_t*) { jog(-1); }
    static void onJogStop(lv_event_t*)  { jog(0); }
    static void jog(int dir) {
        if (!s_inst || !s_inst->m_cbs.onJog) return;
        const uint32_t speed = (uint32_t)taLong(s_inst->m_taVel);
        s_inst->m_cbs.onJog(dir, speed);
    }

    static void onOutputClicked(lv_event_t* ev) {
        if (!s_inst || !s_inst->m_cbs.onSetOutput) return;
        const uint8_t bit = (uint8_t)(uintptr_t)lv_event_get_user_data(ev);
        if (bit >= 4) return;
        s_inst->m_outState[bit] = !s_inst->m_outState[bit];
        const bool on = s_inst->m_outState[bit];
        if (s_inst->m_outBtn[bit]) {
            lv_obj_set_style_bg_color(s_inst->m_outBtn[bit], lv_color_hex(on ? 0x00AA44 : 0x555577), 0);
        }
        s_inst->m_cbs.onSetOutput(bit, on);
    }

    static void onBrakeReleaseClicked(lv_event_t*) { if (s_inst && s_inst->m_cbs.onBrake) s_inst->m_cbs.onBrake(true); }
    static void onBrakeEngageClicked(lv_event_t*)  { if (s_inst && s_inst->m_cbs.onBrake) s_inst->m_cbs.onBrake(false); }

    // Keyboard show/hide
    static void onTaFocused(lv_event_t* ev) {
        if (!s_inst || !s_inst->m_kb) return;
        lv_obj_t* ta = lv_event_get_target(ev);
        lv_keyboard_set_textarea(s_inst->m_kb, ta);
        lv_obj_clear_flag(s_inst->m_kb, LV_OBJ_FLAG_HIDDEN);
    }
    static void onKbReady(lv_event_t*) {
        if (!s_inst || !s_inst->m_kb) return;
        lv_obj_add_flag(s_inst->m_kb, LV_OBJ_FLAG_HIDDEN);
    }

    // ---------------- State ----------------
    Dunker_Callbacks m_cbs;
    uint8_t m_nodeId = 0;

    lv_obj_t* m_screen = nullptr;

    // DS402 tab
    lv_obj_t* m_lblStatusword = nullptr;
    lv_obj_t* m_lblState = nullptr;
    lv_obj_t* m_lblFlags = nullptr;
    lv_obj_t* m_lblCw = nullptr;

    // Motion tab
    lv_obj_t* m_lblMode = nullptr;
    lv_obj_t* m_lblActPos = nullptr;
    lv_obj_t* m_lblActVel = nullptr;
    lv_obj_t* m_taTarget = nullptr;
    lv_obj_t* m_taVel = nullptr;
    lv_obj_t* m_kb = nullptr;

    // I/O tab
    lv_obj_t* m_lblDi = nullptr;
    lv_obj_t* m_lblBrake = nullptr;
    lv_obj_t* m_outBtn[4] = { nullptr, nullptr, nullptr, nullptr };
    bool      m_outState[4] = { false, false, false, false };

    inline static DunkerUI* s_inst = nullptr;
};
