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
    std::function<void(uint8_t, uint32_t)> onLssApply  = nullptr; // Cfg (new node-id, baud; baud 0 = keep)
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

        m_lblTitle = lv_label_create(hdr);
        char tbuf[48];
        snprintf(tbuf, sizeof(tbuf), "Dunker DS402 - Node %u", (unsigned)nodeId);
        lv_label_set_text(m_lblTitle, tbuf);
        lv_obj_set_style_text_color(m_lblTitle, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(m_lblTitle, &lv_font_montserrat_20, 0);
        lv_obj_align(m_lblTitle, LV_ALIGN_LEFT_MID, 15, 0);

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
        lv_obj_t* tabCfg  = lv_tabview_add_tab(tv, "Cfg");

        m_cfgNodeId = nodeId;   // default target = current node

        buildDs402Tab(tabDs);
        buildMotionTab(tabMot);
        buildIoTab(tabIo);
        buildCfgTab(tabCfg);

        s_inst = this;
    }

    // Called by the app after an LSS apply to show the result.
    void setLssStatus(const char* text, bool ok) {
        if (!m_lblLssStatus) return;
        lv_label_set_text(m_lblLssStatus, text);
        lv_obj_set_style_text_color(m_lblLssStatus, lv_color_hex(ok ? 0x33DD66 : 0xFF4444), 0);
    }

    void load() {
        if (m_screen) lv_scr_load_anim(m_screen, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
    }

    bool created() const { return m_screen != nullptr; }

    // Rebind the (already created) page to another node without recreating it.
    void setNode(uint8_t nodeId) {
        m_nodeId = nodeId;
        m_cfgNodeId = nodeId;
        if (m_lblTitle) {
            char tbuf[48];
            snprintf(tbuf, sizeof(tbuf), "Dunker DS402 - Node %u", (unsigned)nodeId);
            lv_label_set_text(m_lblTitle, tbuf);
        }
        refreshCfgLabels();
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
        // Scrollable form container; shrinks to the area above the keyboard on
        // focus (iPhone-style) so all controls stay reachable by scrolling.
        m_motCont = lv_obj_create(t);
        lv_obj_set_size(m_motCont, lv_pct(100), lv_pct(100));
        lv_obj_set_pos(m_motCont, 0, 0);
        lv_obj_set_style_bg_opa(m_motCont, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(m_motCont, 0, 0);
        lv_obj_set_style_radius(m_motCont, 0, 0);
        lv_obj_set_style_pad_all(m_motCont, 0, 0);
        lv_obj_set_scroll_dir(m_motCont, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(m_motCont, LV_SCROLLBAR_MODE_AUTO);
        lv_obj_t* c = m_motCont;

        m_lblMode   = mkLabel(c, "Mode 6061h: ---",     8,   8, 0xFFBB33, &lv_font_montserrat_18);
        m_lblActPos = mkLabel(c, "Act. Pos 6064h: ---", 400, 8, 0xFFFFFF, &lv_font_montserrat_18);
        m_lblActVel = mkLabel(c, "Act. Vel 606Ch: ---", 400, 40, 0xCCCCCC, &lv_font_montserrat_14);

        // Mode buttons
        mkBtn(c, "PP",     0x0066CC, 8,   44, 110, 44, DunkerUI::onModePpClicked);
        mkBtn(c, "PV",     0x0066CC, 128, 44, 110, 44, DunkerUI::onModePvClicked);
        mkBtn(c, "HOMING", 0x0066CC, 248, 44, 120, 44, DunkerUI::onModeHomeClicked);

        // Target position + velocity inputs
        mkLabel(c, "Target:", 8, 108, 0xFFFFFF, &lv_font_montserrat_14);
        m_taTarget = mkTextArea(c, "0", "0123456789-", 80, 102, 180);
        mkLabel(c, "Vel:", 280, 108, 0xFFFFFF, &lv_font_montserrat_14);
        m_taVel = mkTextArea(c, "0", "0123456789", 330, 102, 150);

        mkBtn(c, "GO", 0x007744, 500, 100, 130, 50, DunkerUI::onGoClicked);
        mkBtn(c, "STOP", 0x884444, 640, 100, 120, 50, DunkerUI::onHaltClicked);

        // Jog (momentary: press = move, release = stop)
        lv_obj_t* jm = mkBtn(c, "JOG -", 0xCC6600, 8,   170, 175, 70, nullptr);
        lv_obj_add_event_cb(jm, DunkerUI::onJogMinus, LV_EVENT_PRESSED,  nullptr);
        lv_obj_add_event_cb(jm, DunkerUI::onJogStop,  LV_EVENT_RELEASED, nullptr);
        lv_obj_t* jp = mkBtn(c, "JOG +", 0xCC6600, 193, 170, 175, 70, nullptr);
        lv_obj_add_event_cb(jp, DunkerUI::onJogPlus, LV_EVENT_PRESSED,  nullptr);
        lv_obj_add_event_cb(jp, DunkerUI::onJogStop, LV_EVENT_RELEASED, nullptr);

        // Shared numeric keyboard (overlay on the screen, hidden until focus)
        m_kb = lv_keyboard_create(m_screen);
        lv_keyboard_set_mode(m_kb, LV_KEYBOARD_MODE_NUMBER);
        lv_obj_set_size(m_kb, 800, KB_HEIGHT);
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

    void buildCfgTab(lv_obj_t* t) {
        mkLabel(t, "LSS Node-ID / Baud (CiA 305)", 8, 8, 0xFFFFFF, &lv_font_montserrat_18);

        // Node-ID stepper (-/+), no keyboard needed
        mkLabel(t, "Node-ID:", 8, 60, 0xCCCCCC, &lv_font_montserrat_14);
        mkBtn(t, "-", 0x555577, 110, 48, 64, 56, DunkerUI::onCfgNodeMinus);
        m_lblCfgNode = mkLabel(t, "1", 198, 58, 0xFFBB33, &lv_font_montserrat_24);
        mkBtn(t, "+", 0x555577, 258, 48, 64, 56, DunkerUI::onCfgNodePlus);

        // Target baud (toggle; none selected = keep current baud)
        mkLabel(t, "Baud:", 8, 128, 0xCCCCCC, &lv_font_montserrat_14);
        m_btnCfgB125 = mkBtn(t, "125k", 0x444444, 110, 120, 90, 44, DunkerUI::onCfgBaud125);
        m_btnCfgB250 = mkBtn(t, "250k", 0x444444, 208, 120, 90, 44, DunkerUI::onCfgBaud250);
        m_btnCfgB500 = mkBtn(t, "500k", 0x444444, 306, 120, 90, 44, DunkerUI::onCfgBaud500);
        mkLabel(t, "(keiner = Baud unveraendert)", 410, 130, 0x8888AA, &lv_font_montserrat_14);

        mkBtn(t, "APPLY (LSS)", 0x007744, 8, 178, 220, 56, DunkerUI::onCfgApply);

        mkLabel(t,
            "Dunker: Node-ID (0x2000:03) + Baud (0x2000:02) via SDO.\n"
            "Nach dem Senden Geraet power-cyclen.",
            250, 186, 0xFFBB33, &lv_font_montserrat_14);

        m_lblLssStatus = mkLabel(t, "Status: ---", 8, 250, 0xAAAAAA, &lv_font_montserrat_14);

        refreshCfgLabels();
    }

    void refreshCfgLabels() {
        if (m_lblCfgNode) {
            char b[8]; snprintf(b, sizeof(b), "%u", (unsigned)m_cfgNodeId);
            lv_label_set_text(m_lblCfgNode, b);
        }
        auto hl = [](lv_obj_t* btn, bool sel) {
            if (btn) lv_obj_set_style_bg_color(btn, lv_color_hex(sel ? 0x007744 : 0x444444), 0);
        };
        hl(m_btnCfgB125, m_cfgBaud == 125000);
        hl(m_btnCfgB250, m_cfgBaud == 250000);
        hl(m_btnCfgB500, m_cfgBaud == 500000);
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

    // Cfg tab (LSS)
    static void onCfgNodeMinus(lv_event_t*) { if (!s_inst) return; if (s_inst->m_cfgNodeId > 1)   s_inst->m_cfgNodeId--; s_inst->refreshCfgLabels(); }
    static void onCfgNodePlus(lv_event_t*)  { if (!s_inst) return; if (s_inst->m_cfgNodeId < 127) s_inst->m_cfgNodeId++; s_inst->refreshCfgLabels(); }
    static void cfgSetBaud(uint32_t b)      { if (!s_inst) return; s_inst->m_cfgBaud = (s_inst->m_cfgBaud == b) ? 0 : b; s_inst->refreshCfgLabels(); }
    static void onCfgBaud125(lv_event_t*)   { cfgSetBaud(125000); }
    static void onCfgBaud250(lv_event_t*)   { cfgSetBaud(250000); }
    static void onCfgBaud500(lv_event_t*)   { cfgSetBaud(500000); }
    static void onCfgApply(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onLssApply) s_inst->m_cbs.onLssApply(s_inst->m_cfgNodeId, s_inst->m_cfgBaud);
    }

    // Keyboard show/hide with iPhone-style content scrolling.
    static void onTaFocused(lv_event_t* ev) {
        if (!s_inst || !s_inst->m_kb) return;
        lv_obj_t* ta = lv_event_get_target(ev);
        lv_keyboard_set_textarea(s_inst->m_kb, ta);
        lv_obj_clear_flag(s_inst->m_kb, LV_OBJ_FLAG_HIDDEN);
        if (s_inst->m_motCont) {
            // Shrink the form to the visible area above the keyboard, then bring
            // the focused field into view (lower controls stay scrollable).
            lv_obj_set_height(s_inst->m_motCont, 480 - KB_HEIGHT - TAB_TOP);
            lv_obj_scroll_to_view(ta, LV_ANIM_ON);
        }
    }
    static void onKbReady(lv_event_t*) {
        if (!s_inst || !s_inst->m_kb) return;
        lv_obj_add_flag(s_inst->m_kb, LV_OBJ_FLAG_HIDDEN);
        if (s_inst->m_motCont) {
            lv_obj_set_height(s_inst->m_motCont, lv_pct(100));
            lv_obj_scroll_to_y(s_inst->m_motCont, 0, LV_ANIM_ON);
        }
    }

    // ---------------- State ----------------
    static constexpr int KB_HEIGHT = 210;
    static constexpr int TAB_TOP   = 100; // header (56) + tab bar (44) in screen coords

    Dunker_Callbacks m_cbs;
    uint8_t m_nodeId = 0;

    lv_obj_t* m_screen = nullptr;
    lv_obj_t* m_lblTitle = nullptr;
    lv_obj_t* m_motCont = nullptr;

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

    // Cfg tab (LSS)
    uint8_t   m_cfgNodeId = 1;
    uint32_t  m_cfgBaud = 0;          // 0 = keep current baud
    lv_obj_t* m_lblCfgNode = nullptr;
    lv_obj_t* m_btnCfgB125 = nullptr;
    lv_obj_t* m_btnCfgB250 = nullptr;
    lv_obj_t* m_btnCfgB500 = nullptr;
    lv_obj_t* m_lblLssStatus = nullptr;

    inline static DunkerUI* s_inst = nullptr;
};
