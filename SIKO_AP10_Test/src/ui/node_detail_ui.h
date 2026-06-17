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

    // Identity (1018h) - filled from discovery / identify-all / READ INFO
    bool     identityKnown = false;
    uint32_t vendorId = 0;
    uint32_t productCode = 0;
    uint32_t revision = 0;
    uint32_t serial = 0;

    // Error register (1001h, UNSIGNED8 bitfield per DS301)
    bool    errRegValid = false;
    uint8_t errReg = 0;
};

struct NodeDetail_Callbacks {
    std::function<void()> onBack = nullptr;

    std::function<void(uint8_t nodeId)> onNmtStart = nullptr;
    std::function<void(uint8_t nodeId)> onNmtPreOp = nullptr;
    std::function<void(uint8_t nodeId)> onNmtStop = nullptr;
    std::function<void(uint8_t nodeId)> onNmtResetNode = nullptr;
    std::function<void(uint8_t nodeId)> onNmtResetComm = nullptr;

    // Read standard objects 1018h (identity) + 1001h (error register) via SDO
    std::function<void(uint8_t nodeId)> onReadInfo = nullptr;

    // Set baudrate (device-specific method chosen by the app from the node type)
    std::function<void(uint8_t nodeId, uint32_t baud)> onSetBaud = nullptr;

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

        // Scrollable content container (everything below the header). When the
        // keyboard appears we shrink this to the area above it and scroll the
        // focused field into view (iPhone-style), so nothing stays hidden.
        m_content = lv_obj_create(m_screen);
        lv_obj_set_pos(m_content, 0, 60);
        lv_obj_set_size(m_content, 800, 420);
        lv_obj_set_style_bg_opa(m_content, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(m_content, 0, 0);
        lv_obj_set_style_radius(m_content, 0, 0);
        lv_obj_set_style_pad_all(m_content, 0, 0);
        lv_obj_set_scroll_dir(m_content, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(m_content, LV_SCROLLBAR_MODE_AUTO);

        // Info card (left column = NMT state, right column = identity + error reg)
        lv_obj_t* card = lv_obj_create(m_content);
        lv_obj_set_size(card, 620, 120);
        lv_obj_set_pos(card, 10, 20);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x12122A), 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x0F3460), 0);
        lv_obj_set_style_border_width(card, 2, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        // Left column
        m_lblNode = lv_label_create(card);
        lv_label_set_text(m_lblNode, "Node: ---");
        lv_obj_set_style_text_color(m_lblNode, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(m_lblNode, &lv_font_montserrat_18, 0);
        lv_obj_set_pos(m_lblNode, 12, 10);

        m_lblState = lv_label_create(card);
        lv_label_set_text(m_lblState, "State: ---");
        lv_obj_set_style_text_color(m_lblState, lv_color_hex(0xFFBB33), 0);
        lv_obj_set_style_text_font(m_lblState, &lv_font_montserrat_16, 0);
        lv_obj_set_pos(m_lblState, 12, 44);

        m_lblLast = lv_label_create(card);
        lv_label_set_text(m_lblLast, "Last seen: ---");
        lv_obj_set_style_text_color(m_lblLast, lv_color_hex(0xAAAAAA), 0);
        lv_obj_set_style_text_font(m_lblLast, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblLast, 12, 80);

        // Right column: identity (1018h)
        m_lblVendor = lv_label_create(card);
        lv_label_set_text(m_lblVendor, "Vendor: ---");
        lv_obj_set_style_text_color(m_lblVendor, lv_color_hex(0xCCCCCC), 0);
        lv_obj_set_style_text_font(m_lblVendor, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblVendor, 320, 8);

        m_lblProduct = lv_label_create(card);
        lv_label_set_text(m_lblProduct, "Product: ---");
        lv_obj_set_style_text_color(m_lblProduct, lv_color_hex(0xCCCCCC), 0);
        lv_obj_set_style_text_font(m_lblProduct, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblProduct, 320, 36);

        m_lblRevSer = lv_label_create(card);
        lv_label_set_text(m_lblRevSer, "Rev/Ser: ---");
        lv_obj_set_style_text_color(m_lblRevSer, lv_color_hex(0xCCCCCC), 0);
        lv_obj_set_style_text_font(m_lblRevSer, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblRevSer, 320, 64);

        // Right column: error register (1001h)
        m_lblErr = lv_label_create(card);
        lv_label_set_text(m_lblErr, "Err 1001h: ---");
        lv_obj_set_style_text_color(m_lblErr, lv_color_hex(0xFF6666), 0);
        lv_obj_set_style_text_font(m_lblErr, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblErr, 320, 92);

        // READ INFO button (reads 1018h identity + 1001h error register via SDO)
        m_btnReadInfo = makeBtn("READ\nINFO", lv_color_hex(0x4A2C82), 640, 20, 150, 120,
                                NodeDetailUI::onReadInfoClicked);

        const int x = 10;

        // Config row (Node-ID)
        m_taNodeId = lv_textarea_create(m_content);
        lv_obj_set_size(m_taNodeId, 160, 48);
        lv_obj_set_pos(m_taNodeId, x, 148);
        lv_textarea_set_one_line(m_taNodeId, true);
        lv_textarea_set_accepted_chars(m_taNodeId, "0123456789");
        lv_textarea_set_max_length(m_taNodeId, 3);
        lv_textarea_set_text(m_taNodeId, "1");
        lv_textarea_set_placeholder_text(m_taNodeId, "Node-ID");
        m_btnSetNodeId = makeBtn("SET NODE-ID", lv_color_hex(0x1D4ED8), x + 180, 146, 250, 52,
                                 NodeDetailUI::onSetNodeIdClicked);

        // NMT buttons
        const int w = 250;
        const int h = 64;
        const int gap = 14;
        int y = 214;
        m_btnStart = makeBtn("NMT START", lv_color_hex(0x007744), x, y, w, h, NodeDetailUI::onStartClicked);
        m_btnPreOp = makeBtn("NMT PRE-OP", lv_color_hex(0xCC6600), x + (w + gap), y, w, h, NodeDetailUI::onPreOpClicked);
        m_btnStop  = makeBtn("NMT STOP", lv_color_hex(0x444444), x + 2*(w + gap), y, w, h, NodeDetailUI::onStopClicked);

        y += h + gap;
        m_btnResetNode = makeBtn("RESET NODE", lv_color_hex(0x666666), x, y, w, h, NodeDetailUI::onResetNodeClicked);
        m_btnResetComm = makeBtn("RESET COMM", lv_color_hex(0x666666), x + (w + gap), y, w, h, NodeDetailUI::onResetCommClicked);

        // Baud config row (device-specific; e.g. Bosch Rexroth ECODRIVE via 0x3FEF:07).
        y += h + gap;
        lv_obj_t* lblB = lv_label_create(m_content);
        lv_label_set_text(lblB, "Set Baud:");
        lv_obj_set_style_text_color(lblB, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(lblB, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(lblB, x, y + 14);
        makeBtn("125k", lv_color_hex(0x0066CC), x + 110, y, 110, 48, NodeDetailUI::onBaud125Clicked);
        makeBtn("250k", lv_color_hex(0x0066CC), x + 228, y, 110, 48, NodeDetailUI::onBaud250Clicked);
        makeBtn("500k", lv_color_hex(0x0066CC), x + 346, y, 110, 48, NodeDetailUI::onBaud500Clicked);
        makeBtn("1M",   lv_color_hex(0x0066CC), x + 464, y, 110, 48, NodeDetailUI::onBaud1MClicked);
        m_lblBaudStatus = lv_label_create(m_content);
        lv_label_set_text(m_lblBaudStatus, "");
        lv_obj_set_style_text_color(m_lblBaudStatus, lv_color_hex(0xAAAAAA), 0);
        lv_obj_set_style_text_font(m_lblBaudStatus, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(m_lblBaudStatus, x, y + 56);

        // On-screen numeric keypad (sibling of m_content so it never scrolls);
        // hidden until the Node-ID field is focused, bottom-aligned.
        m_kb = lv_keyboard_create(m_screen);
        lv_keyboard_set_mode(m_kb, LV_KEYBOARD_MODE_NUMBER);
        lv_obj_set_size(m_kb, 800, KB_HEIGHT);
        lv_obj_align(m_kb, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_add_flag(m_kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(m_kb, NodeDetailUI::onKbReady, LV_EVENT_READY,  nullptr);
        lv_obj_add_event_cb(m_kb, NodeDetailUI::onKbReady, LV_EVENT_CANCEL, nullptr);
        lv_obj_add_event_cb(m_taNodeId, NodeDetailUI::onTaFocused, LV_EVENT_FOCUSED, nullptr);

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

        // Identity (1018h)
        if (m_lblVendor) {
            char b[48];
            if (info.identityKnown) snprintf(b, sizeof(b), "Vendor: 0x%08lX", (unsigned long)info.vendorId);
            else snprintf(b, sizeof(b), "Vendor: ---");
            lv_label_set_text(m_lblVendor, b);
        }
        if (m_lblProduct) {
            char b[64];
            if (info.identityKnown) {
                char ascii[8];
                productAscii(info.productCode, ascii, sizeof(ascii));
                if (ascii[0]) snprintf(b, sizeof(b), "Product: 0x%08lX (%s)", (unsigned long)info.productCode, ascii);
                else          snprintf(b, sizeof(b), "Product: 0x%08lX", (unsigned long)info.productCode);
            } else {
                snprintf(b, sizeof(b), "Product: ---");
            }
            lv_label_set_text(m_lblProduct, b);
        }
        if (m_lblRevSer) {
            char b[64];
            if (info.identityKnown) snprintf(b, sizeof(b), "Rev: 0x%08lX Ser: 0x%08lX",
                                             (unsigned long)info.revision, (unsigned long)info.serial);
            else snprintf(b, sizeof(b), "Rev/Ser: ---");
            lv_label_set_text(m_lblRevSer, b);
        }

        // Error register (1001h)
        if (m_lblErr) {
            char b[64];
            if (info.errRegValid) {
                char dec[40];
                errRegDecode(info.errReg, dec, sizeof(dec));
                snprintf(b, sizeof(b), "Err 1001h: 0x%02X [%s]", (unsigned)info.errReg, dec);
            } else {
                snprintf(b, sizeof(b), "Err 1001h: ---");
            }
            lv_label_set_text(m_lblErr, b);
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

    static void onReadInfoClicked(lv_event_t*) {
        if (s_inst && s_inst->m_cbs.onReadInfo) s_inst->m_cbs.onReadInfo(s_inst->m_info.nodeId);
    }

    // Keyboard show/hide with iPhone-style content scrolling.
    static void onTaFocused(lv_event_t* ev) {
        if (!s_inst || !s_inst->m_kb) return;
        lv_obj_t* ta = lv_event_get_target(ev);
        lv_keyboard_set_textarea(s_inst->m_kb, ta);
        lv_obj_clear_flag(s_inst->m_kb, LV_OBJ_FLAG_HIDDEN);
        // Shrink the scroll area to the space above the keyboard, then bring the
        // focused field into view. Lower widgets stay reachable by scrolling.
        lv_obj_set_height(s_inst->m_content, 480 - HEADER_H - KB_HEIGHT);
        lv_obj_scroll_to_view(ta, LV_ANIM_ON);
    }
    static void onKbReady(lv_event_t*) {
        if (!s_inst || !s_inst->m_kb) return;
        lv_obj_add_flag(s_inst->m_kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_height(s_inst->m_content, 480 - HEADER_H);
        lv_obj_scroll_to_y(s_inst->m_content, 0, LV_ANIM_ON);
    }

    static void emitBaud(uint32_t baud) {
        if (s_inst && s_inst->m_cbs.onSetBaud) s_inst->m_cbs.onSetBaud(s_inst->m_info.nodeId, baud);
    }
    static void onBaud125Clicked(lv_event_t*) { emitBaud(125000); }
    static void onBaud250Clicked(lv_event_t*) { emitBaud(250000); }
    static void onBaud500Clicked(lv_event_t*) { emitBaud(500000); }
    static void onBaud1MClicked(lv_event_t*)  { emitBaud(1000000); }

public:
    void setBaudStatus(const char* text, bool ok) {
        if (!m_lblBaudStatus) return;
        lv_label_set_text(m_lblBaudStatus, text ? text : "");
        lv_obj_set_style_text_color(m_lblBaudStatus, lv_color_hex(ok ? 0x33DD66 : 0xFF6666), 0);
    }

    // Prominent confirmation that the baud write succeeded and a power-cycle is
    // required. baudKbps = new rate in kBit/s (for the message), 0 = omit.
    void showBaudWrittenMsg(uint32_t baudKbps) {
        static const char* btns[] = { "OK", "" };
        char txt[200];
        if (baudKbps)
            snprintf(txt, sizeof(txt),
                     "Neue Baudrate %lu kBit/s erfolgreich geschrieben.\n\n"
                     "Bitte den Antrieb AUS- und wieder EINSCHALTEN\n"
                     "(Power-Cycle), damit die Aenderung aktiv wird.",
                     (unsigned long)baudKbps);
        else
            snprintf(txt, sizeof(txt),
                     "Baudrate erfolgreich geschrieben.\n\n"
                     "Bitte den Antrieb AUS- und wieder EINSCHALTEN (Power-Cycle).");
        lv_obj_t* mbox = lv_msgbox_create(nullptr, LV_SYMBOL_OK "  Baudrate gesetzt", txt, btns, false);
        lv_obj_center(mbox);
        lv_obj_add_event_cb(mbox, NodeDetailUI::onMsgBoxOk, LV_EVENT_VALUE_CHANGED, nullptr);
    }
private:
    static void onMsgBoxOk(lv_event_t* e) {
        lv_msgbox_close(lv_event_get_current_target(e));
    }


    // Render the product-code as ASCII if its bytes are printable (e.g. AP04 = "CAN").
    static void productAscii(uint32_t pc, char* out, size_t n) {
        if (!out || n == 0) return;
        out[0] = 0;
        char tmp[5];
        int k = 0;
        for (int i = 0; i < 4; i++) {
            char c = (char)((pc >> (8 * i)) & 0xFF);
            if (c == 0) break;                 // string terminator
            if (c < 32 || c > 126) { k = 0; break; } // non-printable -> not ASCII
            tmp[k++] = c;
        }
        if (k >= 2) { tmp[k] = 0; snprintf(out, n, "%s", tmp); }
    }

    // Decode the DS301 error-register (1001h) bitfield into a compact string.
    static void errRegDecode(uint8_t e, char* out, size_t n) {
        if (!out || n == 0) return;
        if (e == 0) { snprintf(out, n, "OK"); return; }
        out[0] = 0;
        size_t len = 0;
        auto add = [&](const char* s) {
            if (len >= n) return;
            len += snprintf(out + len, n - len, "%s%s", len ? "|" : "", s);
        };
        if (e & 0x01) add("GEN");   // generic error
        if (e & 0x02) add("CUR");   // current
        if (e & 0x04) add("VOLT");  // voltage
        if (e & 0x08) add("TEMP");  // temperature
        if (e & 0x10) add("COMM");  // communication
        if (e & 0x20) add("PROF");  // device profile specific
        if (e & 0x80) add("MFR");   // manufacturer specific
    }

    lv_obj_t* makeBtn(const char* text, lv_color_t bg, int x, int y, int w, int h,
                      lv_event_cb_t cb) {
        lv_obj_t* b = lv_btn_create(m_content);
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

    static constexpr int HEADER_H = 60;
    static constexpr int KB_HEIGHT = 190;

    lv_obj_t* m_screen = nullptr;
    lv_obj_t* m_content = nullptr;
    NodeDetail_Info m_info;
    NodeDetail_Callbacks m_cbs;

    lv_obj_t* m_lblNode = nullptr;
    lv_obj_t* m_lblState = nullptr;
    lv_obj_t* m_lblLast = nullptr;

    lv_obj_t* m_lblBaudStatus = nullptr;

    // Identity + error-register widgets
    lv_obj_t* m_lblVendor = nullptr;
    lv_obj_t* m_lblProduct = nullptr;
    lv_obj_t* m_lblRevSer = nullptr;
    lv_obj_t* m_lblErr = nullptr;
    lv_obj_t* m_btnReadInfo = nullptr;

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
