#pragma once

/*
 * CANopen LSS Master (minimal) for Arduino-style CAN driver
 *
 * Implements:
 * - Switch Mode Global (configuration/operation)
 * - Configure Node-ID
 * - Configure Bit Timing / Baudrate (standard table)
 * - Store Configuration
 *
 * NOTE:
 * This is a minimal implementation aimed at configuring ONE device at a time.
 * It uses GLOBAL mode switching (all LSS slaves). For safety, instruct user to
 * connect only one unconfigured device at a time, or use Identify (fastscan)
 * later.
 */

#include <Arduino.h>
#include <stdint.h>

#include "src/canopen/canopen_driver.h"

class LssMaster {
public:
    explicit LssMaster(CanopenDriver* drv)
        : m_drv(drv)
        , m_lastTxMs(0)
        , m_lastRxMs(0)
        , m_lastRespCs(0)
        , m_lastError(0)
        , m_hasResp(false)
    {}

    void begin() {
        m_hasResp = false;
        m_lastError = 0;
    }

    // Call from CAN RX callback
    void processFrame(uint32_t cobId, const uint8_t* data, uint8_t len) {
        if (cobId != 0x7E4 || !data || len < 1) return;
        // byte0 = CS
        m_lastRespCs = data[0];
        m_lastRxMs = millis();
        m_hasResp = true;
        if (len >= 2) m_lastError = data[1];
    }

    bool hasResponse() const { return m_hasResp; }
    uint8_t lastRespCs() const { return m_lastRespCs; }
    uint8_t lastError() const { return m_lastError; }
    uint32_t lastRxMs() const { return m_lastRxMs; }

    // ---- LSS Services ----

    // Switch Mode Global: mode=0 (operation) or 1 (configuration)
    bool switchModeGlobal(uint8_t mode) {
        uint8_t d[8] = {0};
        d[0] = 0x04; // Switch Mode Global
        d[1] = mode;
        return tx(0x7E5, d, 2);
    }

    // Configure Node-ID: nodeId 1..127
    bool configureNodeId(uint8_t nodeId) {
        uint8_t d[8] = {0};
        d[0] = 0x11; // Configure Node-ID
        d[1] = nodeId;
        return tx(0x7E5, d, 2);
    }

    // Configure Bit Timing: table selector 0..9 (depends on CiA 305 table)
    // According to CiA 305: CS=0x13, byte1=0x00 (table selector), byte2=table index.
    bool configureBitTiming(uint8_t tableSel) {
        uint8_t d[8] = {0};
        d[0] = 0x13; // Configure Bit Timing Parameters
        d[1] = 0x00; // Bit timing parameter selector (0 = standard table)
        d[2] = tableSel;
        return tx(0x7E5, d, 3);
    }

    // Activate Bit Timing (apply new bitrate): CS=0x15
    bool activateBitTiming() {
        uint8_t d[8] = {0};
        d[0] = 0x15;
        return tx(0x7E5, d, 1);
    }

    // Store configuration: CS=0x17
    bool storeConfiguration() {
        uint8_t d[8] = {0};
        d[0] = 0x17;
        return tx(0x7E5, d, 1);
    }

    // Inquire Node-ID: CS=0x5E
    bool inquireNodeId() {
        uint8_t d[8] = {0};
        d[0] = 0x5E;
        return tx(0x7E5, d, 1);
    }

    // Inquire Bit Timing: CS=0x5C
    bool inquireBitTiming() {
        uint8_t d[8] = {0};
        d[0] = 0x5C;
        return tx(0x7E5, d, 1);
    }

    // Wait for LSS response; optionally validate CS and error==0
    bool waitOk(uint32_t timeoutMs, uint8_t expectedCs) {
        if (!waitResponse(timeoutMs, expectedCs)) return false;
        return m_lastError == 0;
    }

    // Convenience: configure node-id + bitrate (global)
    // WARNING: Global mode switching affects all LSS-capable nodes.
    bool configureGlobal(uint8_t newNodeId, uint32_t newBaud, bool doStore, bool doActivate, uint32_t timeoutMs = 250) {
        uint8_t sel = 0;
        if (!baudrateToTableSel(newBaud, sel)) return false;

        // Enter config
        if (!switchModeGlobal(1)) return false;
        delay(10);

        // Set bitrate
        configureBitTiming(sel);
        delay(10);

        // Set node-id
        configureNodeId(newNodeId);
        delay(10);

        if (doStore) {
            storeConfiguration();
            delay(10);
        }

        if (doActivate) {
            activateBitTiming();
            delay(10);
        }

        // Back to operation
        switchModeGlobal(0);
        return true;
    }

    // Wait for response with expected CS (optionally)
    bool waitResponse(uint32_t timeoutMs, int expectedCs = -1) {
        const uint32_t start = millis();
        m_hasResp = false;
        while ((millis() - start) < timeoutMs) {
            // Driver RX is interrupt/callback driven; just yield
            delay(1);
            if (m_hasResp) {
                if (expectedCs < 0) return true;
                return (m_lastRespCs == (uint8_t)expectedCs);
            }
        }
        return false;
    }

    static bool baudrateToTableSel(uint32_t baud, uint8_t& outSel) {
        // CiA 305 standard table (common):
        // 0=1M,1=800k,2=500k,3=250k,4=125k,5=100k,6=50k,7=20k,8=10k
        switch (baud) {
            case 1000000: outSel = 0; return true;
            case 800000:  outSel = 1; return true;
            case 500000:  outSel = 2; return true;
            case 250000:  outSel = 3; return true;
            case 125000:  outSel = 4; return true;
            case 100000:  outSel = 5; return true;
            case 50000:   outSel = 6; return true;
            case 20000:   outSel = 7; return true;
            case 10000:   outSel = 8; return true;
            default: return false;
        }
    }

private:
    bool tx(uint32_t id, const uint8_t* data, uint8_t len) {
        if (!m_drv) return false;
        m_lastTxMs = millis();
        return m_drv->sendFrame(id, data, len);
    }

    CanopenDriver* m_drv;
    uint32_t m_lastTxMs;
    uint32_t m_lastRxMs;
    uint8_t  m_lastRespCs;
    uint8_t  m_lastError;
    volatile bool m_hasResp;
};
