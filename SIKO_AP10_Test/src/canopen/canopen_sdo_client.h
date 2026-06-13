/**
 * @file sdo_client.h
 * @brief CANopen SDO Client (Service Data Object)
 * @project SIKO AP10 Testprojekt
 * @date 2025
 *
 * Ermöglicht das Lesen und Schreiben von Objektverzeichnis-Einträgen
 * der AP10 über SDO-Expedited-Transfer (bis 4 Byte).
 *
 * SDO COB-IDs:
 *   SDO Request (ESP32 → AP10): 0x600 + Node-ID
 *   SDO Response (AP10 → ESP32): 0x580 + Node-ID
 *
 * Protokoll (Expedited, vereinfacht):
 *   Write Request:  [0x23/0x27/0x2B/0x2F] [Index-L] [Index-H] [Sub] [D0..D3]
 *   Write Response: [0x60] [Index-L] [Index-H] [Sub] [0 0 0 0]
 *   Read Request:   [0x40] [Index-L] [Index-H] [Sub] [0 0 0 0]
 *   Read Response:  [0x43/0x47/0x4B/0x4F] [Index-L] [Index-H] [Sub] [D0..D3]
 *   Error Response: [0x80] [Index-L] [Index-H] [Sub] [Err-Code 4 Byte]
 *
 * SPEICHERN ALS: src/canopen/sdo_client.h
 */

#ifndef SDO_CLIENT_H
#define SDO_CLIENT_H

#include <Arduino.h>
#include "canopen_driver.h"

// ============================================================================
// SDO Ergebnis-Codes
// ============================================================================

enum SdoResult {
    SDO_OK          = 0,
    SDO_TIMEOUT     = 1,
    SDO_ERROR       = 2,
    SDO_BUSY        = 3
};

// ============================================================================
// SDO Client Klasse
// ============================================================================

class SdoClient {
private:
    CanopenDriver* m_driver;
    uint8_t        m_nodeId;

    // Pending-Transaktion
    bool     m_pending;
    uint16_t m_pendingIndex;
    uint8_t  m_pendingSub;
    bool     m_pendingWrite;
    uint32_t m_pendingStartMs;
    uint32_t m_timeoutMs;

    // Ergebnis der letzten Transaktion
    SdoResult m_lastResult;
    uint32_t  m_lastValue;
    uint32_t  m_lastAbortCode;

    // Callback bei Abschluss
    std::function<void(SdoResult, uint32_t value)> m_callback;

public:
    SdoClient(CanopenDriver* driver, uint8_t nodeId = 125)
        : m_driver(driver)
        , m_nodeId(nodeId)
        , m_pending(false)
        , m_pendingIndex(0)
        , m_pendingSub(0)
        , m_pendingWrite(false)
        , m_pendingStartMs(0)
        , m_timeoutMs(500)
        , m_lastResult(SDO_OK)
        , m_lastValue(0)
        , m_lastAbortCode(0)
        , m_callback(nullptr)
    {}

    void setNodeId(uint8_t id)       { m_nodeId = id; }
    void setTimeout(uint32_t ms)     { m_timeoutMs = ms; }
    bool isBusy()              const { return m_pending; }
    SdoResult getLastResult()  const { return m_lastResult; }
    uint32_t  getLastValue()   const { return m_lastValue; }

    void setCallback(std::function<void(SdoResult, uint32_t)> cb) {
        m_callback = cb;
    }

    // -----------------------------------------------------------------------
    // Eingehende CAN-Frames verarbeiten
    // -----------------------------------------------------------------------
    void processFrame(uint32_t cobId, const uint8_t* data, uint8_t length) {
        // SDO Response COB-ID = 0x580 + Node-ID
        if (cobId != (uint32_t)(0x580 + m_nodeId)) return;
        if (!m_pending) return;
        if (length < 4) return;

        uint8_t  cmd   = data[0];
        uint16_t index = (uint16_t)(data[1]) | ((uint16_t)(data[2]) << 8);
        uint8_t  sub   = data[3];

        // Prüfen ob Antwort zur laufenden Transaktion gehört
        if (index != m_pendingIndex || sub != m_pendingSub) return;

        if (cmd == 0x80) {
            // Abort / Fehler
            m_lastAbortCode = ((uint32_t)data[4]) |
                              ((uint32_t)data[5] << 8) |
                              ((uint32_t)data[6] << 16) |
                              ((uint32_t)data[7] << 24);
            Serial.printf("[SDO] ABORT Index=0x%04X Sub=%d Code=0x%08lX\n",
                         index, sub, (unsigned long)m_lastAbortCode);
            finishTransaction(SDO_ERROR, 0);

        } else if (!m_pendingWrite && (cmd & 0x40)) {
            // Read Response: 0x4F=1Byte, 0x4B=2Byte, 0x47=3Byte, 0x43=4Byte
            uint8_t bytes = 4 - ((cmd >> 2) & 0x03);
            uint32_t value = 0;
            for (int i = 0; i < bytes && i < 4; i++) {
                value |= ((uint32_t)data[4 + i]) << (8 * i);
            }
            Serial.printf("[SDO] Read OK Index=0x%04X Sub=%d Value=%lu (0x%08lX)\n",
                         index, sub, (unsigned long)value, (unsigned long)value);
            finishTransaction(SDO_OK, value);

        } else if (m_pendingWrite && cmd == 0x60) {
            // Write Response OK
            Serial.printf("[SDO] Write OK Index=0x%04X Sub=%d\n", index, sub);
            finishTransaction(SDO_OK, 0);
        }
    }

    // -----------------------------------------------------------------------
    // Timeout-Überwachung (im loop() aufrufen)
    // -----------------------------------------------------------------------
    void update() {
        if (!m_pending) return;
        if ((millis() - m_pendingStartMs) > m_timeoutMs) {
            Serial.printf("[SDO] TIMEOUT Index=0x%04X Sub=%d\n",
                         m_pendingIndex, m_pendingSub);
            finishTransaction(SDO_TIMEOUT, 0);
        }
    }

    // -----------------------------------------------------------------------
    // SDO Read (asynchron, Ergebnis über Callback)
    // -----------------------------------------------------------------------
    bool readAsync(uint16_t index, uint8_t sub,
                   std::function<void(SdoResult, uint32_t)> cb = nullptr) {
        if (m_pending) {
            Serial.println("[SDO] Busy!");
            return false;
        }

        m_callback       = cb;
        m_pendingIndex   = index;
        m_pendingSub     = sub;
        m_pendingWrite   = false;
        m_pending        = true;
        m_pendingStartMs = millis();

        // SDO Read Request: [0x40] [IndexL] [IndexH] [Sub] [0 0 0 0]
        uint8_t frame[8] = {
            0x40,
            (uint8_t)(index & 0xFF),
            (uint8_t)(index >> 8),
            sub,
            0, 0, 0, 0
        };

        uint32_t cobId = 0x600 + m_nodeId;
        Serial.printf("[SDO] Read Request Index=0x%04X Sub=%d\n", index, sub);
        return m_driver->sendFrame(cobId, frame, 8);
    }

    // -----------------------------------------------------------------------
    // SDO Write (asynchron)
    // -----------------------------------------------------------------------
    bool writeAsync(uint16_t index, uint8_t sub, uint32_t value, uint8_t dataBytes = 4,
                    std::function<void(SdoResult, uint32_t)> cb = nullptr) {
        if (m_pending) {
            Serial.println("[SDO] Busy!");
            return false;
        }

        m_callback       = cb;
        m_pendingIndex   = index;
        m_pendingSub     = sub;
        m_pendingWrite   = true;
        m_pending        = true;
        m_pendingStartMs = millis();

        // Kommando-Byte je nach Datenlänge
        // 0x2F=1Byte, 0x2B=2Byte, 0x27=3Byte, 0x23=4Byte
        uint8_t cmd = 0x23 | ((4 - dataBytes) << 2);

        uint8_t frame[8] = {
            cmd,
            (uint8_t)(index & 0xFF),
            (uint8_t)(index >> 8),
            sub,
            (uint8_t)(value & 0xFF),
            (uint8_t)((value >> 8) & 0xFF),
            (uint8_t)((value >> 16) & 0xFF),
            (uint8_t)((value >> 24) & 0xFF)
        };

        uint32_t cobId = 0x600 + m_nodeId;
        Serial.printf("[SDO] Write Request Index=0x%04X Sub=%d Value=%lu\n",
                     index, sub, value);
        return m_driver->sendFrame(cobId, frame, 8);
    }

    // -----------------------------------------------------------------------
    // Blockierendes Read (für Setup-Phase, max. timeoutMs warten)
    // -----------------------------------------------------------------------
    SdoResult readBlocking(uint16_t index, uint8_t sub, uint32_t& outValue,
                           uint32_t timeoutMs = 1000) {
        m_lastResult = SDO_BUSY;
        bool done = false;

        readAsync(index, sub, [&](SdoResult r, uint32_t v) {
            m_lastResult = r;
            outValue     = v;
            done         = true;
        });

        uint32_t start = millis();
        while (!done && (millis() - start) < timeoutMs) {
            update();
            delay(1);
        }

        if (!done) {
            m_pending    = false;
            m_lastResult = SDO_TIMEOUT;
        }

        return m_lastResult;
    }

    // -----------------------------------------------------------------------
    // Blockierendes Write
    // -----------------------------------------------------------------------
    SdoResult writeBlocking(uint16_t index, uint8_t sub, uint32_t value,
                            uint8_t dataBytes = 4, uint32_t timeoutMs = 1000) {
        m_lastResult = SDO_BUSY;
        bool done = false;

        writeAsync(index, sub, value, dataBytes, [&](SdoResult r, uint32_t) {
            m_lastResult = r;
            done         = true;
        });

        uint32_t start = millis();
        while (!done && (millis() - start) < timeoutMs) {
            update();
            delay(1);
        }

        if (!done) {
            m_pending    = false;
            m_lastResult = SDO_TIMEOUT;
        }

        return m_lastResult;
    }

private:
    void finishTransaction(SdoResult result, uint32_t value) {
        m_pending    = false;
        m_lastResult = result;
        m_lastValue  = value;

        if (m_callback) {
            auto cb = m_callback;
            m_callback = nullptr;
            cb(result, value);
        }
    }
};

#endif // SDO_CLIENT_H
