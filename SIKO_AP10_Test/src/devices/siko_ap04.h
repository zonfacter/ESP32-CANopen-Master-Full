/**
 * @file siko_ap04_fixed.h
 * @brief Geraeteabstraktion fuer SIKO AP04 (CANopen)
 *
 * FIXES (2026-06-10):
 * [FIX #1] Auto-Reconnect: Verbindungsverlust -> Reset Comm + NMT Start
 * [FIX #2] Min/Max intern als Rohwerte (int32_t), immer mit aktuellem
 *          scaleFactor umgerechnet. setScaleFactor() rechnet sofort neu.
 * [FIX #3] 0-Setzen: NUR Software-Offset (setZeroLocal()).
 *          KEIN SDO-Write! 2002h und 6003h loesen beim AP04 einen Reset aus.
 *          Der ESP32 merkt sich den aktuellen Rohwert als Offset.
 *          Die SIKO-Anzeige zeigt weiterhin ihren eigenen Wert – das ist korrekt.
 *          Der ESP32 rechnet intern relativ zum gesetzten Nullpunkt.
 * [FIX #4] Statusbyte vollstaendig dekodiert (alle relevanten Bits).
 *
 * WICHTIGE AP04 Objekte (laut Handbuch):
 *   2001h  Manufacturer Offset
 *   2002h  !! LOEST RESET AUS – NICHT VERWENDEN !!
 *   5F09h  Externer Heartbeat Timer (Werkseinst. 300ms SYNC-Ueberwachung)
 *   5F0Ah  Node-ID
 *   5F11h  Nachkommastellen
 *   5F13h  Anzeigendivisor (Skalierung)
 *   5F19h  AP04-Status
 *   6004h  Position value (Istwert lesen)
 *   1800h  TPDO1 Parameter (asynchron/zyklisch)
 *   1801h  TPDO2 Parameter (synchron)
 *   1017h  Producer Heartbeat Time
 */

#pragma once
#ifndef SIKO_AP04_FIXED_H
#define SIKO_AP04_FIXED_H

#include <Arduino.h>
#include <stdint.h>
#include "../canopen/canopen_driver.h"
#include "../canopen/sdo_client.h"
#include "../canopen/nmt_manager.h"


// ============================================================================
// AP04 Statusbyte-Bits (TPDO Byte 4, gemaess AP04-Handbuch)
// ============================================================================
namespace AP04_STATUS {
    constexpr uint8_t IN_POS        = (1 << 0); ///< 0=Not IN-POS,          1=IN-POS
    constexpr uint8_t IST_LT_SOLL   = (1 << 1); ///< 0=IST>=SOLL,           1=IST<SOLL (nach links)
    constexpr uint8_t BATTERY_WARN  = (1 << 2); ///< 0=Batterie ok,         1=Batterie Warnung
    constexpr uint8_t CHAIN_SET     = (1 << 3); ///< 0=Kettenmass=0,        1=Kettenmass gesetzt
    constexpr uint8_t ARROW_GT      = (1 << 4); ///< 0='>'-LED aus,         1='>'-LED ein
    constexpr uint8_t ARROW_LT      = (1 << 5); ///< 0='<'-LED aus,         1='<'-LED ein
    constexpr uint8_t BATTERY_EMPTY = (1 << 6); ///< 0=Batterie nicht leer, 1=Batterie leer
}

// ============================================================================
// AP04 Datenstruktur
// ============================================================================
struct AP04Data {
    // Verbindungsstatus
    bool     connected   = false;
    bool     operational = false;

    // Rohposition aus PDO (Geraeteeinheit, unveraendert vom Geraet)
    int32_t  positionRaw = 0;

    // Skaliert in mm, mit Software-Nullpunkt:
    // positionMm = (positionRaw - zeroOffset) * scaleFactor
    float    positionMm  = 0.0f;

    // Software-Nullpunkt (Rohwert beim letzten "0-Setzen")
    int32_t  zeroOffset  = 0;

    // Statusbyte aus TPDO Byte 4 (Rohwert)
    uint8_t  statusByte  = 0;

    // Dekodierte Statusbits (gemaess AP04-Handbuch)
    bool     inPos        = false; ///< IN-POS erreicht (Bit 0)
    bool     istLtSoll    = false; ///< IST < SOLL – nach links fahren (Bit 1)
    bool     batteryWarn  = false; ///< Batterie-Warnung (Bit 2)
    bool     batteryEmpty = false; ///< Batterie leer (Bit 6)
    bool     chainSet     = false; ///< Kettenmass gesetzt (Bit 3)
    bool     arrowGt      = false; ///< '>' LED aktiv (Bit 4)
    bool     arrowLt      = false; ///< '<' LED aktiv (Bit 5)

    // Statistik
    uint32_t updateCount = 0;
    uint32_t lastRxMs    = 0;

    // Min/Max – intern als relative Rohwerte (relativ zum zeroOffset)
    // [FIX #2] Rohwerte bleiben korrekt wenn scaleFactor sich aendert
    bool    hasMinMax = false;
    int32_t minRaw    = 0;    ///< Kleinster (positionRaw - zeroOffset) seit letztem Reset
    int32_t maxRaw    = 0;    ///< Groesster (positionRaw - zeroOffset) seit letztem Reset
    float   minMm     = 0.0f; ///< minRaw * scaleFactor (immer aktuell)
    float   maxMm     = 0.0f; ///< maxRaw * scaleFactor (immer aktuell)
};

// ============================================================================
// SikoAP04 Klasse
// ============================================================================
class SikoAP04 {
public:
    explicit SikoAP04(CanopenDriver* can, uint8_t nodeId)
        : m_can(can)
        , m_sdo(nullptr)
        , m_nmt(nullptr)
        , m_resetCommRequested(false)
        , m_nodeId(nodeId)
        , m_scaleFactor(0.1f)
        , m_zeroOffset(0)
        , m_reconnectTimer(0)
        , m_reconnectIntervalMs(3000)
        , m_timeoutMs(2000)
        , m_wasConnected(false)
    {
        m_tpdo1 = 0x180 + m_nodeId;
        m_tpdo2 = 0x280 + m_nodeId;
        m_hb    = 0x700 + m_nodeId;
    }

    // -----------------------------------------------------------------------
    // Konfiguration
    // -----------------------------------------------------------------------

    void setSdoClient(SdoClient* sdo)   { m_sdo = sdo; }
    void setNmtManager(NmtManager* nmt) { m_nmt = nmt; }

    /**
     * @brief Setzt den Skalierungsfaktor Rohwert -> mm.
     *        Entspricht dem Anzeigendivisor (Objekt 5F13h) der AP04.
     *        Beispiel: 145 Digits -> 14.5 mm  =>  factor = 0.1
     *
     *        [FIX #2] Min/Max werden als Rohwerte gespeichert und hier
     *        sofort neu in mm umgerechnet – keine veralteten Werte mehr.
     */
    void setScaleFactor(float factor) {
        if (factor <= 0.0f) factor = 1.0f;
        m_scaleFactor = factor;

        // [FIX #2] Min/Max-mm-Werte sofort neu berechnen
        if (m_data.hasMinMax) {
            m_data.minMm = (float)m_data.minRaw * m_scaleFactor;
            m_data.maxMm = (float)m_data.maxRaw * m_scaleFactor;
        }

        // Aktuelle Position neu berechnen
        const int32_t relRaw = m_data.positionRaw - m_zeroOffset;
        m_data.positionMm    = (float)relRaw * m_scaleFactor;
        m_data.zeroOffset    = m_zeroOffset;

        Serial.printf("[AP04] ScaleFactor gesetzt: %.4f | positionMm=%.2f | minMm=%.2f | maxMm=%.2f\n",
                      m_scaleFactor, m_data.positionMm, m_data.minMm, m_data.maxMm);
    }

    float getScaleFactor() const { return m_scaleFactor; }

    void setTimeoutMs(uint32_t ms)        { m_timeoutMs = ms; }
    void setReconnectIntervalMs(uint32_t ms) { m_reconnectIntervalMs = ms; }

    // -----------------------------------------------------------------------
    // Datenzugriff
    // -----------------------------------------------------------------------

    const AP04Data& getData() const { return m_data; }
    uint8_t nodeId() const          { return m_nodeId; }
    int32_t getZeroOffset() const   { return m_zeroOffset; }

    // -----------------------------------------------------------------------
    // Aktionen
    // -----------------------------------------------------------------------

    /**
     * @brief Fordert einen Reset Communication + NMT Start an.
     *        Wird im naechsten update()-Aufruf ausgefuehrt (nicht blockierend).
     */
    void requestResetComm() {
        m_resetCommRequested = true;
    }

    /**
     * @brief Setzt Min/Max-Tracking zurueck.
     *        [FIX #2] Setzt sowohl Roh- als auch mm-Werte zurueck.
     */
    void resetMinMax() {
        m_data.hasMinMax = false;
        m_data.minRaw    = 0;
        m_data.maxRaw    = 0;
        m_data.minMm     = 0.0f;
        m_data.maxMm     = 0.0f;
        Serial.println("[AP04] Min/Max zurueckgesetzt");
    }

    /**
     * @brief [FIX #3] Software-Nullpunkt lokal setzen – KEIN SDO-Write!
     *
     *        WARUM kein SDO:
     *        - Objekt 6003h: loest beim AP04 einen Kommunikations-Reset aus
     *        - Objekt 2002h: loest beim AP04 ebenfalls einen Reset aus
     *        - Die SIKO-Anzeige zeigt weiterhin ihren eigenen absoluten Wert –
     *          das ist korrekt und gewollt. Der ESP32 rechnet intern relativ.
     *
     *        Setzt den aktuellen Rohwert als lokalen Offset.
     *        Alle folgenden Positionen: positionMm = (positionRaw - zeroOffset) * scaleFactor
     *        Min/Max wird zurueckgesetzt (neuer Bezugspunkt).
     */
    void setZeroLocal() {
        m_zeroOffset      = m_data.positionRaw;
        m_data.zeroOffset = m_zeroOffset;
        m_data.positionMm = 0.0f;
        Serial.printf("[AP04] Software-Nullpunkt gesetzt: zeroOffset=%ld (positionRaw=%ld)\n",
                      (long)m_zeroOffset, (long)m_data.positionRaw);
        resetMinMax();
    }

    // -----------------------------------------------------------------------
    // update() – regelmaessig aus dem Sketch-loop() aufrufen
    // -----------------------------------------------------------------------

    void update() {
        const uint32_t now = millis();

        // ---- Manuell angeforderter Reset Comm (UI-Button "Ja") ----
        if (m_resetCommRequested && m_nmt) {
            m_resetCommRequested = false;
            Serial.println("[AP04] UI: Reset Communication + Start");
            m_nmt->sendResetComm();
            delay(50);
            m_nmt->sendStart();
            m_reconnectTimer = now;
            return;
        }

        // ---- Verbindungsstatus pruefen ----
        const bool alive = (m_data.lastRxMs != 0) &&
                           ((now - m_data.lastRxMs) < m_timeoutMs);

        // ---- [FIX #1] Auto-Reconnect bei Verbindungsabbruch ----
        if (!alive && m_nmt) {
            if (m_reconnectTimer == 0) {
                m_reconnectTimer = now;
                Serial.printf("[AP04] Verbindung verloren (lastRx vor %lu ms) – Reconnect-Timer gestartet\n",
                              (m_data.lastRxMs == 0) ? 0UL : (unsigned long)(now - m_data.lastRxMs));
            }
            if ((now - m_reconnectTimer) >= m_reconnectIntervalMs) {
                Serial.printf("[AP04] Auto-Reconnect (Intervall %lu ms) – Reset Comm + Start\n",
                              (unsigned long)m_reconnectIntervalMs);
                m_nmt->sendResetComm();
                delay(50);
                m_nmt->sendStart();
                m_reconnectTimer = now;
            }
        } else if (alive && m_reconnectTimer != 0) {
            // Nur dann als "wiederhergestellt" werten, wenn wir auch wirklich Operational sind.
            // Sonst (Heartbeat=0x7F Pre-Op) muessen wir weiter recovern.
            if (m_data.operational) {
                Serial.println("[AP04] Verbindung wiederhergestellt – Reconnect-Timer gestoppt");
                m_reconnectTimer = 0;
            }
        }

        // Wenn wir zwar alive sind, aber Heartbeat sagt Pre-Op (0x7F),
        // dann automatisch Recovery ausfuehren.
        // In der Praxis braucht die AP04 hier haeufig ResetComm + Start.
        if (alive && m_nmt && !m_data.operational) {
            if ((now - m_lastPreopLogMs) > 500) {
                Serial.println("[AP04] Pre-Op erkannt (Heartbeat 0x7F) – starte Recovery-Timer");
                m_lastPreopLogMs = now;
            }

            // NICHT auf m_reconnectTimer == 0 warten.
            // Der Timer kann durch andere Zweige geloescht/gesetzt werden.
            if ((now - m_lastRecoveryMs) >= m_reconnectIntervalMs) {
                Serial.println("[AP04] Heartbeat=Pre-Op -> Recovery: ResetComm + Start");
                m_nmt->sendResetComm();
                delay(50);
                m_nmt->sendStart();
                m_lastRecoveryMs = now;
            }
        }

        m_wasConnected   = alive;
        m_data.connected  = alive;
        // operational NICHT aus "alive" ableiten.
        // Das AP04 kann in Pre-Operational sein (z.B. Heartbeat 0x7F), obwohl Frames kommen.
        // Wir setzen operational stattdessen beim Heartbeat (NMT State = 0x05).
    }

    // -----------------------------------------------------------------------
    // processFrame() – aus dem CAN RX-Callback aufrufen
    // -----------------------------------------------------------------------

    void processFrame(uint32_t cobId, const uint8_t* data, uint8_t len) {
        if (!data) return;

        // TPDO1 oder TPDO2: Positionsdaten
        if (cobId == m_tpdo1 || cobId == m_tpdo2) {
            if (len >= 5) {
                const int32_t raw = (int32_t)(
                    ((uint32_t)data[0])       |
                    ((uint32_t)data[1] << 8)  |
                    ((uint32_t)data[2] << 16) |
                    ((uint32_t)data[3] << 24));

                m_data.positionRaw = raw;
                m_data.updateCount++;
                m_data.lastRxMs    = millis();

                // [FIX #2+#3] Position mit Software-Offset und scaleFactor berechnen
                const int32_t relRaw = raw - m_zeroOffset;
                m_data.positionMm    = (float)relRaw * m_scaleFactor;
                m_data.zeroOffset    = m_zeroOffset;

                // [FIX #4] Statusbyte vollstaendig dekodieren
                const uint8_t st   = data[4];
                m_data.statusByte  = st;
                m_data.inPos       = (st & AP04_STATUS::IN_POS)        != 0;
                m_data.istLtSoll   = (st & AP04_STATUS::IST_LT_SOLL)   != 0;
                m_data.batteryWarn = (st & AP04_STATUS::BATTERY_WARN)  != 0;
                m_data.batteryEmpty= (st & AP04_STATUS::BATTERY_EMPTY) != 0;
                m_data.chainSet    = (st & AP04_STATUS::CHAIN_SET)     != 0;
                m_data.arrowGt     = (st & AP04_STATUS::ARROW_GT)      != 0;
                m_data.arrowLt     = (st & AP04_STATUS::ARROW_LT)      != 0;

                // [FIX #2] Min/Max als relative Rohwerte speichern
                if (!m_data.hasMinMax) {
                    m_data.hasMinMax = true;
                    m_data.minRaw    = relRaw;
                    m_data.maxRaw    = relRaw;
                } else {
                    if (relRaw < m_data.minRaw) m_data.minRaw = relRaw;
                    if (relRaw > m_data.maxRaw) m_data.maxRaw = relRaw;
                }

                // Min/Max mm immer mit aktuellem scaleFactor berechnen
                m_data.minMm = (float)m_data.minRaw * m_scaleFactor;
                m_data.maxMm = (float)m_data.maxRaw * m_scaleFactor;
            }
            return;
        }

        // Heartbeat / Boot-Up: Verbindung aktiv halten + NMT-State auswerten
        // 0x00 = Boot-Up, 0x7F = Pre-Operational, 0x05 = Operational, 0x04 = Stopped
        if (cobId == m_hb) {
            m_data.lastRxMs = millis();
            if (len >= 1) {
                const uint8_t st = data[0];
                if (st == 0x7F) {
                    m_data.operational = false;
                } else if (st == 0x05) {
                    m_data.operational = true;
                }
            }
            return;
        }
    }

private:
    CanopenDriver* m_can;
    SdoClient*     m_sdo;
    NmtManager*    m_nmt;

    bool     m_resetCommRequested;

    uint8_t  m_nodeId;
    uint32_t m_tpdo1;
    uint32_t m_tpdo2;
    uint32_t m_hb;

    float    m_scaleFactor;

    // [FIX #3] Software-Nullpunkt (Rohwert-Offset)
    int32_t  m_zeroOffset;

    // [FIX #1] Auto-Reconnect Zustand
    uint32_t m_reconnectTimer;
    uint32_t m_reconnectIntervalMs;
    uint32_t m_timeoutMs;
    bool     m_wasConnected;

    // Recovery-Rate-Limit (separat vom reconnectTimer, damit Pre-Op Recovery nicht weggeloescht wird)
    uint32_t m_lastRecoveryMs = 0;
    uint32_t m_lastPreopLogMs = 0;

    AP04Data m_data;
};

#endif // SIKO_AP04_FIXED_H
