/**
 * @file siko_ap10.h
 * @brief SIKO AP10 Absolute Positionsanzeige – CANopen Treiber
 * @project SIKO AP10 Testprojekt
 * @date 2025
 *
 * Kapselt die gesamte Kommunikation mit der SIKO AP10 Positionsanzeige.
 * Nutzt NmtManager und SdoClient intern.
 *
 * Funktionen Phase 1:
 *   - Auto-Erkennung über Boot-Up Nachricht
 *   - NMT: Pre-Op → Operational
 *   - TPDO1 empfangen → IST-Position lesen
 *   - 0-Setzen (Preset via SDO Objekt 6003h)
 *   - Heartbeat-Überwachung
 *   - Min/Max-Merker
 *   - Geschwindigkeitsberechnung
 *
 * TPDO1 Aufbau (COB-ID = 0x180 + Node-ID):
 *   Byte 0-3: Positionswert (INT32, Little-Endian)
 *   Byte 4-5: Dummy 0x0000
 *   Byte 6-7: Zustandswort (5F19h)
 *
 * SPEICHERN ALS: src/devices/siko_ap10.h
 */

#ifndef SIKO_AP10_H
#define SIKO_AP10_H

#include <Arduino.h>
#include "../canopen/canopen_driver.h"
#include "../canopen/nmt_manager.h"
#include "../canopen/sdo_client.h"

// ============================================================================
// AP10 Objekt-Adressen (Objektverzeichnis)
// ============================================================================

namespace AP10_OBJ {
    // Standard CANopen Encoder-Profil (DS-406)
    constexpr uint16_t POSITION_VALUE          = 0x6004;  // IST-Position (Read)
    constexpr uint16_t PRESET_VALUE            = 0x6003;  // Kalibrierwert / 0-Setzen
    constexpr uint16_t OPERATING_PARAMS        = 0x6000;  // Betriebsparameter
    constexpr uint16_t STEPS_PER_REV           = 0x6001;  // Messschritte/Umdrehung
    constexpr uint16_t TOTAL_STEPS             = 0x6002;  // Gesamtmessschritte
    constexpr uint16_t ALARMS                  = 0x6503;  // Alarm-Flags
    constexpr uint16_t WARNINGS                = 0x6505;  // Warning-Flags
    constexpr uint16_t SERIAL_NUMBER           = 0x650B;  // Seriennummer

    // SIKO-spezifische Objekte
    constexpr uint16_t APP_OFFSET              = 0x2001;  // Applikationsoffset
    constexpr uint16_t CALIBRATE_VALUE         = 0x2002;  // Geberwert kalibrieren
    constexpr uint16_t CALIBRATE_ENABLE        = 0x2003;  // Freigabe Kalibrierung
    constexpr uint16_t CONTROL_WORD            = 0x5F0C;  // Steuerwort
    constexpr uint16_t STATUS_WORD             = 0x5F19;  // Zustandswort
    constexpr uint16_t NODE_ID_BAUDRATE        = 0x5F0A;  // Node-ID & Baudrate
    constexpr uint16_t DECIMAL_PLACES          = 0x5F11;  // Dezimalstellen
    constexpr uint16_t CYCLE_TIMER             = 0x6200;  // Zykluszeit TPDO1 (ms)
    constexpr uint16_t TPDO1_PARAM            = 0x1800;  // TPDO1 Parameter
}

// ============================================================================
// AP10 Zustandswort Bits (Objekt 5F19h, Byte 6-7 im TPDO)
// ============================================================================

namespace AP10_STATUS {
    constexpr uint16_t IN_POSITION      = (1 << 0);  // Innerhalb Zielfenster1
    constexpr uint16_t DIRECTION_CW     = (1 << 1);  // Richtung Uhrzeigersinn
    constexpr uint16_t DIRECTION_CCW    = (1 << 2);  // Richtung gegen Uhrzeigersinn
    constexpr uint16_t BATTERY_LOW      = (1 << 3);  // Batterie schwach
    constexpr uint16_t BATTERY_CRITICAL = (1 << 4);  // Batterie kritisch
    constexpr uint16_t ERROR            = (1 << 7);  // Allgemeiner Fehler
}

// ============================================================================
// AP10 Datenstruktur
// ============================================================================

struct AP10Data {
    bool     connected;          ///< Verbindung aktiv
    bool     operational;        ///< NMT Operational

    int32_t  position;           ///< IST-Position (Rohwert)
    float    positionMm;         ///< IST-Position in mm (nach Skalierung)
    int32_t  positionMin;        ///< Minimum seit Reset
    int32_t  positionMax;        ///< Maximum seit Reset
    float    velocityMmPerSec;   ///< Geschwindigkeit mm/s

    uint16_t statusWord;         ///< Zustandswort
    bool     inPosition;         ///< Innerhalb Zielfenster
    bool     batteryLow;         ///< Batterie-Warnung
    bool     hasError;           ///< Fehler aktiv

    uint32_t lastUpdateMs;       ///< Zeitstempel letztes Update
    uint32_t updateCount;        ///< Anzahl empfangener TPDO1

    // Geräteinformationen (via SDO gelesen)
    uint32_t serialNumber;
    uint32_t stepsPerRev;
    uint32_t totalSteps;
    bool     deviceInfoRead;

    AP10Data() {
        connected        = false;
        operational      = false;
        position         = 0;
        positionMm       = 0.0f;
        positionMin      = INT32_MAX;
        positionMax      = INT32_MIN;
        velocityMmPerSec = 0.0f;
        statusWord       = 0;
        inPosition       = false;
        batteryLow       = false;
        hasError         = false;
        lastUpdateMs     = 0;
        updateCount      = 0;
        serialNumber     = 0;
        stepsPerRev      = 0;
        totalSteps       = 0;
        deviceInfoRead   = false;
    }
};

// ============================================================================
// SIKO AP10 Klasse
// ============================================================================

class SikoAP10 {
private:
    CanopenDriver* m_driver;
    NmtManager     m_nmt;
    SdoClient      m_sdo;

    uint8_t  m_nodeId;
    AP10Data m_data;

    // Skalierung: Rohwert → mm
    // Wird aus stepsPerRev und mechanischer Auflösung berechnet
    // Standard: 1 Schritt = 1 Einheit (konfigurierbar)
    float    m_scaleFactor;      ///< mm pro Rohwert-Einheit
    uint8_t  m_decimalPlaces;    ///< Dezimalstellen der AP10

    // Geschwindigkeitsberechnung
    int32_t  m_lastPositionForVel;
    uint32_t m_lastVelTimeMs;

    // Startup-Sequenz
    enum StartupState {
        STARTUP_WAIT_BOOTUP,
        STARTUP_SEND_NMT_START,
        STARTUP_CONFIGURE_TPDO,
        STARTUP_READ_DEVICE_INFO,
        STARTUP_DONE
    };
    StartupState m_startupState;
    uint32_t     m_startupTimer;

    // Callback bei neuen Daten
    std::function<void(const AP10Data&)> m_dataCallback;

public:
    SikoAP10(CanopenDriver* driver, uint8_t nodeId = 125)
        : m_driver(driver)
        , m_nmt(driver, nodeId)
        , m_sdo(driver, nodeId)
        , m_nodeId(nodeId)
        , m_scaleFactor(1.0f)
        , m_decimalPlaces(0)
        , m_lastPositionForVel(0)
        , m_lastVelTimeMs(0)
        , m_startupState(STARTUP_WAIT_BOOTUP)
        , m_startupTimer(0)
        , m_dataCallback(nullptr)
    {
        // NMT Callback: Zustandsänderung
        m_nmt.setStateCallback([this](NmtState state) {
            Serial.printf("[AP10] NMT Zustand: %s\n", m_nmt.getStateName());
            m_data.operational = (state == NMT_OPERATIONAL);
            m_data.connected   = (state != NMT_UNKNOWN);
        });
    }

    // -----------------------------------------------------------------------
    // Konfiguration
    // -----------------------------------------------------------------------
    void setScaleFactor(float mmPerUnit) { m_scaleFactor = mmPerUnit; }
    void setNodeId(uint8_t id) {
        m_nodeId = id;
        m_nmt.setNodeId(id);
        m_sdo.setNodeId(id);
    }
    void setDataCallback(std::function<void(const AP10Data&)> cb) {
        m_dataCallback = cb;
    }

    const AP10Data& getData() const { return m_data; }
    bool isConnected()        const { return m_data.connected; }
    bool isOperational()      const { return m_data.operational; }

    // -----------------------------------------------------------------------
    // Eingehende CAN-Frames verarbeiten
    // Muss aus dem CAN RX-Callback aufgerufen werden!
    // -----------------------------------------------------------------------
    void processFrame(uint32_t cobId, const uint8_t* data, uint8_t length) {
        // NMT / Heartbeat
        m_nmt.processFrame(cobId, data, length);

        // SDO Response
        m_sdo.processFrame(cobId, data, length);

        // TPDO1: Positionswert (COB-ID = 0x180 + Node-ID)
        // Einige Geräte senden nur 5 Byte (4 Byte Position + 1 Byte Status) statt 8.
        if (cobId == (uint32_t)(0x180 + m_nodeId)) {
            if (length >= 8) {
                processTpdo1(data);
            } else if (length >= 5) {
                processTpdo1_len5(data);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Update (im loop() aufrufen, ca. alle 10ms)
    // -----------------------------------------------------------------------
    void update() {
        m_nmt.update();
        m_sdo.update();
        runStartupSequence();
    }

    // -----------------------------------------------------------------------
    // 0-Setzen: Aktuelle Position als Nullpunkt definieren
    // Schreibt den aktuellen Rohwert als Preset (Objekt 6003h)
    // -----------------------------------------------------------------------
    bool setZero() {
        if (!m_data.operational) {
            Serial.println("[AP10] setZero: Nicht operational!");
            return false;
        }

        Serial.printf("[AP10] 0-Setzen: Schreibe Preset = %ld\n", m_data.position);

        // Schreibe aktuellen Positionswert als Preset
        // Dadurch zeigt AP10 ab jetzt 0 an dieser Stelle an
        SdoResult r = m_sdo.writeBlocking(AP10_OBJ::PRESET_VALUE, 0x00,
                                           (uint32_t)m_data.position, 4, 1000);

        if (r == SDO_OK) {
            Serial.println("[AP10] 0-Setzen erfolgreich!");
            // Min/Max zurücksetzen
            resetMinMax();
            return true;
        } else {
            Serial.printf("[AP10] 0-Setzen FEHLER: %d\n", r);
            return false;
        }
    }

    // -----------------------------------------------------------------------
    // Min/Max Merker zurücksetzen
    // -----------------------------------------------------------------------
    void resetMinMax() {
        m_data.positionMin = m_data.position;
        m_data.positionMax = m_data.position;
        Serial.println("[AP10] Min/Max zurückgesetzt");
    }

private:
    // -----------------------------------------------------------------------
    // TPDO1 dekodieren (Variante mit 5 Byte: pos(int32 LE) + status(uint8))
    // -----------------------------------------------------------------------
    void processTpdo1_len5(const uint8_t* data) {
        const int32_t pos = (int32_t)(
            ((uint32_t)data[0]) |
            ((uint32_t)data[1] << 8) |
            ((uint32_t)data[2] << 16) |
            ((uint32_t)data[3] << 24));

        m_data.position     = pos;
        m_data.positionMm   = (float)pos * m_scaleFactor;
        m_data.lastUpdateMs = millis();
        m_data.updateCount++;

        // Bei 5-Byte-PDO liegt kein 16-bit StatusWord vor.
        // Wir nutzen Byte4 als "StatusByte" und mappen grob.
        const uint8_t st = data[4];
        m_data.statusWord = (uint16_t)st;
        m_data.inPosition = false;
        m_data.batteryLow = false;
        m_data.hasError   = false;

        // Min/Max
        if (m_data.positionMin == INT32_MAX && m_data.positionMax == INT32_MIN) {
            m_data.positionMin = pos;
            m_data.positionMax = pos;
        } else {
            if (pos < m_data.positionMin) m_data.positionMin = pos;
            if (pos > m_data.positionMax) m_data.positionMax = pos;
        }

        // Geschwindigkeit
        if (m_lastVelTimeMs == 0) {
            m_lastVelTimeMs = m_data.lastUpdateMs;
            m_lastPositionForVel = pos;
        } else {
            const uint32_t dtMs = m_data.lastUpdateMs - m_lastVelTimeMs;
            if (dtMs > 0) {
                const float dpMm = ((float)(pos - m_lastPositionForVel)) * m_scaleFactor;
                m_data.velocityMmPerSec = dpMm / ((float)dtMs / 1000.0f);
                m_lastVelTimeMs = m_data.lastUpdateMs;
                m_lastPositionForVel = pos;
            }
        }

        if (m_dataCallback) m_dataCallback(m_data);
    }

    // -----------------------------------------------------------------------
    // TPDO1 Zykluszeit konfigurieren (ms, 0 = aus)
    // Standard ab Werk: 0 (aus!) → muss aktiviert werden
    // -----------------------------------------------------------------------
    bool setTpdoCycleTime(uint16_t ms) {
        Serial.printf("[AP10] Setze TPDO1 Zykluszeit: %d ms\n", ms);
        SdoResult r = m_sdo.writeBlocking(AP10_OBJ::CYCLE_TIMER, 0x00, ms, 2, 1000);
        return (r == SDO_OK);
    }

    // -----------------------------------------------------------------------
    // Geräteinformationen lesen (Seriennummer, Auflösung)
    // -----------------------------------------------------------------------
    bool readDeviceInfo() {
        uint32_t val = 0;

        Serial.println("[AP10] Lese Geräteinformationen...");

        if (m_sdo.readBlocking(AP10_OBJ::SERIAL_NUMBER, 0x00, val) == SDO_OK) {
            m_data.serialNumber = val;
            Serial.printf("[AP10] Seriennummer: %lu\n", val);
        }

        if (m_sdo.readBlocking(AP10_OBJ::STEPS_PER_REV, 0x00, val) == SDO_OK) {
            m_data.stepsPerRev = val;
            Serial.printf("[AP10] Schritte/Umdrehung: %lu\n", val);
        }

        if (m_sdo.readBlocking(AP10_OBJ::TOTAL_STEPS, 0x00, val) == SDO_OK) {
            m_data.totalSteps = val;
            Serial.printf("[AP10] Gesamtschritte: %lu\n", val);
        }

        m_data.deviceInfoRead = true;
        return true;
    }

    // -----------------------------------------------------------------------
    // NMT Start manuell auslösen
    // -----------------------------------------------------------------------
    bool startNode() { return m_nmt.sendStart(); }
    bool resetNode() {
        m_startupState = STARTUP_WAIT_BOOTUP;
        return m_nmt.sendResetNode();
    }

private:
    // -----------------------------------------------------------------------
    // TPDO1 verarbeiten
    // Byte 0-3: Positionswert INT32 Little-Endian
    // Byte 4-5: Dummy
    // Byte 6-7: Zustandswort
    // -----------------------------------------------------------------------
    void processTpdo1(const uint8_t* data) {
        // Position (INT32, Little-Endian)
        int32_t rawPos = (int32_t)(
            ((uint32_t)data[0])       |
            ((uint32_t)data[1] << 8)  |
            ((uint32_t)data[2] << 16) |
            ((uint32_t)data[3] << 24)
        );

        // Zustandswort (Byte 6-7, Little-Endian)
        uint16_t statusWord = (uint16_t)(data[6]) | ((uint16_t)(data[7]) << 8);

        // Geschwindigkeit berechnen
        uint32_t now = millis();
        if (m_lastVelTimeMs > 0 && m_data.updateCount > 0) {
            float dt = (now - m_lastVelTimeMs) / 1000.0f;
            if (dt > 0.001f) {
                float dPos = (rawPos - m_lastPositionForVel) * m_scaleFactor;
                m_data.velocityMmPerSec = dPos / dt;
            }
        }
        m_lastPositionForVel = rawPos;
        m_lastVelTimeMs      = now;

        // Daten übernehmen
        m_data.position    = rawPos;
        m_data.positionMm  = rawPos * m_scaleFactor;
        m_data.statusWord  = statusWord;
        m_data.inPosition  = (statusWord & AP10_STATUS::IN_POSITION) != 0;
        m_data.batteryLow  = (statusWord & (AP10_STATUS::BATTERY_LOW |
                                             AP10_STATUS::BATTERY_CRITICAL)) != 0;
        m_data.hasError    = (statusWord & AP10_STATUS::ERROR) != 0;
        m_data.lastUpdateMs = now;
        m_data.updateCount++;

        // Min/Max aktualisieren
        if (rawPos < m_data.positionMin) m_data.positionMin = rawPos;
        if (rawPos > m_data.positionMax) m_data.positionMax = rawPos;

        // Callback
        if (m_dataCallback) {
            m_dataCallback(m_data);
        }
    }

    // -----------------------------------------------------------------------
    // Startup-Sequenz (nicht-blockierend)
    // -----------------------------------------------------------------------
    void runStartupSequence() {
        uint32_t now = millis();

        switch (m_startupState) {

            case STARTUP_WAIT_BOOTUP:
                // Warte auf Boot-Up Nachricht der AP10
                if (m_nmt.isBootUpReceived()) {
                    Serial.println("[AP10] Boot-Up erkannt → Starte Konfiguration");
                    m_startupTimer = now;
                    m_startupState = STARTUP_SEND_NMT_START;
                }
                break;

            case STARTUP_SEND_NMT_START:
                // Kurze Pause, dann NMT Start senden
                if (now - m_startupTimer > 200) {
                    m_nmt.sendStart();
                    m_startupTimer = now;
                    m_startupState = STARTUP_CONFIGURE_TPDO;
                }
                break;

            case STARTUP_CONFIGURE_TPDO:
                // Warte bis Operational, dann TPDO1 aktivieren
                if (now - m_startupTimer > 500) {
                    if (m_data.operational) {
                        // TPDO1 Zykluszeit auf 100ms setzen
                        // (ab Werk ist TPDO1 deaktiviert!)
                        setTpdoCycleTime(100);
                        m_startupTimer = now;
                        m_startupState = STARTUP_READ_DEVICE_INFO;
                    } else {
                        // Nochmal versuchen
                        m_nmt.sendStart();
                        m_startupTimer = now;
                    }
                }
                break;

            case STARTUP_READ_DEVICE_INFO:
                // Geräteinformationen lesen
                if (now - m_startupTimer > 300) {
                    readDeviceInfo();
                    m_startupState = STARTUP_DONE;
                    Serial.println("[AP10] ✓ Startup abgeschlossen!");
                }
                break;

            case STARTUP_DONE:
                // Nichts mehr zu tun
                break;
        }
    }
};

#endif // SIKO_AP10_H
