/**
 * @file canopen_driver.h
 * @brief CANopen Treiber über ESP32 TWAI
 * @project SIKO AP10 Testprojekt
 * @date 2025
 *
 * Kapselt die TWAI-Hardware für CANopen-Kommunikation.
 * Sendet und empfängt CAN-Frames, stellt Callbacks bereit.
 */

#ifndef CANOPEN_DRIVER_H
#define CANOPEN_DRIVER_H

#include <Arduino.h>
#include "driver/twai.h"
#include <functional>

// ============================================================================
// Callback-Typen
// ============================================================================

/**
 * @brief Callback für empfangene CAN-Frames
 * @param cobId  COB-ID (11-Bit Identifier)
 * @param data   Zeiger auf Daten (max. 8 Byte)
 * @param length Datenlänge
 */
using CanFrameCallback = std::function<void(uint32_t cobId, const uint8_t* data, uint8_t length)>;

// ============================================================================
// CANopen Driver Klasse
// ============================================================================

class CanopenDriver {
private:
    bool             m_initialized;
    bool             m_running;
    uint8_t          m_txPin;
    uint8_t          m_rxPin;
    uint32_t         m_baudrate;
    TaskHandle_t     m_rxTask;
    CanFrameCallback m_rxCallback;

    uint32_t m_rxCount;
    uint32_t m_txCount;
    uint32_t m_errorCount;

    // -----------------------------------------------------------------------
    // Hilfsfunktion: TWAI Status ausgeben
    // -----------------------------------------------------------------------
    static void printTwaiStatus(const char* tag)
    {
        twai_status_info_t s;
        esp_err_t e = twai_get_status_info(&s);
        if (e == ESP_OK) {
            Serial.printf("[%s] state=%d txq=%lu rxq=%lu txerr=%u rxerr=%u bus_err=%lu arb_lost=%lu\n",
                          tag,
                          (int)s.state,
                          (unsigned long)s.msgs_to_tx,
                          (unsigned long)s.msgs_to_rx,
                          (unsigned)s.tx_error_counter,
                          (unsigned)s.rx_error_counter,
                          (unsigned long)s.bus_error_count,
                          (unsigned long)s.arb_lost_count);
        } else {
            Serial.printf("[%s] twai_get_status_info failed: %d\n", tag, (int)e);
        }
    }

    // -----------------------------------------------------------------------
    // RX-Task (FreeRTOS)
    // -----------------------------------------------------------------------
    static void rxTaskFunction(void* param)
    {
        CanopenDriver* drv = static_cast<CanopenDriver*>(param);
        twai_message_t msg;

        Serial.println("[CANopen] RX-Task gestartet");

        while (drv->m_running) {
            esp_err_t err = twai_receive(&msg, pdMS_TO_TICKS(100));

            if (err == ESP_OK) {
                drv->m_rxCount++;
                if (drv->m_rxCallback) {
                    drv->m_rxCallback(msg.identifier, msg.data, msg.data_length_code);
                }
            } else if (err != ESP_ERR_TIMEOUT) {
                drv->m_errorCount++;

                twai_status_info_t status;
                if (twai_get_status_info(&status) == ESP_OK) {
                    if (status.state == TWAI_STATE_BUS_OFF) {
                        Serial.println("[CANopen] Bus-Off! Versuche Recovery...");
                        twai_initiate_recovery();
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        Serial.println("[CANopen] RX-Task beendet");
        vTaskDelete(nullptr);
    }

public:
    CanopenDriver()
        : m_initialized(false)
        , m_running(false)
        , m_txPin(0)
        , m_rxPin(0)
        , m_baudrate(0)
        , m_rxTask(nullptr)
        , m_rxCallback(nullptr)
        , m_rxCount(0)
        , m_txCount(0)
        , m_errorCount(0)
    {}

    ~CanopenDriver() { deinit(); }

    // -----------------------------------------------------------------------
    // Initialisierung
    // -----------------------------------------------------------------------
    // ... existing code ...
    void stop()
    {
        if (!m_running) return;
        m_running = false;
        
        // Erzwinge das Stoppen des Treibers
        twai_stop();
        
        // Wichtig: Kurze Pause, damit die Hardware den aktuellen Status verarbeitet
        delay(10); 
        Serial.println("[CANopen] Gestoppt");
    }

    void deinit()
    {
        stop(); 
        
        if (m_initialized) {
            // Vor dem Uninstall prüfen, ob der Treiber noch läuft und stoppen
            twai_stop();
            delay(20); // Kurze Pufferzeit für Hardware-Freigabe

            esp_err_t e = twai_driver_uninstall();
            if (e != ESP_OK) {
                Serial.printf("[CANopen] Warnung: twai_driver_uninstall fehlgeschlagen: %d\n", (int)e);
            } else {
                m_initialized = false;
                Serial.println("[CANopen] Treiber deinstalliert");
            }
            
            // Falls Uninstall fehlschlägt, setzen wir m_initialized trotzdem auf false, 
            // um den nächsten Init-Versuch zu erlauben (mit Retry-Logik).
            m_initialized = false;
        }
    }

    bool init(uint8_t txPin, uint8_t rxPin, uint32_t baudrate = 250000)
    {
        // Wenn bereits initialisiert, sauberer Abbruch erzwingen
        if (m_initialized) {
            deinit();
            delay(100); 
        }

        m_txPin    = txPin;
        m_rxPin    = rxPin;
        m_baudrate = baudrate;

        Serial.printf("[CANopen] Init TX=%d RX=%d @ %lu bps\n", txPin, rxPin, (unsigned long)baudrate);

        twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)txPin, (gpio_num_t)rxPin, TWAI_MODE_NORMAL);
        
        // Reduziere die Queue-Größen auf das absolute Minimum für Scan-Operationen
        g_cfg.rx_queue_len = 8;  
        g_cfg.tx_queue_len = 4;  

        twai_timing_config_t t_cfg;
        // ... switch baudrate bleibt gleich ...
        switch (baudrate) {
            case 10000:   t_cfg = TWAI_TIMING_CONFIG_10KBITS();   break;
            case 20000:   t_cfg = TWAI_TIMING_CONFIG_20KBITS();   break;
            case 50000:   t_cfg = TWAI_TIMING_CONFIG_50KBITS();   break;
            case 100000:  t_cfg = TWAI_TIMING_CONFIG_100KBITS();  break;
            case 125000:  t_cfg = TWAI_TIMING_CONFIG_125KBITS();  break;
            case 250000:  t_cfg = TWAI_TIMING_CONFIG_250KBITS();  break;
            case 500000:  t_cfg = TWAI_TIMING_CONFIG_500KBITS();  break;
            case 1000000: t_cfg = TWAI_TIMING_CONFIG_1MBITS();     break;
            default:      t_cfg = TWAI_TIMING_CONFIG_250KBITS();  break;
        }

        twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        esp_err_t e;
        // Erhöhe die Retries und füge eine kleine Verzögerung zwischen den Versuchen hinzu
        for (int i = 0; i < 5; i++) {
            e = twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
            if (e == ESP_OK) break;
            
            Serial.printf("[CANopen] Versuch %d fehlgeschlagen (Err: %d). Retrying...\n", i + 1, (int)e);
            delay(100); // Längere Pause zwischen den Versuchen bei Fehler 259
        }

        if (e != ESP_OK) {
            Serial.printf("[CANopen] KRITISCH: twai_driver_install fehlgeschlagen nach %d Versuchen: %d\n", 5, (int)e);
            return false;
        }

        m_initialized = true;
        Serial.println("[CANopen] Treiber erfolgreich installiert");
        return true;
    }

    bool start()
    {
        if (!m_initialized || m_running) return false;

        esp_err_t e = twai_start();
        if (e != ESP_OK) {
            Serial.printf("[CANopen] ERROR: twai_start fehlgeschlagen: %d\n", (int)e);
            return false;
        }

        m_running = true;

        xTaskCreatePinnedToCore(
            rxTaskFunction, "canopen_rx", 4096, this, 3, &m_rxTask, 1);

        Serial.println("[CANopen] Gestartet");
        printTwaiStatus("TWAI");
        return true;
    }

    // -----------------------------------------------------------------------
    // Callback registrieren
    // -----------------------------------------------------------------------
    void setRxCallback(CanFrameCallback cb) { m_rxCallback = cb; }

    // -----------------------------------------------------------------------
    // Frame senden
    // -----------------------------------------------------------------------
    bool sendFrame(uint32_t cobId, const uint8_t* data, uint8_t length, bool rtr = false)
    {
        if (!m_running) return false;
        if (length > 8)  return false;

        twai_message_t msg = {};
        msg.identifier       = cobId;
        msg.data_length_code = length;
        msg.extd             = 0;  // 11-Bit Standard Frame
        msg.rtr              = rtr ? 1 : 0;

        if (!rtr && data && length > 0) {
            memcpy(msg.data, data, length);
        }

        // Timeout etwas höher, um Bus/ACK-Probleme zu erkennen
        esp_err_t e = twai_transmit(&msg, pdMS_TO_TICKS(200));
        if (e == ESP_OK) {
            m_txCount++;
            return true;
        }

        m_errorCount++;
        Serial.printf("[CANopen] TX Fehler COB-ID=0x%03lX err=%d\n", (unsigned long)cobId, (int)e);
        printTwaiStatus("TWAI");
        return false;
    }

    // Kurzform ohne Daten (z.B. NMT)
    bool sendFrame(uint32_t cobId,
                   uint8_t b0, uint8_t b1 = 0,
                   uint8_t b2 = 0, uint8_t b3 = 0,
                   uint8_t b4 = 0, uint8_t b5 = 0,
                   uint8_t b6 = 0, uint8_t b7 = 0,
                   uint8_t len = 2)
    {
        uint8_t d[8] = {b0, b1, b2, b3, b4, b5, b6, b7};
        return sendFrame(cobId, d, len);
    }

    // -----------------------------------------------------------------------
    // Statistik
    // -----------------------------------------------------------------------
    uint32_t getRxCount()    const { return m_rxCount; }
    uint32_t getTxCount()    const { return m_txCount; }
    uint32_t getErrorCount() const { return m_errorCount; }
    bool     isRunning()     const { return m_running; }
};

#endif // CANOPEN_DRIVER_H
