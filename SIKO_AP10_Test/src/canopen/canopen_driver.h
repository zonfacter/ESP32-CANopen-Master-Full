/**
 * @file canopen_driver_loopback.h
 * @brief CANopen Treiber über ESP32 TWAI (fixed) + optional Loopback/No-ACK mode
 *
 * Based on canopen_driver_fixed.h, extended with:
 * - setLoopbackEnabled(bool): installs TWAI in TWAI_MODE_NO_ACK when enabled.
 *   This allows local transmit without requiring another node to ACK.
 */

#ifndef CANOPEN_DRIVER_LOOPBACK_H
#define CANOPEN_DRIVER_LOOPBACK_H

#include <Arduino.h>
#include "driver/twai.h"
#include <functional>

using CanFrameCallback = std::function<void(uint32_t cobId, const uint8_t* data, uint8_t length)>;

class CanopenDriver {
private:
    bool             m_initialized;
    bool             m_running;
    bool             m_rxTaskEnabled;

    bool             m_loopbackNoAck;

    uint8_t          m_txPin;
    uint8_t          m_rxPin;
    uint32_t         m_baudrate;

    TaskHandle_t     m_rxTask;
    TaskHandle_t     m_stopWaiter;

    int              m_rxTaskCore;

    CanFrameCallback m_rxCallback;

    uint32_t m_rxCount;
    uint32_t m_txCount;
    uint32_t m_errorCount;
    uint32_t m_recoveryCount;
    bool     m_recoveryInProgress;
    bool     m_busOffLatched;
    uint32_t m_recoveryStartMs;
    uint32_t m_lastRecoveryAttemptMs;

    static constexpr uint32_t RECOVERY_RETRY_MS = 1000;
    static constexpr uint32_t RECOVERY_HARD_RESET_MS = 5000;

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
                twai_status_info_t status;
                if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_BUS_OFF) {
                    drv->noteBusOff("rx");
                } else {
                    drv->m_errorCount++;
                }
            }

            vTaskDelay(pdMS_TO_TICKS(1));
        }

        Serial.println("[CANopen] RX-Task beendet");

        if (drv->m_stopWaiter) {
            xTaskNotifyGive(drv->m_stopWaiter);
        }

        drv->m_rxTask = nullptr;
        vTaskDelete(nullptr);
    }

    bool installDriver(uint32_t baudrate)
    {
        const twai_mode_t mode = m_loopbackNoAck ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL;

        twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)m_txPin, (gpio_num_t)m_rxPin, mode);

        g_cfg.rx_queue_len = 8;
        g_cfg.tx_queue_len = 4;

        twai_timing_config_t t_cfg;
        switch (baudrate) {
            // 10k/20k need a large baud-rate prescaler. The convenience macros are
            // only defined when SOC_TWAI_BRP_MAX is big enough; on cores/SoCs where
            // they are absent these cases are compiled out (and fall to default),
            // instead of breaking the build.
#if defined(TWAI_TIMING_CONFIG_10KBITS)
            case 10000:   t_cfg = TWAI_TIMING_CONFIG_10KBITS();   break;
#endif
#if defined(TWAI_TIMING_CONFIG_20KBITS)
            case 20000:   t_cfg = TWAI_TIMING_CONFIG_20KBITS();   break;
#endif
            case 50000:   t_cfg = TWAI_TIMING_CONFIG_50KBITS();   break;
            case 100000:  t_cfg = TWAI_TIMING_CONFIG_100KBITS();  break;
            case 125000:  t_cfg = TWAI_TIMING_CONFIG_125KBITS();  break;
            case 250000:  t_cfg = TWAI_TIMING_CONFIG_250KBITS();  break;
            case 500000:  t_cfg = TWAI_TIMING_CONFIG_500KBITS();  break;
            case 1000000: t_cfg = TWAI_TIMING_CONFIG_1MBITS();    break;
            default:
                Serial.printf("[CANopen] WARN: baud %lu not supported here -> using 250k\n",
                              (unsigned long)baudrate);
                t_cfg = TWAI_TIMING_CONFIG_250KBITS();
                break;
        }

        twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        esp_err_t e = ESP_FAIL;
        for (int i = 0; i < 5; i++) {
            e = twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
            if (e == ESP_OK) return true;
            Serial.printf("[CANopen] twai_driver_install Versuch %d fehlgeschlagen: %d\n", i + 1, (int)e);
            delay(100);
        }

        Serial.printf("[CANopen] KRITISCH: twai_driver_install fehlgeschlagen: %d\n", (int)e);
        return false;
    }

    void requestRecovery(const char* reason)
    {
        if (!m_initialized) return;

        const uint32_t now = millis();
        if (m_recoveryInProgress) return;
        if (now - m_lastRecoveryAttemptMs < RECOVERY_RETRY_MS) return;

        m_lastRecoveryAttemptMs = now;
        esp_err_t e = twai_initiate_recovery();
        if (e == ESP_OK) {
            m_recoveryInProgress = true;
            m_recoveryStartMs = now;
            Serial.printf("[CANopen] Bus-Off Recovery gestartet (%s)\n", reason ? reason : "?");
        } else if (e != ESP_ERR_INVALID_STATE) {
            Serial.printf("[CANopen] WARN: twai_initiate_recovery fehlgeschlagen (%s): %d\n",
                          reason ? reason : "?", (int)e);
        }
    }

    void noteBusOff(const char* reason)
    {
        if (!m_busOffLatched) {
            m_busOffLatched = true;
            m_errorCount++;
            Serial.printf("[CANopen] Bus-Off erkannt (%s)\n", reason ? reason : "?");
        }
        requestRecovery(reason);
    }

    void hardResetAfterRecoveryTimeout()
    {
        Serial.println("[CANopen] WARN: Bus-Off Recovery Timeout -> harter TWAI Re-Init");
        const uint8_t tx = m_txPin;
        const uint8_t rx = m_rxPin;
        const uint32_t baud = m_baudrate;
        deinit();
        delay(50);
        if (init(tx, rx, baud)) {
            start();
        }
    }

public:
    CanopenDriver()
        : m_initialized(false)
        , m_running(false)
        , m_rxTaskEnabled(true)
        , m_loopbackNoAck(false)
        , m_txPin(0)
        , m_rxPin(0)
        , m_baudrate(0)
        , m_rxTask(nullptr)
        , m_stopWaiter(nullptr)
        , m_rxTaskCore(0)
        , m_rxCallback(nullptr)
        , m_rxCount(0)
        , m_txCount(0)
        , m_errorCount(0)
        , m_recoveryCount(0)
        , m_recoveryInProgress(false)
        , m_busOffLatched(false)
        , m_recoveryStartMs(0)
        , m_lastRecoveryAttemptMs(0)
    {}

    ~CanopenDriver() { deinit(); }

    void enableRxTask(bool en) { m_rxTaskEnabled = en; }
    void setRxTaskCore(int core) { m_rxTaskCore = core; }

    // Loopback/No-ACK mode must be set before init() (or require re-init)
    void setLoopbackEnabled(bool en) {
        if (m_initialized) {
            Serial.println("[CANopen] INFO: Loopback change requires re-init; calling deinit()");
            deinit();
        }
        m_loopbackNoAck = en;
        Serial.printf("[CANopen] Loopback(No-ACK) %s\n", en ? "EN" : "DIS");
    }

    bool isLoopbackEnabled() const { return m_loopbackNoAck; }

    void stop(uint32_t timeoutMs = 500)
    {
        if (!m_running) return;

        m_stopWaiter = xTaskGetCurrentTaskHandle();
        m_running = false;

        twai_stop();

        if (m_rxTaskEnabled && m_rxTask != nullptr) {
            uint32_t ok = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeoutMs));
            if (!ok) {
                Serial.println("[CANopen] WARN: RX-Task stop timeout");
            }
        }

        m_stopWaiter = nullptr;
        Serial.println("[CANopen] Gestoppt");
    }

    void deinit()
    {
        stop();
        if (!m_initialized) return;

        esp_err_t e = twai_driver_uninstall();
        if (e != ESP_OK) {
            Serial.printf("[CANopen] WARN: twai_driver_uninstall fehlgeschlagen: %d\n", (int)e);
            printTwaiStatus("TWAI");

            twai_stop();
            delay(50);
            e = twai_driver_uninstall();
            Serial.printf("[CANopen] Uninstall retry: %d\n", (int)e);
        } else {
            Serial.println("[CANopen] Treiber deinstalliert");
        }

        m_initialized = false;
        m_recoveryInProgress = false;
        m_busOffLatched = false;
        m_recoveryStartMs = 0;
    }

    bool init(uint8_t txPin, uint8_t rxPin, uint32_t baudrate = 250000)
    {
        if (m_initialized) {
            deinit();
            delay(50);
        }

        m_txPin = txPin;
        m_rxPin = rxPin;
        m_baudrate = baudrate;
        m_recoveryInProgress = false;
        m_busOffLatched = false;
        m_recoveryStartMs = 0;

        Serial.printf("[CANopen] Init TX=%d RX=%d @ %lu bps (mode=%s)\n",
                      txPin, rxPin, (unsigned long)baudrate,
                      m_loopbackNoAck ? "NO_ACK" : "NORMAL");

        if (!installDriver(baudrate)) return false;

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

        if (m_rxTaskEnabled) {
            BaseType_t ret = xTaskCreatePinnedToCore(
                rxTaskFunction, "canopen_rx", 4096, this, 3, &m_rxTask,
                (m_rxTaskCore < 0) ? tskNO_AFFINITY : m_rxTaskCore);

            if (ret != pdPASS) {
                Serial.println("[CANopen] ERROR: RX-Task konnte nicht erstellt werden");
                m_running = false;
                twai_stop();
                return false;
            }
        } else {
            m_rxTask = nullptr;
        }

        Serial.println("[CANopen] Gestartet");
        m_busOffLatched = false;
        printTwaiStatus("TWAI");
        return true;
    }

    void setRxCallback(CanFrameCallback cb) { m_rxCallback = cb; }

    bool sendFrame(uint32_t cobId, const uint8_t* data, uint8_t length)
    {
        if (!m_running) return false;

        twai_message_t msg = {};
        msg.identifier = cobId;
        msg.data_length_code = length;
        msg.flags = 0;
        if (data && length) memcpy(msg.data, data, length);

        esp_err_t e = twai_transmit(&msg, pdMS_TO_TICKS(50));
        if (e == ESP_OK) {
            m_txCount++;
            return true;
        }

        Serial.printf("[CANopen] TX Fehler COB-ID=0x%03lX err=%d\n", (unsigned long)cobId, (int)e);
        printTwaiStatus("TWAI");
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK && status.state == TWAI_STATE_BUS_OFF) {
            noteBusOff("tx");
        } else {
            m_errorCount++;
        }
        return false;
    }

    uint32_t rxCount() const { return m_rxCount; }
    uint32_t txCount() const { return m_txCount; }
    uint32_t errCount() const { return m_errorCount; }
    uint32_t recoveryCount() const { return m_recoveryCount; }
    bool recoveryInProgress() const { return m_recoveryInProgress; }

    const char* stateText() const
    {
        if (!m_initialized) return "Not initialized";
        if (!m_running) return "Stopped";
        if (m_recoveryInProgress) return "Recovering";

        twai_status_info_t status;
        if (twai_get_status_info(&status) != ESP_OK) return "Status error";

        switch (status.state) {
            case TWAI_STATE_STOPPED:    return "Stopped";
            case TWAI_STATE_RUNNING:    return "Running";
            case TWAI_STATE_BUS_OFF:    return "Bus-Off";
            case TWAI_STATE_RECOVERING: return "Recovering";
            default:                    return "Unknown";
        }
    }

    void service()
    {
        if (!m_initialized || !m_running) return;

        twai_status_info_t status;
        if (twai_get_status_info(&status) != ESP_OK) return;

        const uint32_t now = millis();
        if (status.state == TWAI_STATE_BUS_OFF) {
            noteBusOff("service");
            return;
        }

        if (!m_recoveryInProgress) return;

        if (status.state == TWAI_STATE_STOPPED) {
            esp_err_t e = twai_start();
            if (e == ESP_OK) {
                m_recoveryInProgress = false;
                m_busOffLatched = false;
                m_recoveryCount++;
                Serial.printf("[CANopen] Bus-Off Recovery abgeschlossen (count=%lu)\n",
                              (unsigned long)m_recoveryCount);
                printTwaiStatus("TWAI");
            } else {
                Serial.printf("[CANopen] WARN: twai_start nach Recovery fehlgeschlagen: %d\n", (int)e);
            }
            return;
        }

        if (now - m_recoveryStartMs > RECOVERY_HARD_RESET_MS) {
            m_recoveryInProgress = false;
            m_busOffLatched = false;
            hardResetAfterRecoveryTimeout();
        }
    }

    // True while it is safe to keep transmitting. Used by active auto-scan to
    // stop probing a wrong baudrate before the unacked TX pushes us to bus-off.
    bool busHealthyForTx() const
    {
        if (!m_running) return false;
        twai_status_info_t s;
        if (twai_get_status_info(&s) != ESP_OK) return false;
        if (s.state == TWAI_STATE_BUS_OFF) return false;
        return s.tx_error_counter < 96;   // below error-passive (128) / bus-off (256)
    }

    bool pollReceive(uint32_t timeoutMs, uint32_t& outCobId, uint8_t* outData, uint8_t& outLen)
    {
        if (!m_initialized) return false;

        twai_message_t msg;
        esp_err_t e = twai_receive(&msg, pdMS_TO_TICKS(timeoutMs));
        if (e != ESP_OK) return false;

        outCobId = msg.identifier;
        outLen = msg.data_length_code;
        if (outLen > 8) outLen = 8;
        if (outData && outLen) memcpy(outData, msg.data, outLen);
        return true;
    }
};

#endif // CANOPEN_DRIVER_LOOPBACK_H
