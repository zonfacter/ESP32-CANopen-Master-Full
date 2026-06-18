#pragma once

#include <Arduino.h>
#include <vector>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// -----------------------------
// Core data structures
// -----------------------------

struct SniffFrame {
    uint32_t timestampMs = 0;
    uint32_t id = 0;
    uint8_t  dlc = 0;
    uint8_t  data[8] = {0};
    bool     ext = false;
    bool     rtr = false;
};

struct DecodedFrame {
    SniffFrame raw;

    // cheap-to-copy fields
    const char* type = ""; // e.g. "HB", "BootUp", "SDO_Req", "SDO_Resp", "PDO", "EMCY", "NMT", "SYNC", "TIME", "RTR", "UNK"
    uint8_t nodeId = 0;
    uint16_t baseId = 0;
};

class SnifferManager {
public:
    enum class FilterType : uint8_t {
        All = 0,
        Heartbeat,
        SDO,
        PDO,
        EMCY,
        NMT,
        SYNC,
        TIME,
        Unknown,
    };

    struct Config {
        uint16_t queueLen = 128;        // 64..256 typically
        uint16_t maxRecentFrames = 120; // UI log size
        uint32_t uiPollMinMs = 100;     // recommended UI refresh cadence
        uint32_t noTrafficWarnMs = 1000;
        uint16_t serialPrintMaxPerSecond = 40;
    };

    SnifferManager();

    void begin(const Config& cfg);
    void begin();
    void end();

    void setEnabled(bool en);
    bool enabled() const { return _enabled; }

    // Feed from your CAN RX callback (task context; not ISR)
    void processFrame(uint32_t id, uint8_t dlc, const uint8_t* data, bool ext, bool rtr);

    // Health
    uint32_t getDroppedCount() const { return _droppedCount; }
    uint32_t getLastFrameTimeMs() const { return _lastFrameTimeMs; }
    uint32_t getLastTrafficAgeMs() const;
    uint16_t getQueueDepth() const;
    uint16_t getQueueHighWatermark() const { return _queueHighWatermark; }

    // UI access (copy out)
    std::vector<DecodedFrame> getRecentFramesCopy();
    uint16_t copyRecentNewest(DecodedFrame* out, uint16_t maxOut, uint16_t* totalAvailable = nullptr);
    void clearRecent();
    void clearStats();

    // Filters (applied in decode/task stage)
    void setNodeIdFilter(uint8_t nodeIdOr0);
    void setTypeFilter(FilterType ft);
    void setIdRangeFilter(bool enabled, uint32_t idMin, uint32_t idMax);

    // Serial output (optional)
    void setSerialOutput(bool en);

private:
    static void taskThunk(void* arg);
    void taskLoop();

    void decodeAndStore(const SniffFrame& f);
    void printDecodedSerialThrottled(const DecodedFrame& df);
    static const char* classifyType(const SniffFrame& f, uint16_t baseId, uint8_t nodeId);
    bool passFilters(const DecodedFrame& df) const;

private:
    Config _cfg;

    // runtime
    bool _enabled = false;
    bool _serialOutput = true;
    uint32_t _serialWindowStartMs = 0;
    uint16_t _serialWindowCount = 0;
    uint32_t _serialSuppressedCount = 0;

    // queue
    QueueHandle_t _q = nullptr;
    TaskHandle_t _task = nullptr;

    // health
    volatile uint32_t _droppedCount = 0;
    volatile uint32_t _lastFrameTimeMs = 0;
    volatile uint16_t _queueHighWatermark = 0;

    // recent frames: fixed-size circular buffer, copied out as oldest->newest
    std::vector<DecodedFrame> _recentRing;
    uint16_t _recentHead = 0;   // next write position
    uint16_t _recentCount = 0;  // valid entries
    portMUX_TYPE _recentMux = portMUX_INITIALIZER_UNLOCKED;

    // filters
    uint8_t _nodeIdFilter = 0; // 0=all
    FilterType _typeFilter = FilterType::All;
    bool _idRangeEnabled = false;
    uint32_t _idMin = 0;
    uint32_t _idMax = 0x7FF;
};
