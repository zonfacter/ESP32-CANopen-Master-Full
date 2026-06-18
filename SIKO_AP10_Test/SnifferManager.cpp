#include "SnifferManager.h"

static inline uint8_t clampDlc(uint8_t dlc) { return (dlc > 8) ? 8 : dlc; }

SnifferManager::SnifferManager() {}

void SnifferManager::begin(const Config& cfg)
{
    _cfg = cfg;

    if (_cfg.queueLen < 16) _cfg.queueLen = 16;
    if (_cfg.maxRecentFrames < 16) _cfg.maxRecentFrames = 16;

    _recentRing.clear();
    _recentRing.resize(_cfg.maxRecentFrames);
    _recentHead = 0;
    _recentCount = 0;

    if (_q) {
        vQueueDelete(_q);
        _q = nullptr;
    }

    _q = xQueueCreate(_cfg.queueLen, sizeof(SniffFrame));

    if (_task) {
        vTaskDelete(_task);
        _task = nullptr;
    }

    // Core choice: keep LVGL mostly on core 1; run sniffer on core 0.
    xTaskCreatePinnedToCore(taskThunk, "sniffer", 4096, this, 2, &_task, 0);
}

void SnifferManager::begin()
{
    begin(Config{});
}

void SnifferManager::end()
{
    setEnabled(false);

    if (_task) {
        vTaskDelete(_task);
        _task = nullptr;
    }
    if (_q) {
        vQueueDelete(_q);
        _q = nullptr;
    }

    portENTER_CRITICAL(&_recentMux);
    _recentHead = 0;
    _recentCount = 0;
    portEXIT_CRITICAL(&_recentMux);
}

void SnifferManager::setEnabled(bool en)
{
    if (_enabled == en) return;

    if (en) {
        if (_q) xQueueReset(_q);
        _lastFrameTimeMs = 0;
        _queueHighWatermark = 0;
        _enabled = true;
    } else {
        _enabled = false;
        if (_q) xQueueReset(_q);
    }
}

void SnifferManager::processFrame(uint32_t id, uint8_t dlc, const uint8_t* data, bool ext, bool rtr)
{
    if (!_enabled || !_q) return;

    SniffFrame f;
    f.timestampMs = millis();
    f.id = id;
    f.dlc = clampDlc(dlc);
    f.ext = ext;
    f.rtr = rtr;

    if (data && f.dlc) {
        memcpy(f.data, data, f.dlc);
    }

    // Non-blocking enqueue
    if (xQueueSend(_q, &f, 0) != pdTRUE) {
        _droppedCount++;
    } else {
        const UBaseType_t waiting = uxQueueMessagesWaiting(_q);
        if (waiting > _queueHighWatermark) {
            _queueHighWatermark = (uint16_t)waiting;
        }
    }
}

uint32_t SnifferManager::getLastTrafficAgeMs() const
{
    const uint32_t last = _lastFrameTimeMs;
    if (last == 0) return 0xFFFFFFFFu;
    return (uint32_t)(millis() - last);
}

uint16_t SnifferManager::getQueueDepth() const
{
    if (!_q) return 0;
    return (uint16_t)uxQueueMessagesWaiting(_q);
}

std::vector<DecodedFrame> SnifferManager::getRecentFramesCopy()
{
    portENTER_CRITICAL(&_recentMux);
    std::vector<DecodedFrame> copy;
    copy.reserve(_recentCount);
    const uint16_t cap = (uint16_t)_recentRing.size();
    if (cap > 0) {
        const uint16_t start = (_recentCount == cap) ? _recentHead : 0;
        for (uint16_t i = 0; i < _recentCount; ++i) {
            copy.push_back(_recentRing[(start + i) % cap]);
        }
    }
    portEXIT_CRITICAL(&_recentMux);
    return copy;
}

uint16_t SnifferManager::copyRecentNewest(DecodedFrame* out, uint16_t maxOut, uint16_t* totalAvailable)
{
    portENTER_CRITICAL(&_recentMux);
    const uint16_t cap = (uint16_t)_recentRing.size();
    const uint16_t count = _recentCount;
    if (totalAvailable) *totalAvailable = count;

    uint16_t copied = 0;
    if (out && maxOut > 0 && cap > 0 && count > 0) {
        const uint16_t n = (count < maxOut) ? count : maxOut;
        for (uint16_t i = 0; i < n; ++i) {
            const uint16_t newestOffset = (uint16_t)(i + 1);
            const uint16_t idx = (uint16_t)((_recentHead + cap - newestOffset) % cap);
            out[i] = _recentRing[idx];
        }
        copied = n;
    }
    portEXIT_CRITICAL(&_recentMux);
    return copied;
}

void SnifferManager::clearRecent()
{
    portENTER_CRITICAL(&_recentMux);
    _recentHead = 0;
    _recentCount = 0;
    portEXIT_CRITICAL(&_recentMux);
}

void SnifferManager::clearStats()
{
    _droppedCount = 0;
    _lastFrameTimeMs = 0;
    _queueHighWatermark = getQueueDepth();
}

void SnifferManager::setNodeIdFilter(uint8_t nodeIdOr0)
{
    _nodeIdFilter = nodeIdOr0;
}

void SnifferManager::setTypeFilter(FilterType ft)
{
    _typeFilter = ft;
}

void SnifferManager::setIdRangeFilter(bool enabled, uint32_t idMin, uint32_t idMax)
{
    _idRangeEnabled = enabled;
    _idMin = idMin;
    _idMax = idMax;
}

void SnifferManager::taskThunk(void* arg)
{
    static_cast<SnifferManager*>(arg)->taskLoop();
}

void SnifferManager::taskLoop()
{
    SniffFrame f;

    for (;;) {
        if (!_enabled || !_q) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (xQueueReceive(_q, &f, pdMS_TO_TICKS(100)) == pdTRUE) {
            _lastFrameTimeMs = f.timestampMs;
            decodeAndStore(f);
        }
    }
}

static void printDecodedSerial(const DecodedFrame& df)
{
    // ID is <= 0x7FF typically, but keep 32-bit.
    Serial.printf("[%lu] ID: 0x%03lX  %-7s  N:%u  DLC:%u  %s%s  ",
                  (unsigned long)df.raw.timestampMs,
                  (unsigned long)df.raw.id,
                  df.type,
                  (unsigned)df.nodeId,
                  (unsigned)df.raw.dlc,
                  df.raw.ext ? "EXT " : "",
                  df.raw.rtr ? "RTR" : "");

    for (uint8_t i = 0; i < df.raw.dlc; i++) {
        Serial.printf("%02X ", df.raw.data[i]);
    }
    Serial.println();
}

const char* SnifferManager::classifyType(const SniffFrame& f, uint16_t baseId, uint8_t nodeId)
{
    (void)nodeId;

    if (f.rtr) return "RTR";

    if (f.id == 0x000) return "NMT";
    if (f.id == 0x080) return "SYNC";
    if (f.id == 0x100) return "TIME";

    if (baseId == 0x700) {
        if (f.dlc > 0 && f.data[0] == 0x00) return "BootUp";
        return "HB";
    }

    if (baseId == 0x580) return "SDO_Resp";
    if (baseId == 0x600) return "SDO_Req";
    if (baseId == 0x080) return "EMCY";

    // PDO ranges
    if (f.id >= 0x180 && f.id <= 0x1FF) return "TPDO1";
    if (f.id >= 0x200 && f.id <= 0x27F) return "RPDO1";
    if (f.id >= 0x280 && f.id <= 0x2FF) return "TPDO2";
    if (f.id >= 0x300 && f.id <= 0x37F) return "RPDO2";
    if (f.id >= 0x380 && f.id <= 0x3FF) return "TPDO3";
    if (f.id >= 0x400 && f.id <= 0x47F) return "RPDO3";
    if (f.id >= 0x480 && f.id <= 0x4FF) return "TPDO4";
    if (f.id >= 0x500 && f.id <= 0x57F) return "RPDO4";

    return "UNK";
}

bool SnifferManager::passFilters(const DecodedFrame& df) const
{
    if (_idRangeEnabled) {
        if (df.raw.id < _idMin || df.raw.id > _idMax) return false;
    }

    if (_nodeIdFilter != 0) {
        if (df.nodeId != _nodeIdFilter) return false;
    }

    if (_typeFilter != FilterType::All) {
        const char* t = df.type;
        switch (_typeFilter) {
            case FilterType::Heartbeat:
                if (!(strcmp(t, "HB") == 0 || strcmp(t, "BootUp") == 0)) return false;
                break;
            case FilterType::SDO:
                if (!(strncmp(t, "SDO_", 4) == 0)) return false;
                break;
            case FilterType::PDO:
                if (!(strstr(t, "PDO") != nullptr)) return false;
                break;
            case FilterType::EMCY:
                if (!(strcmp(t, "EMCY") == 0)) return false;
                break;
            case FilterType::NMT:
                if (!(strcmp(t, "NMT") == 0)) return false;
                break;
            case FilterType::SYNC:
                if (!(strcmp(t, "SYNC") == 0)) return false;
                break;
            case FilterType::TIME:
                if (!(strcmp(t, "TIME") == 0)) return false;
                break;
            case FilterType::Unknown:
                if (!(strcmp(t, "UNK") == 0)) return false;
                break;
            default: break;
        }
    }

    return true;
}

void SnifferManager::decodeAndStore(const SniffFrame& f)
{
    DecodedFrame df;
    df.raw = f;
    df.nodeId = (uint8_t)(f.id & 0x7F);
    df.baseId = (uint16_t)(f.id & 0x780);
    df.type = classifyType(f, df.baseId, df.nodeId);

    if (!passFilters(df)) return;

    portENTER_CRITICAL(&_recentMux);
    const uint16_t cap = (uint16_t)_recentRing.size();
    if (cap > 0) {
        _recentRing[_recentHead] = df;
        _recentHead = (uint16_t)((_recentHead + 1) % cap);
        if (_recentCount < cap) _recentCount++;
    }
    portEXIT_CRITICAL(&_recentMux);

    if (_serialOutput) {
        printDecodedSerial(df);
    }
}
