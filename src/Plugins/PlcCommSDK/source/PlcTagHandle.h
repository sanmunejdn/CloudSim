#pragma once

#include <cstdint>
#include <string>

/// libplctag 句柄 RAII
class PlcTagHandle
{
public:
    PlcTagHandle() = default;
    PlcTagHandle(const std::string& attributeString, int timeoutMs);
    ~PlcTagHandle();

    PlcTagHandle(const PlcTagHandle&) = delete;
    PlcTagHandle& operator=(const PlcTagHandle&) = delete;
    PlcTagHandle(PlcTagHandle&& other) noexcept;
    PlcTagHandle& operator=(PlcTagHandle&& other) noexcept;

    bool valid() const { return id_ > 0; }
    int32_t id() const { return id_; }
    int lastStatus() const { return lastStatus_; }

private:
    void reset();

    int32_t id_ = 0;
    int lastStatus_ = 0;
};
