#include "PlcTagHandle.h"

#include <libplctag.h>

PlcTagHandle::PlcTagHandle(const std::string& attributeString, int timeoutMs)
{
    id_ = plc_tag_create(attributeString.c_str(), timeoutMs);
    if (id_ < 0) {
        lastStatus_ = static_cast<int>(id_);
        id_ = 0;
    } else {
        lastStatus_ = plc_tag_status(id_);
    }
}

PlcTagHandle::~PlcTagHandle()
{
    reset();
}

PlcTagHandle::PlcTagHandle(PlcTagHandle&& other) noexcept
    : id_(other.id_)
    , lastStatus_(other.lastStatus_)
{
    other.id_ = 0;
    other.lastStatus_ = 0;
}

PlcTagHandle& PlcTagHandle::operator=(PlcTagHandle&& other) noexcept
{
    if (this != &other) {
        reset();
        id_ = other.id_;
        lastStatus_ = other.lastStatus_;
        other.id_ = 0;
        other.lastStatus_ = 0;
    }
    return *this;
}

void PlcTagHandle::reset()
{
    if (id_ > 0) {
        plc_tag_destroy(id_);
        id_ = 0;
    }
}
