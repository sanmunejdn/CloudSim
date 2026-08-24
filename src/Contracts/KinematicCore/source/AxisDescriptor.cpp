#include "AxisDescriptor.h"

namespace kinematic_core
{
AxisDescriptor::AxisDescriptor() = default;
AxisDescriptor::~AxisDescriptor() = default;
AxisDescriptor::AxisDescriptor(const AxisDescriptor&) = default;
AxisDescriptor::AxisDescriptor(AxisDescriptor&&) noexcept = default;
AxisDescriptor& AxisDescriptor::operator=(const AxisDescriptor&) = default;
AxisDescriptor& AxisDescriptor::operator=(AxisDescriptor&&) noexcept = default;
} // namespace kinematic_core
