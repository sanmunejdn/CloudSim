#ifndef DATA_BACKENDPROPERTYVISUALASPECT_H
#define DATA_BACKENDPROPERTYVISUALASPECT_H

/// @file BackendPropertyVisualAspect.h
/// @brief 属性 key → 视觉同步面（无 OSG 依赖）

#include "BackendPropertySchema.h"
#include "data_global.h"

#include <cstdint>
#include <string>

namespace backend_property_schema
{
/// 与 cloudsim::host::VisualAspect 位值对齐
constexpr std::uint32_t kVisualAspectTransform = 1u << 0;
constexpr std::uint32_t kVisualAspectAppearance = 1u << 1;
constexpr std::uint32_t kVisualAspectVisibility = 1u << 2;
constexpr std::uint32_t kVisualAspectGeometry = 1u << 3;
constexpr std::uint32_t kVisualAspectSelection = 1u << 4;
constexpr std::uint32_t kVisualAspectHierarchy = 1u << 5;

DATA_EXPORT std::uint32_t visualAspectsForPropertyKey(const std::string& className, const std::string& key);
DATA_EXPORT bool propertyCommitsPoseFromSchema(const std::string& className, const std::string& key);

} // namespace backend_property_schema

#endif // DATA_BACKENDPROPERTYVISUALASPECT_H
