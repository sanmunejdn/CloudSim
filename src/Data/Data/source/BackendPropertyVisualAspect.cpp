/// @file BackendPropertyVisualAspect.cpp
/// @brief 属性语义 → 视觉同步面映射

#include "BackendPropertyVisualAspect.h"

#include "../../PropertyCore/inc/PropertyTypes.h"

namespace backend_property_schema
{
namespace
{
using namespace property_core;

std::uint32_t aspectsFromSemanticFlags(const PropertySemanticFlags flags)
{
	std::uint32_t aspects = 0u;
	if ((flags & PropertySemanticFlags::AffectsBackendRootWorldXform) != PropertySemanticFlags::None)
	{
		aspects |= kVisualAspectTransform;
	}
	if ((flags & PropertySemanticFlags::AffectsColorOnly) != PropertySemanticFlags::None)
	{
		aspects |= kVisualAspectAppearance;
	}
	if ((flags & PropertySemanticFlags::AffectsVisibility) != PropertySemanticFlags::None)
	{
		aspects |= kVisualAspectVisibility;
	}
	if ((flags & PropertySemanticFlags::AffectsGeometry) != PropertySemanticFlags::None)
	{
		aspects |= kVisualAspectGeometry;
	}
	return aspects;
}

} // namespace

std::uint32_t visualAspectsForPropertyKey(const std::string& className, const std::string& key)
{
	if (key.empty() || (key.size() >= 3U && key.compare(0, 3, "ui.") == 0))
	{
		return 0u;
	}
	if (key.rfind("follow.", 0) == 0)
	{
		return 0u;
	}
	if (const PropertyDescriptor* d = findBackendPropertyDescriptor(className, key))
	{
		return aspectsFromSemanticFlags(d->semanticFlags);
	}
	// 未知 key：不再全量脏
	return 0u;
}

bool propertyCommitsPoseFromSchema(const std::string& className, const std::string& key)
{
	const std::uint32_t aspects = visualAspectsForPropertyKey(className, key);
	if ((aspects & kVisualAspectTransform) != 0u)
	{
		return true;
	}
	if (key.rfind("follow.", 0) == 0)
	{
		return true;
	}
	return key.rfind("pose.", 0) == 0 || key.rfind("rotation.", 0) == 0;
}

} // namespace backend_property_schema
