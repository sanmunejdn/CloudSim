/// @file BackendPropertyVisualAspect.cpp
/// @brief 属性语义 → 视觉同步面映射

#include "BackendPropertyVisualAspect.h"

#include "../../PropertyCore/inc/PropertyTypes.h"

#include <algorithm>

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
	if ((flags & PropertySemanticFlags::LegacyFullCommitBehavior) != PropertySemanticFlags::None)
	{
		aspects |= kVisualAspectTransform | kVisualAspectAppearance | kVisualAspectSelection;
	}
	return aspects;
}

const PropertyDescriptor* findDescriptorForClass(const std::string& className, const std::string& key)
{
	if (const PropertyDescriptor* d = schemaForBackendClassName(className).find(key))
	{
		return d;
	}
	return findAnyBackendPropertyDescriptor(key);
}

} // namespace

std::uint32_t visualAspectsForPropertyKey(const std::string& className, const std::string& key)
{
	if (key.empty() || (key.size() >= 3U && key.compare(0, 3, "ui.") == 0))
	{
		return 0u;
	}
	if (key == "visible")
	{
		return kVisualAspectVisibility;
	}
	if (const PropertyDescriptor* d = findDescriptorForClass(className, key))
	{
		const std::uint32_t aspects = aspectsFromSemanticFlags(d->semanticFlags);
		if (aspects != 0u)
		{
			return aspects;
		}
	}
	if (key.rfind("follow.", 0) == 0)
	{
		return 0u;
	}
	if (key.rfind("pose.", 0) == 0 || key.rfind("rotation.", 0) == 0)
	{
		return kVisualAspectTransform;
	}
	// 前缀精确匹配，避免 "discolorXxx"/"invisibleXxx" 这类键被子串误判
	if (key.rfind("color.", 0) == 0)
	{
		return kVisualAspectAppearance;
	}
	if (key.rfind("visible.", 0) == 0)
	{
		return kVisualAspectVisibility;
	}
	return kVisualAspectTransform | kVisualAspectAppearance | kVisualAspectSelection;
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
	// 前缀精确匹配，避免 "transposedXxx" 这类键被 "pose" 子串误判为位姿提交
	return key.rfind("pose.", 0) == 0 || key.rfind("rotation.", 0) == 0;
}

} // namespace backend_property_schema
