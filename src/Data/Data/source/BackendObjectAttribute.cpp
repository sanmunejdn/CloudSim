/// @file BackendObjectAttribute.cpp
/// @brief 后端对象属性

#include "BackendObjectAttribute.h"

#include "../../PropertyCore/inc/PropertyAttributeHelpers.h"
#include "../../PropertyCore/inc/PropertyRgbaAttribute.h"
#include "../../PropertyCore/inc/PropertyVec3Attribute.h"
#include "BackendDataBase.h"
#include "BackendPropertyRow.h"

#include <array>

namespace
{
void appendBackendRow(nlohmann::json& rows, const char* key, const char* label, bool editable, const std::string& value)
{
	backend_property_json::appendRow(rows, key, label, editable, value);
}

bool hasPoseProperty(const BackendDataBase& data)
{
	return data.hasPoseProperty();
}

BackendVec3 getPose(const BackendDataBase& data)
{
	return data.pose();
}

void setPose(BackendDataBase& data, const BackendVec3& pose)
{
	data.setPose(pose);
}

bool hasRotationProperty(const BackendDataBase& data)
{
	return data.hasRotationProperty();
}

BackendVec3 getRotation(const BackendDataBase& data)
{
	return data.rotation();
}

void setRotation(BackendDataBase& data, const BackendVec3& rotation)
{
	data.setRotation(rotation);
}

bool hasColorProperty(const BackendDataBase& data)
{
	return data.hasColorProperty();
}

BackendColor getColor(const BackendDataBase& data)
{
	return data.color();
}

void setColor(BackendDataBase& data, const BackendColor& color)
{
	data.setColor(color);
}

} // namespace

namespace
{
using Vec3AttributeImpl = property_core::PropertyVec3Attribute<BackendDataBase, BackendVec3, BackendAttributeBase>;
using RgbaAttributeImpl = property_core::PropertyRgbaAttribute<BackendDataBase, BackendColor, BackendAttributeBase>;
} // namespace

BackendAttributePtr makeBackendPoseAttribute()
{
	return std::make_shared<Vec3AttributeImpl>(
		hasPoseProperty, getPose, setPose, std::array<const char*, 3>{"pose.x", "pose.y", "pose.z"},
		std::array<const char*, 3>{"Pose X", "Pose Y", "Pose Z"}, appendBackendRow);
}

BackendAttributePtr makeBackendRotationAttribute()
{
	return std::make_shared<Vec3AttributeImpl>(
		hasRotationProperty, getRotation, setRotation,
		std::array<const char*, 3>{"rotation.x", "rotation.y", "rotation.z"},
		std::array<const char*, 3>{"Rotation X (deg)", "Rotation Y (deg)", "Rotation Z (deg)"}, appendBackendRow);
}

BackendAttributePtr makeBackendDisplayColorAttribute()
{
	return std::make_shared<RgbaAttributeImpl>(
		hasColorProperty, getColor, setColor, std::array<const char*, 4>{"color.r", "color.g", "color.b", "color.a"},
		std::array<const char*, 4>{"Color R", "Color G", "Color B", "Color A"}, appendBackendRow);
}
