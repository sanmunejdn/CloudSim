#pragma once

#include "robot_scene_global.h"
#include "../../Data/PropertyCore/inc/PropertyAttribute.h"
#include "../../Data/PropertyCore/inc/PropertyEnumAttribute.h"
#include "../../Data/PropertyCore/inc/PropertyScalarAttribute.h"
#include "../../Data/PropertyCore/inc/PropertyVec3Attribute.h"

#include <json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace RobotInstruction
{
class Base;
struct Vec3;

class ROBOT_SCENE_API AttributeBase : public property_core::PropertyAttribute<Base>
{
public:
	virtual ~AttributeBase() = default;
};

class ROBOT_SCENE_API PoseAttribute final
	: public property_core::PropertyVec3Attribute<Base, Vec3, AttributeBase>
{
public:
	PoseAttribute();
};

class ROBOT_SCENE_API EulerAttribute final
	: public property_core::PropertyVec3Attribute<Base, Vec3, AttributeBase>
{
public:
	EulerAttribute();
};

using AttributePtr = std::shared_ptr<AttributeBase>;

ROBOT_SCENE_API AttributePtr makeScalarDoubleAttribute(
	bool (*hasProperty)(const Base&),
	double (*getter)(const Base&),
	void (*setter)(Base&, const double&),
	const char* key,
	const char* label);

ROBOT_SCENE_API AttributePtr makeEnumAttribute(
	bool (*hasProperty)(const Base&),
	std::string (*getter)(const Base&),
	void (*setter)(Base&, const std::string&),
	const char* key,
	const char* label,
	bool (*isValidFn)(const std::string&) = nullptr);

ROBOT_SCENE_API AttributePtr makeSpeedAttribute();
ROBOT_SCENE_API AttributePtr makeAccelAttribute();
ROBOT_SCENE_API AttributePtr makeAxisConfigAttribute();
ROBOT_SCENE_API std::vector<AttributePtr> makeMotionAxisConfigAttributes();
ROBOT_SCENE_API AttributePtr makeBlendRadiusAttribute();

} // namespace RobotInstruction
