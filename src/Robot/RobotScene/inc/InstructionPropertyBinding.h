#ifndef ROBOTSCENE_INSTRUCTIONPROPERTYBINDING_H
#define ROBOTSCENE_INSTRUCTIONPROPERTYBINDING_H

/// @file InstructionPropertyBinding.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 指令面板/schema 唯一清单：descriptor + 字符串 format/apply

#include "robot_scene_global.h"

#include "../../Data/PropertyCore/inc/PropertyDescriptor.h"
#include "../../Data/PropertyCore/inc/PropertySchema.h"

#include <string>
#include <vector>

#include <json.hpp>

namespace RobotInstruction
{
class Base;
enum class Type;

struct ROBOT_SCENE_API InstructionPropertyBinding
{
	property_core::PropertyDescriptor desc;
	std::string (*formatValue)(const Base&) = nullptr;
	bool (*applyValue)(Base&, const std::string& value, std::string* err) = nullptr;
	/// false 时仅进 schema，不进当前 snapshot（条件门控）
	bool (*isActive)(const Base&) = nullptr;
};

namespace instruction_property_binding
{
ROBOT_SCENE_API std::vector<InstructionPropertyBinding> collectBindings(const Base& cmd);

ROBOT_SCENE_API const property_core::PropertySchema& schemaForType(Type type);

ROBOT_SCENE_API void appendBindingRows(const Base& cmd, nlohmann::json& rows);

ROBOT_SCENE_API bool applyBindingKey(Base& cmd, const std::string& key, const std::string& value,
									 std::string* errMsg);
} // namespace instruction_property_binding

} // namespace RobotInstruction

#endif // ROBOTSCENE_INSTRUCTIONPROPERTYBINDING_H
