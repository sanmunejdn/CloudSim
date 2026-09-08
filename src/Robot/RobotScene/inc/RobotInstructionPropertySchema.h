#ifndef ROBOTSCENE_ROBOTINSTRUCTIONPROPERTYSCHEMA_H
#define ROBOTSCENE_ROBOTINSTRUCTIONPROPERTYSCHEMA_H

/// @file RobotInstructionPropertySchema.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 指令属性 schema 薄封装

#include "InstructionPropertyBinding.h"
#include "RobotInstructionModel.h"

namespace RobotInstruction
{
inline const property_core::PropertySchema& schemaForInstructionType(const Type type)
{
	return instruction_property_binding::schemaForType(type);
}

inline const property_core::PropertyDescriptor* findInstructionPropertyDescriptor(const Type type,
																				 const std::string& key)
{
	return schemaForInstructionType(type).find(key);
}

inline const property_core::PropertyDescriptor* findInstructionPropertyDescriptor(const std::string& key)
{
	static const Type kAllTypes[] = {Type::PTP,	 Type::LINE,	  Type::ARC,	 Type::WAIT,	  Type::IF,
									 Type::WHILE, Type::SET_DO,	  Type::SET_AO,	 Type::PathPlan, Type::DeviceAxis};
	for (const Type t : kAllTypes)
	{
		if (const property_core::PropertyDescriptor* d = findInstructionPropertyDescriptor(t, key))
		{
			return d;
		}
	}
	return nullptr;
}

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONPROPERTYSCHEMA_H
