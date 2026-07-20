#ifndef DATA_BACKENDOBJECTATTRIBUTE_H
#define DATA_BACKENDOBJECTATTRIBUTE_H

/// @file BackendObjectAttribute.h
/// @brief 无状态属性插件：面板 JSON 行，apply 写回 BackendDataBase

#include "data_global.h"

#include "../../PropertyCore/inc/PropertyAttribute.h"

#include <memory>
#include <string>

#include <json.hpp>

class BackendDataBase;

/// 无状态属性插件：面板 JSON 行，apply 写回 BackendDataBase
class DATA_EXPORT BackendAttributeBase : public property_core::PropertyAttribute<BackendDataBase>
{
public:
	virtual ~BackendAttributeBase() = default;
};

using BackendAttributePtr = std::shared_ptr<BackendAttributeBase>;

DATA_EXPORT BackendAttributePtr makeBackendPoseAttribute();
DATA_EXPORT BackendAttributePtr makeBackendRotationAttribute();
DATA_EXPORT BackendAttributePtr makeBackendDisplayColorAttribute();

#endif // DATA_BACKENDOBJECTATTRIBUTE_H
