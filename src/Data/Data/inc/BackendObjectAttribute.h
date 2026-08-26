#ifndef DATA_BACKENDOBJECTATTRIBUTE_H
#define DATA_BACKENDOBJECTATTRIBUTE_H

/// @file BackendObjectAttribute.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 无状态属性插件：面板 JSON 行，apply 写回 BackendDataBase

#include "data_global.h"

#include "../../PropertyCore/inc/PropertyAttribute.h"

#include <memory>
#include <string>
#include <vector>

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

/// 按对象声明的能力追加标准属性 attribute。
/// 派生类构造函数必须改调此函数而非手工 push 单个 attribute：
/// 手工 push 与 has*Property() 无编译期关联，漏推时面板静默少行
DATA_EXPORT void appendStandardAttributesForCapabilities(const BackendDataBase& self,
														 std::vector<BackendAttributePtr>& outAttributes);

#endif // DATA_BACKENDOBJECTATTRIBUTE_H
