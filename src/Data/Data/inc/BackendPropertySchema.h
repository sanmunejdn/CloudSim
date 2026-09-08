#ifndef DATA_BACKENDPROPERTYSCHEMA_H
#define DATA_BACKENDPROPERTYSCHEMA_H

/// @file BackendPropertySchema.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后端属性 schema：由 Binding 表生成（薄分发）

#include "BackendPropertyBinding.h"
#include "BackendTypeIdentity.h"

#include "../../PropertyCore/inc/PropertySchema.h"
#include "../../PropertyCore/inc/PropertyTypes.h"

/// 后端对象属性 schema（真源为 Binding）
namespace backend_property_schema
{
inline const property_core::PropertySchema& schemaForBackendClassName(const std::string& className)
{
	return backend_property_binding::schemaForClassName(className);
}

inline const property_core::PropertySchema& followAttachmentBackendPropertySchema()
{
	using namespace property_core;
	static const PropertySchema schema = []()
	{
		PropertySchema s;
		s.objectTypeId = "backend.component.follow_attachment";
		s.schemaVersion = 1;
		PropertyDescriptor d;
		d.key = "follow.targetName";
		d.label = "Follow: target object name";
		d.type = PropertyType::String;
		d.editable = true;
		d.semanticFlags = PropertySemanticFlags::AffectsFollowConstraintGraph;
		s.descriptors.push_back(std::move(d));
		return s;
	}();
	return schema;
}

/// 按 className 查 key；未知 class 不猜其它类型的 color.*
inline const property_core::PropertyDescriptor* findBackendPropertyDescriptor(const std::string& className,
																			 const std::string& key)
{
	if (const property_core::PropertyDescriptor* d = schemaForBackendClassName(className).find(key))
	{
		return d;
	}
	return followAttachmentBackendPropertySchema().find(key);
}

/// 无 className 时全局搜；面板应优先 findBackendPropertyDescriptor(className, key)
inline const property_core::PropertyDescriptor* findAnyBackendPropertyDescriptor(const std::string& key)
{
	for (const char* cn :
		 {backend_type::kClassPointCloud, backend_type::kClassModel, backend_type::kClassBrepModel,
		  backend_type::kClassParametricBrep, backend_type::kClassFrame, backend_type::kClassCustomDevice})
	{
		if (const property_core::PropertyDescriptor* d = schemaForBackendClassName(cn).find(key))
		{
			return d;
		}
	}
	return followAttachmentBackendPropertySchema().find(key);
}

} // namespace backend_property_schema

#endif // DATA_BACKENDPROPERTYSCHEMA_H
