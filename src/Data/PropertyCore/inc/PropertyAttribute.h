#ifndef PROPERTYCORE_PROPERTYATTRIBUTE_H
#define PROPERTYCORE_PROPERTYATTRIBUTE_H

/// @file PropertyAttribute.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 属性面板行生成与 apply 插件基类

#include <memory>
#include <string>
#include <vector>

#include <json.hpp>

namespace property_core
{
/// 属性面板行生成与 apply 插件基类
template <typename TContext>
class PropertyAttribute
{
public:
	virtual ~PropertyAttribute() = default;

	virtual void appendRows(const TContext& context, nlohmann::json& rows) const = 0;
	virtual bool handlesKey(const TContext& context, const std::string& key) const = 0;
	virtual bool apply(TContext& context, const std::string& key, const std::string& value,
					   std::string* errMsg) const = 0;
};

/// 按序串联多个 PropertyAttribute
template <typename TContext, typename TAttributeBase>
class PropertyPipeline
{
public:
	using AttributePtr = std::shared_ptr<TAttributeBase>;
	using AttributeList = std::vector<AttributePtr>;

	static void appendRows(const AttributeList& attributes, const TContext& context, nlohmann::json& rows)
	{
		for (const AttributePtr& attr : attributes)
		{
			if (attr)
			{
				attr->appendRows(context, rows);
			}
		}
	}

	static bool apply(const AttributeList& attributes, TContext& context, const std::string& key,
					  const std::string& value, std::string* errMsg)
	{
		for (const AttributePtr& attr : attributes)
		{
			if (!attr || !attr->handlesKey(context, key))
			{
				continue;
			}
			return attr->apply(context, key, value, errMsg);
		}
		return false;
	}
};

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYATTRIBUTE_H
