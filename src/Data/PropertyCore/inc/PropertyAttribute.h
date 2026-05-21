#pragma once

#include <json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace property_core
{

template <typename TContext>
class PropertyAttribute
{
public:
	virtual ~PropertyAttribute() = default;

	virtual void appendRows(const TContext& context, nlohmann::json& rows) const = 0;
	virtual bool handlesKey(const TContext& context, const std::string& key) const = 0;
	virtual bool apply(
		TContext& context,
		const std::string& key,
		const std::string& value,
		std::string* errMsg) const = 0;
};

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

	static bool apply(
		const AttributeList& attributes,
		TContext& context,
		const std::string& key,
		const std::string& value,
		std::string* errMsg)
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
