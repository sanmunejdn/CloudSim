#ifndef PROPERTYCORE_PROPERTYVEC3ATTRIBUTE_H
#define PROPERTYCORE_PROPERTYVEC3ATTRIBUTE_H

/// @file PropertyVec3Attribute.h
/// @brief Vec3 属性行（位姿分量等）

#include "PropertyAttributeHelpers.h"

#include <array>
#include <string>

namespace property_core
{
/// Vec3 属性行（位姿分量等）
template <typename TContext, typename TVec3, typename TBase>
class PropertyVec3Attribute : public TBase
{
public:
	using HasPropertyFn = bool (*)(const TContext&);
	using GetterFn = TVec3 (*)(const TContext&);
	using SetterFn = void (*)(TContext&, const TVec3&);
	using AppendRowFn = void (*)(nlohmann::json&, const char*, const char*, bool, const std::string&);

	PropertyVec3Attribute(HasPropertyFn hasPropertyFn, GetterFn getterFn, SetterFn setterFn,
						  std::array<const char*, 3> keys, std::array<const char*, 3> labels, AppendRowFn appendRowFn)
		: m_hasPropertyFn(hasPropertyFn), m_getterFn(getterFn), m_setterFn(setterFn), m_keys(keys), m_labels(labels),
		  m_appendRowFn(appendRowFn)
	{
	}

	void appendRows(const TContext& context, nlohmann::json& rows) const override
	{
		if (!m_hasPropertyFn || !m_getterFn || !m_appendRowFn || !m_hasPropertyFn(context))
		{
			return;
		}
		appendVec3Rows(rows, m_getterFn(context), m_keys, m_labels, m_appendRowFn);
	}

	bool handlesKey(const TContext& context, const std::string& key) const override
	{
		return m_hasPropertyFn && m_hasPropertyFn(context) && containsKey(m_keys, key);
	}

	bool apply(TContext& context, const std::string& key, const std::string& value, std::string* errMsg) const override
	{
		if (!m_hasPropertyFn || !m_getterFn || !m_setterFn || !m_hasPropertyFn(context) || !containsKey(m_keys, key))
		{
			return false;
		}
		double parsed = 0.0;
		if (!parseStrictDouble(value, parsed, errMsg))
		{
			return false;
		}
		TVec3 vec = m_getterFn(context);
		if (!applyVec3ByKey(vec, m_keys, key, parsed))
		{
			return false;
		}
		m_setterFn(context, vec);
		return true;
	}

private:
	HasPropertyFn m_hasPropertyFn = nullptr;
	GetterFn m_getterFn = nullptr;
	SetterFn m_setterFn = nullptr;
	std::array<const char*, 3> m_keys{};
	std::array<const char*, 3> m_labels{};
	AppendRowFn m_appendRowFn = nullptr;
};

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYVEC3ATTRIBUTE_H
