#ifndef PROPERTYCORE_PROPERTYRGBAATTRIBUTE_H
#define PROPERTYCORE_PROPERTYRGBAATTRIBUTE_H

/// @file PropertyRgbaAttribute.h
/// @brief RGBA 颜色属性行

#include "PropertyAttributeHelpers.h"

#include <array>
#include <string>

namespace property_core
{
/// RGBA 颜色属性行
template <typename TContext, typename TColor, typename TBase>
class PropertyRgbaAttribute : public TBase
{
public:
	using HasPropertyFn = bool (*)(const TContext&);
	using GetterFn = TColor (*)(const TContext&);
	using SetterFn = void (*)(TContext&, const TColor&);
	using AppendRowFn = void (*)(nlohmann::json&, const char*, const char*, bool, const std::string&);

	PropertyRgbaAttribute(HasPropertyFn hasPropertyFn, GetterFn getterFn, SetterFn setterFn,
						  std::array<const char*, 4> keys, std::array<const char*, 4> labels, AppendRowFn appendRowFn)
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
		appendRgbaRows(rows, m_getterFn(context), m_keys, m_labels, m_appendRowFn);
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
		TColor color = m_getterFn(context);
		if (!applyRgbaByKey(color, m_keys, key, parsed))
		{
			return false;
		}
		m_setterFn(context, color);
		return true;
	}

private:
	HasPropertyFn m_hasPropertyFn = nullptr;
	GetterFn m_getterFn = nullptr;
	SetterFn m_setterFn = nullptr;
	std::array<const char*, 4> m_keys{};
	std::array<const char*, 4> m_labels{};
	AppendRowFn m_appendRowFn = nullptr;
};

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYRGBAATTRIBUTE_H
