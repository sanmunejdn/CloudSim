#ifndef PROPERTYCORE_PROPERTYENUMATTRIBUTE_H
#define PROPERTYCORE_PROPERTYENUMATTRIBUTE_H

/// @file PropertyEnumAttribute.h
/// @brief 枚举属性行（下拉/自定义）

#include <string>

#include <json.hpp>

namespace property_core
{
/// 枚举属性行（下拉/自定义）
template <typename TContext, typename TBase>
class PropertyEnumAttribute : public TBase
{
public:
	using HasPropertyFn = bool (*)(const TContext&);
	using GetterFn = std::string (*)(const TContext&);
	using SetterFn = void (*)(TContext&, const std::string&);
	using IsValidFn = bool (*)(const std::string&);
	using AppendRowFn = void (*)(nlohmann::json&, const char*, const char*, bool, const std::string&);

	PropertyEnumAttribute(HasPropertyFn hasPropertyFn, GetterFn getterFn, SetterFn setterFn, const char* key,
						  const char* label, AppendRowFn appendRowFn, IsValidFn isValidFn = nullptr)
		: m_hasPropertyFn(hasPropertyFn), m_getterFn(getterFn), m_setterFn(setterFn), m_key(key), m_label(label),
		  m_appendRowFn(appendRowFn), m_isValidFn(isValidFn)
	{
	}

	void appendRows(const TContext& context, nlohmann::json& rows) const override
	{
		if (!m_hasPropertyFn || !m_getterFn || !m_appendRowFn || !m_hasPropertyFn(context))
		{
			return;
		}
		m_appendRowFn(rows, m_key, m_label, true, m_getterFn(context));
	}

	bool handlesKey(const TContext& context, const std::string& key) const override
	{
		return m_hasPropertyFn && m_hasPropertyFn(context) && key == m_key;
	}

	bool apply(TContext& context, const std::string& key, const std::string& value, std::string* errMsg) const override
	{
		(void)errMsg;
		if (!m_hasPropertyFn || !m_setterFn || !m_hasPropertyFn(context) || key != m_key)
		{
			return false;
		}
		if (m_isValidFn && !m_isValidFn(value))
		{
			return false;
		}
		m_setterFn(context, value);
		return true;
	}

private:
	HasPropertyFn m_hasPropertyFn = nullptr;
	GetterFn m_getterFn = nullptr;
	SetterFn m_setterFn = nullptr;
	const char* m_key = "";
	const char* m_label = "";
	AppendRowFn m_appendRowFn = nullptr;
	IsValidFn m_isValidFn = nullptr;
};

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYENUMATTRIBUTE_H
