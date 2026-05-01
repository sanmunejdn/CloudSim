#pragma once

#include "PropertyAttributeHelpers.h"

#include <string>

namespace property_core
{

template <typename TContext, typename TValue, typename TBase>
class PropertyScalarAttribute : public TBase
{
public:
	using HasPropertyFn = bool (*)(const TContext&);
	using GetterFn = TValue (*)(const TContext&);
	using SetterFn = void (*)(TContext&, const TValue&);
	using AppendRowFn = void (*)(nlohmann::json&, const char*, const char*, bool, const std::string&);

	PropertyScalarAttribute(
		HasPropertyFn hasPropertyFn,
		GetterFn getterFn,
		SetterFn setterFn,
		const char* key,
		const char* label,
		AppendRowFn appendRowFn)
		: m_hasPropertyFn(hasPropertyFn)
		, m_getterFn(getterFn)
		, m_setterFn(setterFn)
		, m_key(key)
		, m_label(label)
		, m_appendRowFn(appendRowFn)
	{
	}

	void appendRows(const TContext& context, nlohmann::json& rows) const override
	{
		if (!m_hasPropertyFn || !m_getterFn || !m_appendRowFn || !m_hasPropertyFn(context))
		{
			return;
		}
		m_appendRowFn(rows, m_key, m_label, true, formatScalarValue<TValue>(m_getterFn(context)));
	}

	bool handlesKey(const TContext& context, const std::string& key) const override
	{
		return m_hasPropertyFn && m_hasPropertyFn(context) && key == m_key;
	}

	bool apply(TContext& context, const std::string& key, const std::string& value, std::string* errMsg) const override
	{
		if (!m_hasPropertyFn || !m_setterFn || !m_hasPropertyFn(context) || key != m_key)
		{
			return false;
		}
		TValue parsed{};
		if (!parseScalarValue<TValue>(value, parsed, errMsg))
		{
			return false;
		}
		m_setterFn(context, parsed);
		return true;
	}

private:
	HasPropertyFn m_hasPropertyFn = nullptr;
	GetterFn m_getterFn = nullptr;
	SetterFn m_setterFn = nullptr;
	const char* m_key = "";
	const char* m_label = "";
	AppendRowFn m_appendRowFn = nullptr;
};

} // namespace property_core
