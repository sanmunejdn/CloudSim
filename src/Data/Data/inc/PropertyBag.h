#pragma once

#include <array>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <json.hpp>

struct PropertyKey
{
	std::string name;
	std::type_index type{ typeid(void) };

	bool operator==(const PropertyKey& other) const noexcept
	{
		return name == other.name && type == other.type;
	}
};

struct PropertyKeyHash
{
	std::size_t operator()(const PropertyKey& key) const noexcept
	{
		const std::size_t h1 = std::hash<std::string>{}(key.name);
		const std::size_t h2 = std::hash<std::type_index>{}(key.type);
		return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6U) + (h1 >> 2U));
	}
};

using PropertyValue = std::variant<std::monostate, bool, int, double, std::string, std::array<double, 3>,
	std::array<float, 4>>;

struct PropertyBagDiffItem
{
	std::string name;
	PropertyValue value;
};

struct PropertyBagDiff
{
	std::vector<PropertyBagDiffItem> updates;
};

/// 运行时属性袋：name+type 键，variant 值
class PropertyBag
{
public:
	template <typename T>
	void set(const std::string& name, const T& value)
	{
		const PropertyKey key{ name, std::type_index(typeid(T)) };
		m_values[key] = value;
	}

	template <typename T>
	bool tryGet(const std::string& name, T& out) const
	{
		const PropertyKey key{ name, std::type_index(typeid(T)) };
		const auto it = m_values.find(key);
		if (it == m_values.end())
		{
			return false;
		}
		if (const T* const v = std::get_if<T>(&it->second))
		{
			out = *v;
			return true;
		}
		return false;
	}

	void applyDiff(const PropertyBagDiff& diff)
	{
		for (const PropertyBagDiffItem& item : diff.updates)
		{
			const PropertyKey key{ item.name, item.value.index() == 0 ? std::type_index(typeid(void)) : valueTypeIndex(item.value) };
			m_values[key] = item.value;
		}
	}

	nlohmann::json toJson() const
	{
		nlohmann::json out = nlohmann::json::object();
		for (const auto& kv : m_values)
		{
			const std::string& name = kv.first.name;
			const PropertyValue& v = kv.second;
			if (const bool* b = std::get_if<bool>(&v))
			{
				out[name] = *b;
			}
			else if (const int* i = std::get_if<int>(&v))
			{
				out[name] = *i;
			}
			else if (const double* d = std::get_if<double>(&v))
			{
				out[name] = *d;
			}
			else if (const std::string* s = std::get_if<std::string>(&v))
			{
				out[name] = *s;
			}
			else if (const std::array<double, 3>* vec3 = std::get_if<std::array<double, 3>>(&v))
			{
				out[name] = { (*vec3)[0], (*vec3)[1], (*vec3)[2] };
			}
			else if (const std::array<float, 4>* rgba = std::get_if<std::array<float, 4>>(&v))
			{
				out[name] = { (*rgba)[0], (*rgba)[1], (*rgba)[2], (*rgba)[3] };
			}
		}
		return out;
	}

private:
	static std::type_index valueTypeIndex(const PropertyValue& value)
	{
		if (std::holds_alternative<bool>(value))
		{
			return std::type_index(typeid(bool));
		}
		if (std::holds_alternative<int>(value))
		{
			return std::type_index(typeid(int));
		}
		if (std::holds_alternative<double>(value))
		{
			return std::type_index(typeid(double));
		}
		if (std::holds_alternative<std::string>(value))
		{
			return std::type_index(typeid(std::string));
		}
		if (std::holds_alternative<std::array<double, 3>>(value))
		{
			return std::type_index(typeid(std::array<double, 3>));
		}
		if (std::holds_alternative<std::array<float, 4>>(value))
		{
			return std::type_index(typeid(std::array<float, 4>));
		}
		return std::type_index(typeid(void));
	}

	std::unordered_map<PropertyKey, PropertyValue, PropertyKeyHash> m_values;
};
