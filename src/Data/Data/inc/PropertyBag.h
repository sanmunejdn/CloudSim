#ifndef DATA_PROPERTYBAG_H
#define DATA_PROPERTYBAG_H

/// @file PropertyBag.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 运行时属性袋：name+type 键，variant 值

#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <json.hpp>

#ifdef DATA_BUILD_DLL
// Data.dll 内部：直接链 RunLogger
#include "RunLogger.h"
#else
// 宿主工程（BackendVisual 等）无 RunLogger include 目录：
// 此处仅保证可编译；同名异型冲突告警在宿主侧静默（Data.dll 内部仍全量告警）
namespace PropertyBagDetail
{
inline void defaultWarn(const std::string&) {}
} // namespace PropertyBagDetail
#endif

struct PropertyKey
{
	std::string name;
	std::type_index type{typeid(void)};

	bool operator==(const PropertyKey& other) const noexcept { return name == other.name && type == other.type; }
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

using PropertyValue =
	std::variant<std::monostate, bool, int, double, std::string, std::array<double, 3>, std::array<float, 4>>;

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
	using WarningHook = std::function<void(const std::string&)>;

	/// 默认接 RunLogger::warn（不设 hook 时同名异型冲突也会告警）；测试可替换
	static void setWarningHook(WarningHook hook) { s_warningHook = std::move(hook); }

	template <typename T>
	void set(const std::string& name, const T& value)
	{
		const PropertyKey key{name, std::type_index(typeid(T))};
		// 同名异型冲突检测退化为 name 桶内常数时间，不再 O(n) 全表扫描
		const auto range = m_nameBuckets.equal_range(name);
		for (auto it = range.first; it != range.second; ++it)
		{
			if (it->second != key.type)
			{
				if (s_warningHook)
				{
					s_warningHook(std::string("[PropertyBag] type mismatch overwrite for key: ") + name);
				}
				break;
			}
		}
		const bool existed = m_values.find(key) != m_values.end();
		m_values[key] = value;
		if (!existed)
		{
			m_nameBuckets.emplace(name, key.type);
		}
	}

	template <typename T>
	bool tryGet(const std::string& name, T& out) const
	{
		const PropertyKey key{name, std::type_index(typeid(T))};
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
			const PropertyKey key{item.name,
								  item.value.index() == 0 ? std::type_index(typeid(void)) : valueTypeIndex(item.value)};
			const bool existed = m_values.find(key) != m_values.end();
			m_values[key] = item.value;
			if (!existed)
			{
				m_nameBuckets.emplace(item.name, key.type);
			}
		}
	}

	nlohmann::json toJson() const
	{
		// 以 name 为主键（同名多型时按类型名取首个）；先按 name 排一次序，
		// 避免 names × m_values 的 O(n²) 双循环
		std::vector<std::pair<std::string, std::type_index>> entries;
		entries.reserve(m_values.size());
		for (const auto& kv : m_values)
		{
			entries.emplace_back(kv.first.name, kv.first.type);
		}
		std::sort(entries.begin(), entries.end(),
				  [](const std::pair<std::string, std::type_index>& a,
					 const std::pair<std::string, std::type_index>& b) -> bool
				  {
					  if (a.first != b.first)
					  {
						  return a.first < b.first;
					  }
					  // 同名异型按类型名字典序，跨运行输出确定（type_index 地址次序不稳定）
					  return std::string(a.second.name()) < std::string(b.second.name());
				  });
		nlohmann::json out = nlohmann::json::object();
		std::string lastName;
		for (const auto& e : entries)
		{
			if (e.first == lastName)
			{
				continue;
			}
			lastName = e.first;
			const auto it = m_values.find(PropertyKey{e.first, e.second});
			if (it == m_values.end())
			{
				continue;
			}
			const PropertyValue& v = it->second;
			if (const bool* b = std::get_if<bool>(&v))
			{
				out[e.first] = *b;
			}
			else if (const int* i = std::get_if<int>(&v))
			{
				out[e.first] = *i;
			}
			else if (const double* d = std::get_if<double>(&v))
			{
				out[e.first] = *d;
			}
			else if (const std::string* s = std::get_if<std::string>(&v))
			{
				out[e.first] = *s;
			}
			else if (const std::array<double, 3>* vec3 = std::get_if<std::array<double, 3>>(&v))
			{
				out[e.first] = {(*vec3)[0], (*vec3)[1], (*vec3)[2]};
			}
			else if (const std::array<float, 4>* rgba = std::get_if<std::array<float, 4>>(&v))
			{
				out[e.first] = {(*rgba)[0], (*rgba)[1], (*rgba)[2], (*rgba)[3]};
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
	// name → 已用 type_index 桶，供 set() 常数时间做同名异型冲突检测
	std::unordered_multimap<std::string, std::type_index> m_nameBuckets;
#ifdef DATA_BUILD_DLL
	static inline WarningHook s_warningHook = [](const std::string& message) { RunLogger::warn(message); };
#else
	static inline WarningHook s_warningHook = [](const std::string& message) { PropertyBagDetail::defaultWarn(message); };
#endif
};

#endif // DATA_PROPERTYBAG_H
