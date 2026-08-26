#ifndef DATA_BACKENDCOMPONENT_H
#define DATA_BACKENDCOMPONENT_H

/// @file BackendComponent.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后端组件类型擦除契约（Follow 等挂到 BackendDataBase）

#include "data_global.h"

#include <memory>
#include <string>
#include <vector>

#include <json.hpp>

class BackendDataBase;
class BackendDataManager;

/// 后端组件类型擦除契约（Follow 等挂到 BackendDataBase）
class DATA_EXPORT IBackendComponent
{
public:
	virtual ~IBackendComponent() = default;
	virtual std::string componentType() const = 0;

	virtual void appendPropertyRows(nlohmann::json& rows, const BackendDataManager* mgr = nullptr) const
	{
		(void)rows;
		(void)mgr;
	}

	virtual bool applyPropertyChange(BackendDataBase& owner, const std::string& key, const std::string& value,
									 std::string* errMsg, const BackendDataManager* mgr = nullptr)
	{
		(void)owner;
		(void)key;
		(void)value;
		(void)errMsg;
		(void)mgr;
		return false;
	}

	/// 对象无此组件时是否仍显示默认属性行（如空 Follow 行）
	virtual bool appendDefaultPropertyRowsWhenAbsent(nlohmann::json& rows,
													 const BackendDataManager* mgr = nullptr) const
	{
		(void)rows;
		(void)mgr;
		return false;
	}

	/// 该组件引用的其他后端对象 id（unregister 悬挂引用检测用）
	virtual void collectReferencedBackendIds(std::vector<std::string>& out) const
	{
		(void)out;
	}
};

using BackendComponentPtr = std::shared_ptr<IBackendComponent>;

#endif // DATA_BACKENDCOMPONENT_H
