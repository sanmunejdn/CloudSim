#ifndef DATA_IBACKENDDATAQUERY_H
#define DATA_IBACKENDDATAQUERY_H

/// @file IBackendDataQuery.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后端数据只读查询接口：Headless/几何库函数经此解耦 BackendDataManager 具体类型

#include "data_global.h"

#include <memory>
#include <string>
#include <vector>

class BackendDataBase;

/// 只读查询窄接口；写路径与层级变更仍走 BackendDataManager 具体类型
class DATA_EXPORT IBackendDataQuery
{
public:
	virtual ~IBackendDataQuery() = default;

	virtual std::shared_ptr<BackendDataBase> getData(const std::string& id) const = 0;
	virtual std::vector<std::shared_ptr<BackendDataBase>> findByClass(const std::string& className) const = 0;
};

#endif // DATA_IBACKENDDATAQUERY_H
