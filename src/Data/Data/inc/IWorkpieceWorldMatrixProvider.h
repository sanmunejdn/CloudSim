#ifndef DATA_IWORKPIECEWORLDMATRIXPROVIDER_H
#define DATA_IWORKPIECEWORLDMATRIXPROVIDER_H

/// @file IWorkpieceWorldMatrixProvider.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 工件世界矩阵唯一查询口（桌面 OSG / Headless Backend）

#include "data_global.h"

#include "BackendFollowMath.h"
#include "IBackendDataQuery.h"

#include <string>

/// 跨工件离散、Raw→世界等凡需工件位姿处只走此接口，禁止再直读 Backend 与 OSG 双源分叉
class DATA_EXPORT IWorkpieceWorldMatrixProvider
{
public:
	virtual ~IWorkpieceWorldMatrixProvider() = default;
	virtual bool getWorldMatrix(const std::string& backendId, BackendMat4& out) const = 0;
};

/// Headless/Web：持久化 Backend 世界矩阵
class DATA_EXPORT BackendDataWorldMatrixProvider final : public IWorkpieceWorldMatrixProvider
{
public:
	explicit BackendDataWorldMatrixProvider(const IBackendDataQuery& query) : m_query(query) {}

	bool getWorldMatrix(const std::string& backendId, BackendMat4& out) const override;

private:
	const IBackendDataQuery& m_query;
};

#endif // DATA_IWORKPIECEWORLDMATRIXPROVIDER_H
