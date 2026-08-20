#ifndef TRAJECTORYALGORITHM_IEXTERNALAXISSEARCHSERVICE_H
#define TRAJECTORYALGORITHM_IEXTERNALAXISSEARCHSERVICE_H

/// @file IExternalAxisSearchService.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 外部轴搜索服务：由 RobotScene/UI 注入；无配置时 Op 直接跳过

#include "trajectory_algorithm_global.h"

#include <string>
#include <vector>

namespace RobotInstruction
{
struct UnifiedTrajectory;
}

namespace trajectory_algo
{
struct TRAJECTORY_ALGORITHM_API ExternalAxisSearchConfigDto
{
	bool enabled = false;
	std::string jointName;
	bool isPrismatic = true;
	double lower = 0.0;
	double upper = 1000.0;
	double home = 0.0;
	double axis[3]{1.0, 0.0, 0.0};
};

class TRAJECTORY_ALGORITHM_API IExternalAxisSearchService
{
public:
	virtual ~IExternalAxisSearchService() = default;

	/// 按配置对轨迹点搜索外轴；allowCoupledRefine 时用联立 DLS 微调
	virtual bool search(RobotInstruction::UnifiedTrajectory& traj,
						const std::vector<ExternalAxisSearchConfigDto>& configs, bool allowCoupledRefine,
						std::string* errMsg) const = 0;
};

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_IEXTERNALAXISSEARCHSERVICE_H
