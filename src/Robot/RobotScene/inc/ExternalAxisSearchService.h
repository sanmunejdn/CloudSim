#ifndef ROBOTSCENE_EXTERNALAXISSEARCHSERVICE_H
#define ROBOTSCENE_EXTERNALAXISSEARCHSERVICE_H

/// @file ExternalAxisSearchService.h
/// @brief 地轨 1D 搜索 + 可选联立微调（依赖 TeachIk）

#include "robot_scene_global.h"

#include <IExternalAxisSearchService.h>

#include <QString>
#include <vector>

namespace RobotInstruction
{
class ROBOT_SCENE_API ExternalAxisSearchService final : public trajectory_algo::IExternalAxisSearchService
{
public:
	ExternalAxisSearchService() = default;
	~ExternalAxisSearchService() override = default;

	void setRobotContext(const QString& urdfPath, const QString& ikLinkName, const std::vector<double>& seedJointRad);

	bool search(RobotInstruction::UnifiedTrajectory& traj,
				const std::vector<trajectory_algo::ExternalAxisSearchConfigDto>& configs, bool allowCoupledRefine,
				std::string* errMsg) const override;

private:
	QString m_urdfPath;
	QString m_ikLinkName;
	std::vector<double> m_seedJointRad;
};

} // namespace RobotInstruction

#endif // ROBOTSCENE_EXTERNALAXISSEARCHSERVICE_H
