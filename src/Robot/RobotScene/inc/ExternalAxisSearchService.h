#ifndef ROBOTSCENE_EXTERNALAXISSEARCHSERVICE_H
#define ROBOTSCENE_EXTERNALAXISSEARCHSERVICE_H

/// @file ExternalAxisSearchService.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 地轨 1D 搜索 + 可选联立微调（依赖 TeachIk）

#include "robot_scene_global.h"

#include <QString>
#include <vector>

#include <BackendDataBase.h>
#include <IExternalAxisSearchService.h>

namespace RobotInstruction
{
class ROBOT_SCENE_API ExternalAxisSearchService final : public trajectory_algo::IExternalAxisSearchService
{
public:
	ExternalAxisSearchService() = default;
	~ExternalAxisSearchService() override = default;

	/// T_flange_tool / useOrientation / Soft 与指令示教 IK 对齐
	void setRobotContext(const QString& urdfPath, const QString& ikLinkName, const std::vector<double>& seedJointRad,
						 const BackendMat4& T_flange_tool = BackendMat4::identity(), bool useOrientation = true,
						 bool allowApproximateOrientation = false);

	bool search(RobotInstruction::UnifiedTrajectory& traj,
				const std::vector<trajectory_algo::ExternalAxisSearchConfigDto>& configs, bool allowCoupledRefine,
				std::string* errMsg) const override;

private:
	QString m_urdfPath;
	QString m_ikLinkName;
	std::vector<double> m_seedJointRad;
	BackendMat4 m_T_flange_tool = BackendMat4::identity();
	bool m_useOrientation = true;
	bool m_allowApproximateOrientation = false;
};

} // namespace RobotInstruction

#endif // ROBOTSCENE_EXTERNALAXISSEARCHSERVICE_H
