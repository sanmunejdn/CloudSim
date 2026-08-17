#ifndef ROBOTSCENE_TRAJECTORYREACHABILITYPROBESERVICE_H
#define ROBOTSCENE_TRAJECTORYREACHABILITYPROBESERVICE_H

/// @file TrajectoryReachabilityProbeService.h
/// @brief 用 TeachIk 标记 UnifiedTrajectory 点可达性

#include "robot_scene_global.h"

#include <ITrajectoryReachabilityProbe.h>

#include <QString>
#include <cstddef>
#include <vector>

namespace RobotInstruction
{
struct ROBOT_SCENE_API ReachabilityProbeLastStats
{
	bool valid = false;
	std::size_t probedCount = 0;
	std::size_t unreachableCount = 0;
};

ROBOT_SCENE_API ReachabilityProbeLastStats trajectoryReachabilityLastStats();
ROBOT_SCENE_API void resetTrajectoryReachabilityLastStats();

class ROBOT_SCENE_API TrajectoryReachabilityProbeService final : public trajectory_algo::ITrajectoryReachabilityProbe
{
public:
	TrajectoryReachabilityProbeService() = default;
	~TrajectoryReachabilityProbeService() override = default;

	void setRobotContext(const QString& urdfPath, const QString& ikLinkName, const std::vector<double>& seedJointRad);

	bool probe(RobotInstruction::UnifiedTrajectory& traj, const std::vector<std::size_t>& indices, bool useOrientation,
			   double residualTolMm, std::string* errMsg) const override;

private:
	QString m_urdfPath;
	QString m_ikLinkName;
	std::vector<double> m_seedJointRad;
};

} // namespace RobotInstruction

#endif // ROBOTSCENE_TRAJECTORYREACHABILITYPROBESERVICE_H
