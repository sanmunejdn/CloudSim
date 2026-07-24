/// @file ManualPoseSource.cpp
/// @brief 手动末端位姿

#include "IRobotPoseSource.h"

namespace industrial_camera
{
namespace
{

class ManualPoseSource final : public IRobotPoseSource
{
public:
	explicit ManualPoseSource(const Pose6d& p)
		: pose_(p)
	{
	}

	bool getCurrentPose(Pose6d& out) override
	{
		out = pose_;
		return true;
	}

	std::string lastError() const override { return {}; }

	void setPose(const Pose6d& p) { pose_ = p; }

private:
	Pose6d pose_;
};

} // namespace

std::unique_ptr<IRobotPoseSource> createManualPoseSource(const Pose6d& initial)
{
	return std::make_unique<ManualPoseSource>(initial);
}

void setManualPose(IRobotPoseSource* src, const Pose6d& pose)
{
	if (auto* m = dynamic_cast<ManualPoseSource*>(src))
		m->setPose(pose);
}

} // namespace industrial_camera
