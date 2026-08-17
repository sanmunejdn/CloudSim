/// @file RotateOp.cpp
/// @brief Rotate 轨迹算子

// Rotate 原子块：程序路点位姿旋转
#include "RotateOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectorySemanticMath.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace trajectory_algo
{
namespace
{
constexpr double kRotateNoOpAngleEps = 1e-9;

bool isRotateNoOp(const RobotInstruction::RotateParams& p)
{
	return std::abs(p.angleDeg) <= kRotateNoOpAngleEps && std::abs(p.endAngleDeg) <= kRotateNoOpAngleEps;
}

bool isRotateInterpolated(const RobotInstruction::RotateParams& p)
{
	return std::abs(p.angleDeg - p.endAngleDeg) > kRotateNoOpAngleEps;
}
} // namespace

RobotInstruction::TrajectoryOpKind RotateOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Rotate;
}

const char* RotateOp::displayName(const bool chinese) const
{
	return chinese ? "旋转" : "Rotate";
}

TrajectoryOpCapability RotateOp::capabilities() const
{
	return TrajectoryOpCapability::PreviewPoseTransform;
}

RobotInstruction::TrajectoryOpDescriptor
RotateOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Rotate;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	RobotInstruction::RotateParams rotate = parseRotateParams(op.params);
	rotate.frame = RobotInstruction::TransformReferenceFrame::World;
	rotate.axisZ = 1.0;
	rotate.endAngleDeg = rotate.angleDeg;
	writeRotateParams(op.params, rotate);

	return op;
}

std::vector<TrajectoryOpParamField> RotateOp::paramFields() const
{
	return {
		enumParamField("rotate.frame", "Frame", "坐标系", {"0", "1"}, {"世界系", "物体系"}, {"World", "Body"}, 0, 0,
					   "transform"),
		doubleParamField("rotate.axisX", "Axis X", "轴X", "", -1.0, 1.0, 0.001, 0.0, 1),
		doubleParamField("rotate.axisY", "Axis Y", "轴Y", "", -1.0, 1.0, 0.001, 0.0, 2),
		doubleParamField("rotate.axisZ", "Axis Z", "轴Z", "", -1.0, 1.0, 0.001, 1.0, 3),
		doubleParamField("rotate.angleDeg", "Angle(Start)", "角度(起点)", "°", -360.0, 360.0, 0.01, 0.0, 4),
		doubleParamField("rotate.endAngleDeg", "Angle(End)", "角度(终点)", "°", -360.0, 360.0, 0.01, 0.0, 5),
	};
}

bool RotateOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	const RobotInstruction::RotateParams rotate = parseRotateParams(op.params);
	if (isRotateNoOp(rotate))
	{
		return true;
	}
	const double len =
		std::sqrt(rotate.axisX * rotate.axisX + rotate.axisY * rotate.axisY + rotate.axisZ * rotate.axisZ);
	if (len < 1e-9)
	{
		if (errMsg)
		{
			*errMsg = "rotation axis is zero";
		}
		return false;
	}
	return true;
}

std::string RotateOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	const RobotInstruction::RotateParams rotate = parseRotateParams(op.params);
	char buffer[256];
	if (isRotateInterpolated(rotate))
	{
		std::snprintf(buffer, sizeof(buffer),
					  chinese ? "%s | %s | 起点%.2f° -> 终点%.2f° @(%.2f,%.2f,%.2f)"
							  : "%s | %s | Start%.2f° -> End%.2f° @(%.2f,%.2f,%.2f)",
					  displayName(chinese), frameLabel(rotate.frame, chinese).c_str(), rotate.angleDeg,
					  rotate.endAngleDeg, rotate.axisX, rotate.axisY, rotate.axisZ);
	}
	else
	{
		std::snprintf(buffer, sizeof(buffer),
					  chinese ? "%s | %s | %.2f° @(%.2f,%.2f,%.2f)" : "%s | %s | %.2f° @(%.2f,%.2f,%.2f)",
					  displayName(chinese), frameLabel(rotate.frame, chinese).c_str(), rotate.angleDeg, rotate.axisX,
					  rotate.axisY, rotate.axisZ);
	}
	return buffer;
}

bool RotateOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
						   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
						   std::string* errMsg) const
{
	return applyTranslateRotateInScope(op, traj, ctx.program);
}

} // namespace trajectory_algo
