/// @file ToWorkpieceInHandOp.cpp
/// @brief ToWorkpieceInHand 轨迹算子

// 工具型末端轨迹 → 工件型（对固定外部 TCP）
#include "ToWorkpieceInHandOp.h"

#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryOpParamsParse.h"
#include "TrajectoryUnifiedScope.h"
#include "UnifiedTrajectorySemanticMath.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace trajectory_algo
{
namespace
{
constexpr double kPrecisionConfusion = 1e-7;
constexpr double kMinSpeedRatio = 0.01;
constexpr double kMaxSpeedRatio = 100.0;

TrajectoryOpParamField boolField(const std::string& key, const std::string& labelEn, const std::string& labelZh,
								 const bool defaultValue, const int order, const std::string& group)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Bool;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.defaultBool = defaultValue;
	field.order = order;
	field.group = group;
	return field;
}

engine::RigidTransform buildExternalTcp(const RobotInstruction::ToWorkpieceInHandParams& p)
{
	return engine::RigidTransform::fromTranslationEulerDeg(p.externalTcpXMm, p.externalTcpYMm, p.externalTcpZMm,
														   p.externalTcpRxDeg, p.externalTcpRyDeg, p.externalTcpRzDeg);
}

double computeSpeedRatio(const Eigen::Vector3d& prevIn, const Eigen::Vector3d& curIn, const Eigen::Vector3d& prevOut,
						 const Eigen::Vector3d& curOut)
{
	const double inLen = (curIn - prevIn).norm();
	if (inLen <= kPrecisionConfusion)
	{
		return 1.0;
	}
	const double ratio = (curOut - prevOut).norm() / inLen;
	return std::max(kMinSpeedRatio, std::min(ratio, kMaxSpeedRatio));
}

void alignQuatHemisphere(Eigen::Quaterniond& q, const Eigen::Quaterniond& prev)
{
	if (prev.dot(q) < 0.0)
	{
		q.coeffs() *= -1.0;
	}
}
} // namespace

RobotInstruction::TrajectoryOpKind ToWorkpieceInHandOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ToWorkpieceInHand;
}

const char* ToWorkpieceInHandOp::displayName(const bool chinese) const
{
	return chinese ? "转换工件型" : "ToWorkpieceInHand";
}

TrajectoryOpCapability ToWorkpieceInHandOp::capabilities() const
{
	return TrajectoryOpCapability::PreviewPoseTransform;
}

RobotInstruction::TrajectoryOpDescriptor
ToWorkpieceInHandOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ToWorkpieceInHand;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	writeToWorkpieceInHandParams(op.params, RobotInstruction::ToWorkpieceInHandParams{});
	return op;
}

std::vector<TrajectoryOpParamField> ToWorkpieceInHandOp::paramFields() const
{
	return {
		messageParamField("toWorkpiece.externalTcpBackendId", "External TCP frame backend", "外部 TCP 坐标系", 0),
		doubleParamField("toWorkpiece.externalTcpXMm", "External TCP X", "外部 TCP X", "mm", -1e6, 1e6, 0.1, 0.0, 1,
						 "toWorkpiece"),
		doubleParamField("toWorkpiece.externalTcpYMm", "External TCP Y", "外部 TCP Y", "mm", -1e6, 1e6, 0.1, 0.0, 2,
						 "toWorkpiece"),
		doubleParamField("toWorkpiece.externalTcpZMm", "External TCP Z", "外部 TCP Z", "mm", -1e6, 1e6, 0.1, 0.0, 3,
						 "toWorkpiece"),
		doubleParamField("toWorkpiece.externalTcpRxDeg", "External TCP Rx", "外部 TCP Rx", "deg", -360.0, 360.0, 0.1,
						 0.0, 4, "toWorkpiece"),
		doubleParamField("toWorkpiece.externalTcpRyDeg", "External TCP Ry", "外部 TCP Ry", "deg", -360.0, 360.0, 0.1,
						 0.0, 5, "toWorkpiece"),
		doubleParamField("toWorkpiece.externalTcpRzDeg", "External TCP Rz", "外部 TCP Rz", "deg", -360.0, 360.0, 0.1,
						 0.0, 6, "toWorkpiece"),
		boolField("toWorkpiece.enableSpeedTransform", "Speed Transform", "启用速度变换", false, 7, "toWorkpiece"),
	};
}

bool ToWorkpieceInHandOp::validate(const RobotInstruction::TrajectoryOpDescriptor& /*op*/,
								   std::string* /*errMsg*/) const
{
	return true;
}

std::string ToWorkpieceInHandOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op,
											   const bool chinese) const
{
	const RobotInstruction::ToWorkpieceInHandParams p = parseToWorkpieceInHandParams(op.params);
	char buffer[192];
	if (chinese)
	{
		std::snprintf(buffer, sizeof(buffer), "转换工件型 | TCP(%.1f,%.1f,%.1f) 速度变换=%s", p.externalTcpXMm,
					  p.externalTcpYMm, p.externalTcpZMm, p.enableSpeedTransform ? "开" : "关");
	}
	else
	{
		std::snprintf(buffer, sizeof(buffer), "ToWorkpieceInHand | TCP(%.1f,%.1f,%.1f) speed=%s", p.externalTcpXMm,
					  p.externalTcpYMm, p.externalTcpZMm, p.enableSpeedTransform ? "on" : "off");
	}
	return buffer;
}

bool ToWorkpieceInHandOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
									  RobotInstruction::UnifiedTrajectory& traj,
									  const TrajectoryOpExecutionContext& ctx, std::string* errMsg) const
{
	if (traj.points.empty())
	{
		return true;
	}
	if (!ctx.hasWorkpieceReferenceInBase)
	{
		if (errMsg)
		{
			*errMsg = "缺少当前机器人 TCP 参考位姿";
		}
		return false;
	}

	const RobotInstruction::ToWorkpieceInHandParams params = parseToWorkpieceInHandParams(op.params);
	const engine::RigidTransform baseToExternalTcp =
		ctx.hasExternalTcpFromBackend ? ctx.externalTcpInBase : buildExternalTcp(params);
	const engine::RigidTransform& baseToWorkpieceFixed = ctx.workpieceReferenceInBase;
	const engine::RigidTransform workpieceFixedToBase = baseToWorkpieceFixed.inverse();

	const std::vector<std::size_t> scoped = resolveScopedPointIndices(traj, op.scope, ctx.program);
	if (scoped.empty())
	{
		if (errMsg)
		{
			*errMsg = "toWorkpiece scope has no points";
		}
		return false;
	}

	Eigen::Quaterniond prevInputQuat = Eigen::Quaterniond::Identity();
	Eigen::Quaterniond prevOutputQuat = Eigen::Quaterniond::Identity();
	Eigen::Vector3d prevInPos = Eigen::Vector3d::Zero();
	Eigen::Vector3d prevOutPos = Eigen::Vector3d::Zero();
	bool hasPrevPose = false;

	for (const std::size_t i : scoped)
	{
		auto& point = traj.points[i];
		const Eigen::Vector3d inPos(point.poseMm.x, point.poseMm.y, point.poseMm.z);
		engine::RigidTransform baseToPathPose = rigidFromPoint(point);

		Eigen::Quaterniond inputQuat = baseToPathPose.rotation().normalized();
		if (hasPrevPose)
		{
			alignQuatHemisphere(inputQuat, prevInputQuat);
			baseToPathPose = engine::RigidTransform::fromTranslationQuat(inPos, inputQuat);
		}

		// B_T_Eout = B_T_TCP * inv(W_f_T_Ei)；F_T_W 当前为单位阵
		const engine::RigidTransform workpieceFixedToInput = workpieceFixedToBase.composeColumn(baseToPathPose);
		engine::RigidTransform baseToEndOut = baseToExternalTcp.composeColumn(workpieceFixedToInput.inverse());

		Eigen::Quaterniond outQuat = baseToEndOut.rotation().normalized();
		if (hasPrevPose)
		{
			alignQuatHemisphere(outQuat, prevOutputQuat);
			baseToEndOut =
				engine::RigidTransform::fromTranslationQuat(baseToEndOut.translationMm(), outQuat);
		}

		const Eigen::Vector3d outPos = baseToEndOut.translationMm();
		pointFromRigid(baseToEndOut, point);

		if (params.enableSpeedTransform)
		{
			const double speedRatio = hasPrevPose ? computeSpeedRatio(prevInPos, inPos, prevOutPos, outPos) : 1.0;
			point.speedMmPerSec *= speedRatio;
		}

		prevInputQuat = inputQuat;
		prevOutputQuat = outQuat;
		prevInPos = inPos;
		prevOutPos = outPos;
		hasPrevPose = true;
	}

	return true;
}

} // namespace trajectory_algo
