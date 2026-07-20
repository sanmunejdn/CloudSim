/// @file TrajectoryOpParamsParse.cpp
/// @brief TrajectoryOpParamsParse 实现

#include "TrajectoryOpParamsParse.h"

namespace trajectory_algo
{
namespace
{
constexpr const char* kTranslateFrame = "translate.frame";
constexpr const char* kTranslateDx = "translate.dxMm";
constexpr const char* kTranslateDy = "translate.dyMm";
constexpr const char* kTranslateDz = "translate.dzMm";
constexpr const char* kTranslateEndDx = "translate.endDxMm";
constexpr const char* kTranslateEndDy = "translate.endDyMm";
constexpr const char* kTranslateEndDz = "translate.endDzMm";

constexpr const char* kRotateFrame = "rotate.frame";
constexpr const char* kRotateAxisX = "rotate.axisX";
constexpr const char* kRotateAxisY = "rotate.axisY";
constexpr const char* kRotateAxisZ = "rotate.axisZ";
constexpr const char* kRotateAngle = "rotate.angleDeg";
constexpr const char* kRotateEndAngle = "rotate.endAngleDeg";

double lerp(const double a, const double b, const double t)
{
	return a + (b - a) * t;
}

} // namespace

double trajectoryParamDouble(const nlohmann::json& params, const char* key, const double defaultValue)
{
	if (!params.is_object() || !params.contains(key))
	{
		return defaultValue;
	}
	const nlohmann::json& item = params.at(key);
	if (item.is_number())
	{
		return item.get<double>();
	}
	return defaultValue;
}

int trajectoryParamInt(const nlohmann::json& params, const char* key, const int defaultValue)
{
	if (!params.is_object() || !params.contains(key))
	{
		return defaultValue;
	}
	const nlohmann::json& item = params.at(key);
	if (item.is_number_integer())
	{
		return item.get<int>();
	}
	if (item.is_number())
	{
		return static_cast<int>(item.get<double>());
	}
	return defaultValue;
}

bool trajectoryParamBool(const nlohmann::json& params, const char* key, const bool defaultValue)
{
	if (!params.is_object() || !params.contains(key))
	{
		return defaultValue;
	}
	const nlohmann::json& item = params.at(key);
	if (item.is_boolean())
	{
		return item.get<bool>();
	}
	return defaultValue;
}

std::string trajectoryParamString(const nlohmann::json& params, const char* key, const std::string& defaultValue)
{
	if (!params.is_object() || !params.contains(key))
	{
		return defaultValue;
	}
	const nlohmann::json& item = params.at(key);
	if (item.is_string())
	{
		return item.get<std::string>();
	}
	return defaultValue;
}

void setTrajectoryParamDouble(nlohmann::json& params, const char* key, const double value)
{
	if (!params.is_object())
	{
		params = nlohmann::json::object();
	}
	params[key] = value;
}

void setTrajectoryParamInt(nlohmann::json& params, const char* key, const int value)
{
	if (!params.is_object())
	{
		params = nlohmann::json::object();
	}
	params[key] = value;
}

void setTrajectoryParamBool(nlohmann::json& params, const char* key, const bool value)
{
	if (!params.is_object())
	{
		params = nlohmann::json::object();
	}
	params[key] = value;
}

void setTrajectoryParamString(nlohmann::json& params, const char* key, const std::string& value)
{
	if (!params.is_object())
	{
		params = nlohmann::json::object();
	}
	params[key] = value;
}

RobotInstruction::TranslateParams parseTranslateParams(const nlohmann::json& params)
{
	RobotInstruction::TranslateParams out{};
	out.frame = static_cast<RobotInstruction::TransformReferenceFrame>(
		trajectoryParamInt(params, kTranslateFrame, static_cast<int>(out.frame)));
	out.dxMm = trajectoryParamDouble(params, kTranslateDx, out.dxMm);
	out.dyMm = trajectoryParamDouble(params, kTranslateDy, out.dyMm);
	out.dzMm = trajectoryParamDouble(params, kTranslateDz, out.dzMm);
	out.endDxMm =
		params.contains(kTranslateEndDx) ? trajectoryParamDouble(params, kTranslateEndDx, out.dxMm) : out.dxMm;
	out.endDyMm =
		params.contains(kTranslateEndDy) ? trajectoryParamDouble(params, kTranslateEndDy, out.dyMm) : out.dyMm;
	out.endDzMm =
		params.contains(kTranslateEndDz) ? trajectoryParamDouble(params, kTranslateEndDz, out.dzMm) : out.dzMm;
	return out;
}

void writeTranslateParams(nlohmann::json& params, const RobotInstruction::TranslateParams& value)
{
	setTrajectoryParamInt(params, kTranslateFrame, static_cast<int>(value.frame));
	setTrajectoryParamDouble(params, kTranslateDx, value.dxMm);
	setTrajectoryParamDouble(params, kTranslateDy, value.dyMm);
	setTrajectoryParamDouble(params, kTranslateDz, value.dzMm);
	setTrajectoryParamDouble(params, kTranslateEndDx, value.endDxMm);
	setTrajectoryParamDouble(params, kTranslateEndDy, value.endDyMm);
	setTrajectoryParamDouble(params, kTranslateEndDz, value.endDzMm);
}

RobotInstruction::RotateParams parseRotateParams(const nlohmann::json& params)
{
	RobotInstruction::RotateParams out{};
	out.frame = static_cast<RobotInstruction::TransformReferenceFrame>(
		trajectoryParamInt(params, kRotateFrame, static_cast<int>(out.frame)));
	out.axisX = trajectoryParamDouble(params, kRotateAxisX, out.axisX);
	out.axisY = trajectoryParamDouble(params, kRotateAxisY, out.axisY);
	out.axisZ = trajectoryParamDouble(params, kRotateAxisZ, out.axisZ);
	out.angleDeg = trajectoryParamDouble(params, kRotateAngle, out.angleDeg);
	out.endAngleDeg =
		params.contains(kRotateEndAngle) ? trajectoryParamDouble(params, kRotateEndAngle, out.angleDeg) : out.angleDeg;
	return out;
}

void writeRotateParams(nlohmann::json& params, const RobotInstruction::RotateParams& value)
{
	setTrajectoryParamInt(params, kRotateFrame, static_cast<int>(value.frame));
	setTrajectoryParamDouble(params, kRotateAxisX, value.axisX);
	setTrajectoryParamDouble(params, kRotateAxisY, value.axisY);
	setTrajectoryParamDouble(params, kRotateAxisZ, value.axisZ);
	setTrajectoryParamDouble(params, kRotateAngle, value.angleDeg);
	setTrajectoryParamDouble(params, kRotateEndAngle, value.endAngleDeg);
}

int parseMirrorAxis(const nlohmann::json& params)
{
	return trajectoryParamInt(params, "mirror.axis", 0);
}

void writeMirrorAxis(nlohmann::json& params, const int axis)
{
	setTrajectoryParamInt(params, "mirror.axis", axis);
}

int parseDuplicateCount(const nlohmann::json& params)
{
	return trajectoryParamInt(params, "structural.duplicateCount", 1);
}

void writeDuplicateCount(nlohmann::json& params, const int count)
{
	setTrajectoryParamInt(params, "structural.duplicateCount", count);
}

RobotInstruction::ResampleParams parseResampleParams(const nlohmann::json& params)
{
	RobotInstruction::ResampleParams out{};
	out.stepMm = trajectoryParamDouble(params, "resample.stepMm", out.stepMm);
	return out;
}

void writeResampleParams(nlohmann::json& params, const RobotInstruction::ResampleParams& value)
{
	setTrajectoryParamDouble(params, "resample.stepMm", value.stepMm);
}

RobotInstruction::PathOffsetParams parsePathOffsetParams(const nlohmann::json& params)
{
	RobotInstruction::PathOffsetParams out{};
	out.offsetMm = trajectoryParamDouble(params, "offset.offsetMm", out.offsetMm);
	out.lateralMm = trajectoryParamDouble(params, "offset.lateralMm", out.lateralMm);
	return out;
}

void writePathOffsetParams(nlohmann::json& params, const RobotInstruction::PathOffsetParams& value)
{
	setTrajectoryParamDouble(params, "offset.offsetMm", value.offsetMm);
	setTrajectoryParamDouble(params, "offset.lateralMm", value.lateralMm);
}

RobotInstruction::WeaveParams parseWeaveParams(const nlohmann::json& params)
{
	RobotInstruction::WeaveParams out{};
	out.amplitudeMm = trajectoryParamDouble(params, "weave.amplitudeMm", out.amplitudeMm);
	out.periodMm = trajectoryParamDouble(params, "weave.periodMm", out.periodMm);
	return out;
}

void writeWeaveParams(nlohmann::json& params, const RobotInstruction::WeaveParams& value)
{
	setTrajectoryParamDouble(params, "weave.amplitudeMm", value.amplitudeMm);
	setTrajectoryParamDouble(params, "weave.periodMm", value.periodMm);
}

RobotInstruction::AssignMotionParams parseAssignMotionParams(const nlohmann::json& params)
{
	RobotInstruction::AssignMotionParams out{};
	out.blendRadiusMm = trajectoryParamDouble(params, "assign.blendRadiusMm", out.blendRadiusMm);
	out.speedMmPerSec = trajectoryParamDouble(params, "assign.speedMmPerSec", out.speedMmPerSec);
	return out;
}

void writeAssignMotionParams(nlohmann::json& params, const RobotInstruction::AssignMotionParams& value)
{
	setTrajectoryParamDouble(params, "assign.blendRadiusMm", value.blendRadiusMm);
	setTrajectoryParamDouble(params, "assign.speedMmPerSec", value.speedMmPerSec);
}

namespace
{
RobotInstruction::ApproachParams parseApproachLikeParams(const nlohmann::json& params, const char* prefix,
														 const double defaultCustomZ)
{
	RobotInstruction::ApproachParams out{};
	const std::string distanceKey = std::string(prefix) + ".distanceMm";
	const std::string directionModeKey = std::string(prefix) + ".directionMode";
	const std::string directionFrameKey = std::string(prefix) + ".directionFrame";
	const std::string customXKey = std::string(prefix) + ".customDirection.x";
	const std::string customYKey = std::string(prefix) + ".customDirection.y";
	const std::string customZKey = std::string(prefix) + ".customDirection.z";
	const std::string insertModeKey = std::string(prefix) + ".insertMode";
	const std::string segmentSelectKey = std::string(prefix) + ".segmentSelectMode";
	const std::string segmentFromKey = std::string(prefix) + ".segmentFrom";
	const std::string segmentToKey = std::string(prefix) + ".segmentTo";
	const std::string overrideSpeedKey = std::string(prefix) + ".overrideSpeedEnabled";
	const std::string speedKey = std::string(prefix) + ".speedMmPerSec";

	out.distanceMm = trajectoryParamDouble(params, distanceKey.c_str(), out.distanceMm);
	out.directionMode = static_cast<RobotInstruction::ApproachDirectionMode>(
		trajectoryParamInt(params, directionModeKey.c_str(), static_cast<int>(out.directionMode)));
	out.directionFrame = static_cast<RobotInstruction::TransformReferenceFrame>(
		trajectoryParamInt(params, directionFrameKey.c_str(), static_cast<int>(out.directionFrame)));
	out.customDirectionX = trajectoryParamDouble(params, customXKey.c_str(), out.customDirectionX);
	out.customDirectionY = trajectoryParamDouble(params, customYKey.c_str(), out.customDirectionY);
	out.customDirectionZ = trajectoryParamDouble(params, customZKey.c_str(), defaultCustomZ);
	out.insertMode = static_cast<RobotInstruction::InsertMode>(
		trajectoryParamInt(params, insertModeKey.c_str(), static_cast<int>(out.insertMode)));
	out.segmentSelectMode = static_cast<RobotInstruction::SegmentSelectMode>(
		trajectoryParamInt(params, segmentSelectKey.c_str(), static_cast<int>(out.segmentSelectMode)));
	out.segmentFrom = trajectoryParamInt(params, segmentFromKey.c_str(), out.segmentFrom);
	out.segmentTo = trajectoryParamInt(params, segmentToKey.c_str(), out.segmentTo);
	out.overrideSpeedEnabled = trajectoryParamBool(params, overrideSpeedKey.c_str(), out.overrideSpeedEnabled);
	out.speedMmPerSec = trajectoryParamDouble(params, speedKey.c_str(), out.speedMmPerSec);
	return out;
}

void writeApproachLikeParams(nlohmann::json& params, const RobotInstruction::ApproachParams& value, const char* prefix)
{
	const std::string distanceKey = std::string(prefix) + ".distanceMm";
	const std::string directionModeKey = std::string(prefix) + ".directionMode";
	const std::string directionFrameKey = std::string(prefix) + ".directionFrame";
	const std::string customXKey = std::string(prefix) + ".customDirection.x";
	const std::string customYKey = std::string(prefix) + ".customDirection.y";
	const std::string customZKey = std::string(prefix) + ".customDirection.z";
	const std::string insertModeKey = std::string(prefix) + ".insertMode";
	const std::string segmentSelectKey = std::string(prefix) + ".segmentSelectMode";
	const std::string segmentFromKey = std::string(prefix) + ".segmentFrom";
	const std::string segmentToKey = std::string(prefix) + ".segmentTo";
	const std::string overrideSpeedKey = std::string(prefix) + ".overrideSpeedEnabled";
	const std::string speedKey = std::string(prefix) + ".speedMmPerSec";

	setTrajectoryParamDouble(params, distanceKey.c_str(), value.distanceMm);
	setTrajectoryParamInt(params, directionModeKey.c_str(), static_cast<int>(value.directionMode));
	setTrajectoryParamInt(params, directionFrameKey.c_str(), static_cast<int>(value.directionFrame));
	setTrajectoryParamDouble(params, customXKey.c_str(), value.customDirectionX);
	setTrajectoryParamDouble(params, customYKey.c_str(), value.customDirectionY);
	setTrajectoryParamDouble(params, customZKey.c_str(), value.customDirectionZ);
	setTrajectoryParamInt(params, insertModeKey.c_str(), static_cast<int>(value.insertMode));
	setTrajectoryParamInt(params, segmentSelectKey.c_str(), static_cast<int>(value.segmentSelectMode));
	setTrajectoryParamInt(params, segmentFromKey.c_str(), value.segmentFrom);
	setTrajectoryParamInt(params, segmentToKey.c_str(), value.segmentTo);
	setTrajectoryParamBool(params, overrideSpeedKey.c_str(), value.overrideSpeedEnabled);
	setTrajectoryParamDouble(params, speedKey.c_str(), value.speedMmPerSec);
}

} // namespace

RobotInstruction::ApproachParams parseApproachParams(const nlohmann::json& params)
{
	return parseApproachLikeParams(params, "approach", -1.0);
}

void writeApproachParams(nlohmann::json& params, const RobotInstruction::ApproachParams& value)
{
	writeApproachLikeParams(params, value, "approach");
}

RobotInstruction::RetractParams parseRetractParams(const nlohmann::json& params)
{
	const RobotInstruction::ApproachParams approach = parseApproachLikeParams(params, "retract", 1.0);
	RobotInstruction::RetractParams out{};
	out.distanceMm = approach.distanceMm;
	out.directionMode = approach.directionMode;
	out.directionFrame = approach.directionFrame;
	out.customDirectionX = approach.customDirectionX;
	out.customDirectionY = approach.customDirectionY;
	out.customDirectionZ = approach.customDirectionZ;
	out.insertMode = approach.insertMode;
	out.segmentSelectMode = approach.segmentSelectMode;
	out.segmentFrom = approach.segmentFrom;
	out.segmentTo = approach.segmentTo;
	out.overrideSpeedEnabled = approach.overrideSpeedEnabled;
	out.speedMmPerSec = approach.speedMmPerSec;
	return out;
}

void writeRetractParams(nlohmann::json& params, const RobotInstruction::RetractParams& value)
{
	RobotInstruction::ApproachParams approach{};
	approach.distanceMm = value.distanceMm;
	approach.directionMode = value.directionMode;
	approach.directionFrame = value.directionFrame;
	approach.customDirectionX = value.customDirectionX;
	approach.customDirectionY = value.customDirectionY;
	approach.customDirectionZ = value.customDirectionZ;
	approach.insertMode = value.insertMode;
	approach.segmentSelectMode = value.segmentSelectMode;
	approach.segmentFrom = value.segmentFrom;
	approach.segmentTo = value.segmentTo;
	approach.overrideSpeedEnabled = value.overrideSpeedEnabled;
	approach.speedMmPerSec = value.speedMmPerSec;
	writeApproachLikeParams(params, approach, "retract");
}

RobotInstruction::ProjectToGeometryParams parseProjectParams(const nlohmann::json& params)
{
	RobotInstruction::ProjectToGeometryParams out{};
	out.targetBackendId = trajectoryParamString(params, "project.targetBackendId", out.targetBackendId);
	out.directionFrame = static_cast<RobotInstruction::TransformReferenceFrame>(
		trajectoryParamInt(params, "project.directionFrame", static_cast<int>(out.directionFrame)));
	out.directionX = trajectoryParamDouble(params, "project.direction.x", out.directionX);
	out.directionY = trajectoryParamDouble(params, "project.direction.y", out.directionY);
	out.directionZ = trajectoryParamDouble(params, "project.direction.z", out.directionZ);
	out.maxDistanceMm = trajectoryParamDouble(params, "project.maxDistanceMm", out.maxDistanceMm);
	out.pointCloudHitRadiusMm =
		trajectoryParamDouble(params, "project.pointCloudHitRadiusMm", out.pointCloudHitRadiusMm);
	return out;
}

void writeProjectParams(nlohmann::json& params, const RobotInstruction::ProjectToGeometryParams& value)
{
	setTrajectoryParamString(params, "project.targetBackendId", value.targetBackendId);
	setTrajectoryParamInt(params, "project.directionFrame", static_cast<int>(value.directionFrame));
	setTrajectoryParamDouble(params, "project.direction.x", value.directionX);
	setTrajectoryParamDouble(params, "project.direction.y", value.directionY);
	setTrajectoryParamDouble(params, "project.direction.z", value.directionZ);
	setTrajectoryParamDouble(params, "project.maxDistanceMm", value.maxDistanceMm);
	setTrajectoryParamDouble(params, "project.pointCloudHitRadiusMm", value.pointCloudHitRadiusMm);
}

RobotInstruction::NonRigidRegistrationParams parseNonRigidRegistrationParams(const nlohmann::json& params)
{
	RobotInstruction::NonRigidRegistrationParams out{};
	out.sourceBackendId = trajectoryParamString(params, "nrr.sourceBackendId", out.sourceBackendId);
	out.targetBackendId = trajectoryParamString(params, "nrr.targetBackendId", out.targetBackendId);
	out.maxBindDistanceMm = trajectoryParamDouble(params, "nrr.maxBindDistanceMm", out.maxBindDistanceMm);
	out.sampleRadiusRatio = trajectoryParamDouble(params, "nrr.sampleRadiusRatio", out.sampleRadiusRatio);
	out.maxOuterIters = trajectoryParamInt(params, "nrr.maxOuterIters", out.maxOuterIters);
	out.rigidPreAlign = trajectoryParamBool(params, "nrr.rigidPreAlign", out.rigidPreAlign);
	out.voxelPrefilterMm = trajectoryParamDouble(params, "nrr.voxelPrefilterMm", out.voxelPrefilterMm);
	return out;
}

void writeNonRigidRegistrationParams(nlohmann::json& params, const RobotInstruction::NonRigidRegistrationParams& value)
{
	setTrajectoryParamString(params, "nrr.sourceBackendId", value.sourceBackendId);
	setTrajectoryParamString(params, "nrr.targetBackendId", value.targetBackendId);
	setTrajectoryParamDouble(params, "nrr.maxBindDistanceMm", value.maxBindDistanceMm);
	setTrajectoryParamDouble(params, "nrr.sampleRadiusRatio", value.sampleRadiusRatio);
	setTrajectoryParamInt(params, "nrr.maxOuterIters", value.maxOuterIters);
	setTrajectoryParamBool(params, "nrr.rigidPreAlign", value.rigidPreAlign);
	setTrajectoryParamDouble(params, "nrr.voxelPrefilterMm", value.voxelPrefilterMm);
}

RobotInstruction::ToWorkpieceInHandParams parseToWorkpieceInHandParams(const nlohmann::json& params)
{
	RobotInstruction::ToWorkpieceInHandParams out{};
	out.externalTcpBackendId =
		trajectoryParamString(params, "toWorkpiece.externalTcpBackendId", out.externalTcpBackendId);
	out.externalTcpXMm = trajectoryParamDouble(params, "toWorkpiece.externalTcpXMm", out.externalTcpXMm);
	out.externalTcpYMm = trajectoryParamDouble(params, "toWorkpiece.externalTcpYMm", out.externalTcpYMm);
	out.externalTcpZMm = trajectoryParamDouble(params, "toWorkpiece.externalTcpZMm", out.externalTcpZMm);
	out.externalTcpRxDeg = trajectoryParamDouble(params, "toWorkpiece.externalTcpRxDeg", out.externalTcpRxDeg);
	out.externalTcpRyDeg = trajectoryParamDouble(params, "toWorkpiece.externalTcpRyDeg", out.externalTcpRyDeg);
	out.externalTcpRzDeg = trajectoryParamDouble(params, "toWorkpiece.externalTcpRzDeg", out.externalTcpRzDeg);
	out.enableSpeedTransform = trajectoryParamBool(params, "toWorkpiece.enableSpeedTransform", out.enableSpeedTransform);
	return out;
}

void writeToWorkpieceInHandParams(nlohmann::json& params, const RobotInstruction::ToWorkpieceInHandParams& value)
{
	setTrajectoryParamString(params, "toWorkpiece.externalTcpBackendId", value.externalTcpBackendId);
	setTrajectoryParamDouble(params, "toWorkpiece.externalTcpXMm", value.externalTcpXMm);
	setTrajectoryParamDouble(params, "toWorkpiece.externalTcpYMm", value.externalTcpYMm);
	setTrajectoryParamDouble(params, "toWorkpiece.externalTcpZMm", value.externalTcpZMm);
	setTrajectoryParamDouble(params, "toWorkpiece.externalTcpRxDeg", value.externalTcpRxDeg);
	setTrajectoryParamDouble(params, "toWorkpiece.externalTcpRyDeg", value.externalTcpRyDeg);
	setTrajectoryParamDouble(params, "toWorkpiece.externalTcpRzDeg", value.externalTcpRzDeg);
	setTrajectoryParamBool(params, "toWorkpiece.enableSpeedTransform", value.enableSpeedTransform);
}

void interpolateTransformParamsInPlace(RobotInstruction::TrajectoryOpDescriptor& op, const double t)
{
	if (op.kind == RobotInstruction::TrajectoryOpKind::Translate)
	{
		RobotInstruction::TranslateParams translate = parseTranslateParams(op.params);
		translate.dxMm = lerp(translate.dxMm, translate.endDxMm, t);
		translate.dyMm = lerp(translate.dyMm, translate.endDyMm, t);
		translate.dzMm = lerp(translate.dzMm, translate.endDzMm, t);
		translate.endDxMm = translate.dxMm;
		translate.endDyMm = translate.dyMm;
		translate.endDzMm = translate.dzMm;
		writeTranslateParams(op.params, translate);
	}
	else if (op.kind == RobotInstruction::TrajectoryOpKind::Rotate)
	{
		RobotInstruction::RotateParams rotate = parseRotateParams(op.params);
		rotate.angleDeg = lerp(rotate.angleDeg, rotate.endAngleDeg, t);
		rotate.endAngleDeg = rotate.angleDeg;
		writeRotateParams(op.params, rotate);
	}
}

void finalizeTransformDefaultParams(RobotInstruction::TrajectoryOpDescriptor& op)
{
	if (op.kind == RobotInstruction::TrajectoryOpKind::Translate)
	{
		RobotInstruction::TranslateParams translate = parseTranslateParams(op.params);
		if (!op.params.contains(kTranslateEndDx))
		{
			translate.endDxMm = translate.dxMm;
		}
		if (!op.params.contains(kTranslateEndDy))
		{
			translate.endDyMm = translate.dyMm;
		}
		if (!op.params.contains(kTranslateEndDz))
		{
			translate.endDzMm = translate.dzMm;
		}
		writeTranslateParams(op.params, translate);
	}
	else if (op.kind == RobotInstruction::TrajectoryOpKind::Rotate)
	{
		RobotInstruction::RotateParams rotate = parseRotateParams(op.params);
		if (!op.params.contains(kRotateEndAngle))
		{
			rotate.endAngleDeg = rotate.angleDeg;
		}
		writeRotateParams(op.params, rotate);
	}
}

} // namespace trajectory_algo
