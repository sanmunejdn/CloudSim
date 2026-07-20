/// @file TrajectoryOpBridge.cpp
/// @brief TrajectoryOpBridge 实现

#include "TrajectoryOpBridge.h"

#include "TrajectoryOpConfigRegistry.h"
#include "TrajectoryOpDescriptorCodec.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"
#include "TrajectoryOpRegistry.h"

namespace RobotInstruction
{
trajectory_algo::TrajectoryOpRegistry& trajectoryOpRegistry()
{
	return trajectory_algo::TrajectoryOpRegistry::instance();
}

void ensureTrajectoryOpBuiltinsRegistered()
{
	trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();
}

bool ensureTrajectoryOpConfigsLoaded(const std::string& resourceBaseDir, std::string* errMsg)
{
	trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();
	return trajectory_algo::TrajectoryOpConfigRegistry::instance().ensureLoaded(resourceBaseDir, errMsg);
}

TrajectoryOpDescriptor trajectoryOpDefaultUnified(const TrajectoryOpKind kind, const OpScope& scope)
{
	trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();
	return trajectory_algo::TrajectoryOpConfigRegistry::instance().defaultUnifiedOp(kind, scope);
}

const trajectory_algo::ITrajectoryOp* trajectoryOpGet(const TrajectoryOpKind kind)
{
	return trajectory_algo::TrajectoryOpRegistry::instance().get(kind);
}

std::vector<TrajectoryOpKind> trajectoryOpPaletteKinds()
{
	return trajectory_algo::TrajectoryOpRegistry::instance().paletteKinds();
}

std::string trajectoryOpKindToString(const TrajectoryOpKind kind)
{
	return trajectory_algo::TrajectoryOpRegistry::instance().kindToString(kind);
}

bool trajectoryOpKindFromString(const std::string& token, TrajectoryOpKind& out)
{
	return trajectory_algo::TrajectoryOpRegistry::instance().kindFromString(token, out);
}

std::vector<trajectory_algo::TrajectoryOpParamField>
trajectoryOpAllParamFields(const trajectory_algo::ITrajectoryOp& op)
{
	return trajectory_algo::TrajectoryOpParamAccess::allFieldsForOp(op);
}

bool trajectoryOpParamRead(const TrajectoryOpDescriptor& op, const trajectory_algo::TrajectoryOpParamField& field,
						   trajectory_algo::TrajectoryParamValue& out)
{
	return trajectory_algo::TrajectoryOpParamAccess::read(op, field, out);
}

bool trajectoryOpParamWrite(TrajectoryOpDescriptor& op, const trajectory_algo::TrajectoryOpParamField& field,
							const trajectory_algo::TrajectoryParamValue& value)
{
	return trajectory_algo::TrajectoryOpParamAccess::write(op, field, value);
}

nlohmann::json trajectoryPipelineToJson(const std::vector<TrajectoryOpDescriptor>& ops)
{
	return trajectory_algo::pipelineToJson(ops);
}

bool trajectoryPipelineFromJson(const nlohmann::json& j, std::vector<TrajectoryOpDescriptor>& out, std::string* errMsg)
{
	return trajectory_algo::pipelineFromJson(j, out, errMsg);
}

bool validateTrajectoryPipeline(const std::vector<TrajectoryOpDescriptor>& ops, std::string* errMsg)
{
	ensureTrajectoryOpBuiltinsRegistered();
	for (const TrajectoryOpDescriptor& op : ops)
	{
		if (!op.enabled)
		{
			continue;
		}
		const trajectory_algo::ITrajectoryOp* algo = trajectoryOpGet(op.kind);
		if (!algo)
		{
			if (errMsg)
			{
				*errMsg = "unknown trajectory operation kind";
			}
			return false;
		}
		std::string localErr;
		if (!algo->validate(op, &localErr))
		{
			if (errMsg)
			{
				*errMsg = localErr.empty() ? "invalid trajectory operation" : localErr;
			}
			return false;
		}
	}
	return true;
}

std::string trajectoryOpProjectTargetBackendId(const TrajectoryOpDescriptor& op)
{
	return trajectory_algo::parseProjectParams(op.params).targetBackendId;
}

void trajectoryOpSetProjectTargetBackendId(TrajectoryOpDescriptor& op, const std::string& backendId)
{
	trajectory_algo::setTrajectoryParamString(op.params, "project.targetBackendId", backendId);
}

std::string trajectoryOpNonRigidSourceBackendId(const TrajectoryOpDescriptor& op)
{
	return trajectory_algo::parseNonRigidRegistrationParams(op.params).sourceBackendId;
}

std::string trajectoryOpNonRigidTargetBackendId(const TrajectoryOpDescriptor& op)
{
	return trajectory_algo::parseNonRigidRegistrationParams(op.params).targetBackendId;
}

void trajectoryOpSetNonRigidSourceBackendId(TrajectoryOpDescriptor& op, const std::string& backendId)
{
	RobotInstruction::NonRigidRegistrationParams nrr = trajectory_algo::parseNonRigidRegistrationParams(op.params);
	nrr.sourceBackendId = backendId;
	trajectory_algo::writeNonRigidRegistrationParams(op.params, nrr);
}

void trajectoryOpSetNonRigidTargetBackendId(TrajectoryOpDescriptor& op, const std::string& backendId)
{
	RobotInstruction::NonRigidRegistrationParams nrr = trajectory_algo::parseNonRigidRegistrationParams(op.params);
	nrr.targetBackendId = backendId;
	trajectory_algo::writeNonRigidRegistrationParams(op.params, nrr);
}

std::string trajectoryOpToWorkpieceExternalTcpBackendId(const TrajectoryOpDescriptor& op)
{
	return trajectory_algo::parseToWorkpieceInHandParams(op.params).externalTcpBackendId;
}

void trajectoryOpSetToWorkpieceExternalTcpBackendId(TrajectoryOpDescriptor& op, const std::string& backendId)
{
	RobotInstruction::ToWorkpieceInHandParams p = trajectory_algo::parseToWorkpieceInHandParams(op.params);
	p.externalTcpBackendId = backendId;
	trajectory_algo::writeToWorkpieceInHandParams(op.params, p);
}

} // namespace RobotInstruction
