/// @file RobotServiceAdapter.cpp
/// @brief RobotServiceAdapter 实现

#include "adapters/RobotServiceAdapter.h"

#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "DocumentHostEvents.h"
#include "IRobotInstructionPropertyDelegate.h"
#include "IRobotUrdfImportContext.h"
#include "OsgWidget.h"
#include "RobotPlanInstruction.h"
#include "RobotProgramJsonIo.h"
#include "RobotProgramStore.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotImport.h"

namespace cloudsim::host
{
RobotServiceAdapter::RobotServiceAdapter(DocumentHost& host, RobotProgramStore& programs)
	: m_host(host), m_programs(programs)
{
}

core::RobotRegistrationDto RobotServiceAdapter::registerUrdfRobot(const QString& urdfPath,
																  const core::ImportOptionsDto& options)
{
	IRobotUrdfImportContext* ctx = m_host.robotUrdfImportContext();
	if (!ctx)
	{
		return {false, QStringLiteral("URDF import: document has no robot import context"), {}, {}, 0, 0, {}};
	}
	const core::RobotRegistrationDto result = importUrdfRobot(*ctx, urdfPath, options);
	if (result.ok)
	{
		publishBackendObjectRegistered(m_host, result.sceneRootBackendId, QStringLiteral("RobotURDF"));
	}
	return result;
}

bool RobotServiceAdapter::applyJointAnglesRad(const core::ObjectId& sceneRootBackendId,
											  const QVector<double>& jointAnglesRad, QVector<double>* outAggregated,
											  QString* outError)
{
	IRobotUrdfImportContext* ctx = m_host.robotUrdfImportContext();
	if (!ctx)
	{
		if (outError)
		{
			*outError = QStringLiteral("no robot import context");
		}
		return false;
	}
	IRobotSimulationDocument* doc = ctx->urdfImportRobotSimulationDocument();
	IRobotBackendPoseSink* poseSink = ctx->urdfImportScenePoseSink();
	if (!doc || !poseSink || !doc->hasRobotSimulationContext())
	{
		if (outError)
		{
			*outError = QStringLiteral("no robot simulation context");
		}
		return false;
	}
	const int instIdx = ctx->robotInstanceIndexForSceneBackendId(sceneRootBackendId);
	if (instIdx < 0)
	{
		if (outError)
		{
			*outError = QStringLiteral("unknown sceneRootBackendId: %1").arg(sceneRootBackendId);
		}
		return false;
	}
	const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
	if (jointAnglesRad.size() != nj)
	{
		if (outError)
		{
			*outError = QStringLiteral("joint count mismatch: expected %1, got %2").arg(nj).arg(jointAnglesRad.size());
		}
		return false;
	}
	QVector<double> aggregated(doc->robotRevoluteJointNames().size(), 0.0);
	if (!RobotSceneKinematics::applyJointAnglesForInstance(doc, poseSink, instIdx, jointAnglesRad, aggregated))
	{
		if (outError)
		{
			*outError = QStringLiteral("applyJointAnglesForInstance failed");
		}
		return false;
	}
	if (outAggregated)
	{
		*outAggregated = aggregated;
	}
	// 跟随脏标记 + 同步求解已在 applyJointAnglesFromDocument → notifyRobotKinematicsAppliedToScene
	publishRobotKinematicsApplied(m_host, sceneRootBackendId, aggregated);
	if (OsgWidget* osg = osgWidgetFrom(m_host))
	{
		osg->requestRedraw();
	}
	return true;
}

bool RobotServiceAdapter::planInstruction(const core::MotionInstructionDto& instruction,
										  const core::PlanContextDto& context, core::PlanResultDto& out,
										  QString* outError)
{
	IRobotUrdfImportContext* ctx = m_host.robotUrdfImportContext();
	if (!ctx)
	{
		if (outError)
		{
			*outError = QStringLiteral("no robot import context");
		}
		return false;
	}
	return planMotionInstruction(*ctx, instruction, context, out, outError);
}

QJsonArray RobotServiceAdapter::robotProgramsJson() const
{
	return robotProgramsToJson(m_programs);
}

bool RobotServiceAdapter::setRobotProgramsJson(const QJsonArray& programs, QString* outError)
{
	IRobotUrdfImportContext* ctx = m_host.robotUrdfImportContext();
	if (!ctx)
	{
		if (outError)
		{
			*outError = QStringLiteral("no robot import context");
		}
		return false;
	}
	return robotProgramsFromJson(m_programs, programs, *ctx, outError);
}

namespace
{
IRobotInstructionPropertyDelegate* mutableInstructionDelegate(const DocumentHost& host)
{
	return const_cast<IRobotInstructionPropertyDelegate*>(host.instructionPropertyDelegate());
}
} // namespace

QVector<core::PropertyRowDto> RobotServiceAdapter::instructionPropertyRows(const QString& instructionId) const
{
	IRobotInstructionPropertyDelegate* delegate = mutableInstructionDelegate(m_host);
	if (!delegate)
	{
		return {};
	}
	return delegate->instructionPropertyRows(instructionId);
}

bool RobotServiceAdapter::applyInstructionPropertyChange(const QString& instructionId, const QString& key,
														 const QString& value, QString* outError)
{
	IRobotInstructionPropertyDelegate* delegate = mutableInstructionDelegate(m_host);
	if (!delegate)
	{
		if (outError)
		{
			*outError = QStringLiteral("no instruction property delegate");
		}
		return false;
	}
	return delegate->applyInstructionPropertyChange(instructionId, key, value, outError);
}

QStringList RobotServiceAdapter::feasibleMotionAxisConfigTokens(const QString& instructionId,
																const core::MotionInstructionDto& instruction,
																const core::PlanContextDto& context) const
{
	(void)instruction;
	(void)context;
	return queryFeasibleMotionAxisOptions(instructionId, nullptr).presetTokens;
}

core::FeasibleMotionAxisOptionsDto
RobotServiceAdapter::queryFeasibleMotionAxisOptions(const QString& instructionId,
													QVector<double>* outSeedJointRad) const
{
	IRobotInstructionPropertyDelegate* delegate = mutableInstructionDelegate(m_host);
	if (!delegate)
	{
		return {};
	}
	return delegate->queryFeasibleMotionAxisOptions(instructionId, outSeedJointRad);
}

core::FeasibleMotionAxisOptionsDto RobotServiceAdapter::cachedFeasibleMotionAxisOptions() const
{
	IRobotInstructionPropertyDelegate* delegate = mutableInstructionDelegate(m_host);
	if (!delegate)
	{
		return {};
	}
	return delegate->cachedFeasibleMotionAxisOptions();
}

} // namespace cloudsim::host
