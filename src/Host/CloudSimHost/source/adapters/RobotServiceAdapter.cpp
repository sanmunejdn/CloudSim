#include "adapters/RobotServiceAdapter.h"

#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "DocumentHostEvents.h"
#include "IRobotUrdfImportContext.h"
#include "RobotPlanInstruction.h"
#include "RobotProgramJsonIo.h"
#include "RobotProgramStore.h"
#include "OsgWidget.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotImport.h"

namespace cloudsim::host {

RobotServiceAdapter::RobotServiceAdapter(DocumentHost& host, RobotProgramStore& programs)
	: m_host(host)
	, m_programs(programs)
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
	const QVector<double>& jointAnglesRad, QVector<double>* outAggregated, QString* outError)
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
	doc->notifyRobotKinematicsAppliedToScene();
	publishRobotKinematicsApplied(m_host, sceneRootBackendId, aggregated);
	if (OsgWidget* osg = osgWidgetFrom(m_host))
	{
		osg->requestRedraw();
	}
	return true;
}

bool RobotServiceAdapter::planInstruction(const core::MotionInstructionDto& instruction,
	const core::PlanContextDto& context, core::PlanResultDto& out, QString* outError)
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

} // namespace cloudsim::host
