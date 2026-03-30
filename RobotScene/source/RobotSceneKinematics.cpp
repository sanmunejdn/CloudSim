#include "RobotSceneKinematics.h"

#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"

#include "UrdfRobotLoader.h"

#include <osg/Matrixd>

#include <QHash>
#include <QString>

namespace RobotSceneKinematics
{

bool applyJointAnglesFromDocument(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, const QVector<double>& anglesRad)
{
	if (!doc || !osg || !doc->hasRobotKinematicsBind())
	{
		return false;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePath();
	QHash<QString, osg::Matrixd> Tq;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, anglesRad, Tq, nullptr))
	{
		return false;
	}
	const QHash<QString, osg::Matrixd>& T0 = doc->robotFkMeshWorldT0();
	const QHash<QString, osg::Matrixd>& M0 = doc->robotOuterWorldAtBind();
	const QHash<QString, QString>& linkToId = doc->robotLinkNameToBackendId();
	for (auto lit = linkToId.constBegin(); lit != linkToId.constEnd(); ++lit)
	{
		const QString linkName = lit.key();
		const QString backendId = lit.value();
		if (!Tq.contains(linkName) || !T0.contains(linkName))
		{
			continue;
		}
		const auto m0It = M0.find(backendId);
		if (m0It == M0.end())
		{
			continue;
		}
		const osg::Matrixd Mnew = Tq[linkName] * osg::Matrixd::inverse(T0[linkName]) * m0It.value();
		osg->setBackendRootWorldMatrixFromWorld(backendId.toStdString(), Mnew);
	}
	return true;
}

void applyMeshWorldMatricesRelativeToBind(
	IRobotBackendPoseSink* osg,
	const QHash<QString, osg::Matrixd>& meshWorldByLink,
	const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
	const QHash<QString, QString>& linkNameToBackendId,
	const std::unordered_map<std::string, osg::Matrixd>& outerWorldAtBind)
{
	if (!osg)
	{
		return;
	}
	for (auto lit = linkNameToBackendId.constBegin(); lit != linkNameToBackendId.constEnd(); ++lit)
	{
		const QString linkName = lit.key();
		const QString backendId = lit.value();
		if (!meshWorldByLink.contains(linkName) || !fkMeshWorldT0.contains(linkName))
		{
			continue;
		}
		const auto m0It = outerWorldAtBind.find(backendId.toStdString());
		if (m0It == outerWorldAtBind.end())
		{
			continue;
		}
		const osg::Matrixd& T0 = fkMeshWorldT0[linkName];
		const osg::Matrixd Trel = meshWorldByLink[linkName] * osg::Matrixd::inverse(T0);
		const osg::Matrixd Mnew = Trel * m0It->second;
		osg->setBackendRootWorldMatrixFromWorld(backendId.toStdString(), Mnew);
	}
}

} // namespace RobotSceneKinematics
