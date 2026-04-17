#include "RobotSceneKinematics.h"

#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"

#include "UrdfRobotLoader.h"
#include "RunLogger.h"

#include <osg/Matrixd>
#include <osg/MatrixTransform>

#include <QByteArray>
#include <QHash>
#include <QString>

namespace
{
std::string qToUtf8Std(const QString& s)
{
	const QByteArray utf8 = s.toUtf8();
	return std::string(utf8.constData(), static_cast<size_t>(utf8.size()));
}
} // namespace

namespace RobotSceneKinematics
{

bool applyJointAnglesFromDocument(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, const QVector<double>& anglesRad)
{
	(void)osg;
	if (!doc || !doc->hasRobotSimulationContext())
	{
		RunLogger::warn("RobotSceneKinematics::applyJointAnglesFromDocument invalid document or missing simulation context.");
		return false;
	}

	const int nInst = doc->robotKinematicInstanceCount();
	int offset = 0;
	bool appliedHierarchy = false;
	for (int i = 0; i < nInst; ++i)
	{
		const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(i);
		const int nj = doc->robotRevoluteJointCountForInstance(i);
		if (nj <= 0)
		{
			continue;
		}
		if (offset + nj > anglesRad.size())
		{
			RunLogger::warn("RobotSceneKinematics::applyJointAnglesFromDocument angle vector size does not match joint count.");
			return false;
		}
		const QVector<double> slice = anglesRad.mid(offset, nj);
		offset += nj;
		if (urdfPath.isEmpty())
		{
			continue;
		}

		QHash<QString, osg::Matrixd> newJointMatrices;
		if (!UrdfRobotLoader::computeJointTransformMatrices(urdfPath, slice, newJointMatrices, nullptr))
		{
			RunLogger::warn(qToUtf8Std(QStringLiteral("RobotSceneKinematics: computeJointTransformMatrices failed for URDF '%1'").arg(urdfPath)));
			return false;
		}

		const QString prefix = doc->robotJointKeyPrefixForInstance(i);
		for (auto it = newJointMatrices.constBegin(); it != newJointMatrices.constEnd(); ++it)
		{
			const QString key = prefix + it.key();
			osg::MatrixTransform* jointMT = doc->robotJointMatrixTransform(key);
			if (jointMT)
			{
				jointMT->setMatrix(it.value());
				appliedHierarchy = true;
			}
		}
	}

	if (appliedHierarchy)
	{
		return true;
	}
	// 已按多实例语义遍历过但未写入任何 MatrixTransform：视为失败（避免用错误的全向量再算一遍 FK）
	if (nInst > 0)
	{
		RunLogger::warn("RobotSceneKinematics::applyJointAnglesFromDocument no joint matrix written for multi-instance context.");
		return false;
	}

	// nInst==0（例如仅传统烘焙数据）：回退到旧单 URDF 路径（无前缀键）
	const QString urdfPath = doc->robotUrdfAbsolutePath();
	QHash<QString, osg::Matrixd> newJointMatrices;
	if (urdfPath.isEmpty() ||
		!UrdfRobotLoader::computeJointTransformMatrices(urdfPath, anglesRad, newJointMatrices, nullptr))
	{
		RunLogger::warn(qToUtf8Std(QStringLiteral("RobotSceneKinematics: fallback computeJointTransformMatrices failed for URDF '%1'").arg(urdfPath)));
		return false;
	}

	bool hasJointTransforms = false;
	const QStringList jointNames = doc->robotRevoluteJointNames();
	for (const QString& jointName : jointNames)
	{
		if (doc->robotJointMatrixTransform(jointName))
		{
			hasJointTransforms = true;
			break;
		}
	}

	if (hasJointTransforms)
	{
		for (auto it = newJointMatrices.constBegin(); it != newJointMatrices.constEnd(); ++it)
		{
			osg::MatrixTransform* jointMT = doc->robotJointMatrixTransform(it.key());
			if (jointMT)
			{
				jointMT->setMatrix(it.value());
			}
		}
		return true;
	}

	// 【中文】回退到旧架构（传统烘焙法）
	if (!doc->hasRobotKinematicsBind())
	{
		RunLogger::warn("RobotSceneKinematics: fallback path requires RobotKinematicsBind but it is missing.");
		return false;
	}

	QHash<QString, osg::Matrixd> Tq;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(urdfPath, anglesRad, Tq, nullptr))
	{
		RunLogger::warn(qToUtf8Std(QStringLiteral("RobotSceneKinematics: computeMeshWorldMatrices failed for URDF '%1'").arg(urdfPath)));
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
