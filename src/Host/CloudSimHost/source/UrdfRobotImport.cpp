#include "UrdfRobotImport.h"

#include "IRobotUrdfImportContext.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "RobotCoordinateFrames.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"

#include <QDateTime>
#include <QFileInfo>

#include <cmath>
#include <memory>

#include <osg/Matrixd>
#include <osg/MatrixTransform>

namespace cloudsim::host {

namespace {

QString makeUniqueBackendId(BackendDataManager& mgr, const QString& baseId)
{
	if (!mgr.contains(baseId.toStdString()))
	{
		return baseId;
	}
	for (int n = 2; n < 10000; ++n)
	{
		const QString candidate = baseId + QStringLiteral("_") + QString::number(n);
		if (!mgr.contains(candidate.toStdString()))
		{
			return candidate;
		}
	}
	return baseId + QStringLiteral("_") + QString::number(QDateTime::currentMSecsSinceEpoch());
}

core::RobotRegistrationDto fail(const QString& error)
{
	return {false, error, {}, {}, 0, 0, {}};
}

} // namespace

core::RobotRegistrationDto importUrdfRobot(IRobotUrdfImportContext& ctx, const QString& urdfFilePath,
	const core::ImportOptionsDto& options)
{
	(void)options.quietUi;

	QVector<QString> warnings;
	const auto warn = [&warnings](const QString& msg) { warnings.append(msg); };

	const QFileInfo fileInfo(urdfFilePath);
	OsgWidget* osg = ctx.urdfImportOsgWidget();
	BackendDataManager& backend = ctx.urdfImportBackend();
	IRobotSimulationDocument* robotDoc = ctx.urdfImportRobotSimulationDocument();
	if (!robotDoc)
	{
		return fail(QStringLiteral("URDF import: no robot simulation document"));
	}

	QString urdfErr;
	QString rootLink;
	QHash<QString, QString> linkMeshes;
	QHash<QString, BackendColor> linkMaterialColors;
	if (!UrdfRobotLoader::enumerateLinkVisualMeshes(fileInfo.absoluteFilePath(), rootLink, linkMeshes, &urdfErr))
	{
		return fail(urdfErr.isEmpty() ? QStringLiteral("Failed to enumerate link meshes from URDF.") : urdfErr);
	}
	if (linkMeshes.isEmpty())
	{
		return fail(QStringLiteral("No mesh visuals found in URDF (need <visual><geometry><mesh>)."));
	}
	(void)UrdfRobotLoader::loadLinkVisualMaterialColors(fileInfo.absoluteFilePath(), linkMaterialColors, nullptr);

	QStringList revoluteJointNames;
	QVector<double> jointLowerRad;
	QVector<double> jointUpperRad;
	if (!UrdfRobotLoader::loadRevoluteJointMeta(
			fileInfo.absoluteFilePath(), revoluteJointNames, jointLowerRad, jointUpperRad, &urdfErr))
	{
		if (!urdfErr.isEmpty())
		{
			warn(QStringLiteral("URDF joint list: %1").arg(urdfErr));
		}
	}

	const QString robotDisplayName = fileInfo.completeBaseName();
	const QString robotRootId = makeUniqueBackendId(backend, QStringLiteral("RobotURDF_%1").arg(robotDisplayName));

	auto robotRoot = std::make_shared<MeshBackendData>();
	robotRoot->setId(robotRootId.toStdString());
	robotRoot->setName(robotDisplayName.toStdString());
	if (!backend.registerData(robotRoot))
	{
		return fail(QStringLiteral("Failed to register robot root backend."));
	}
	ctx.urdfImportBackendSourcePath()[robotRootId] = fileInfo.absoluteFilePath();
	ctx.urdfImportBackendSourceType()[robotRootId] = QStringLiteral("URDF");

	QVector<double> q0(revoluteJointNames.size(), 0.0);

	QHash<QString, QString> linkToBackend;
	QHash<QString, osg::Matrixd> fkT0;
	QHash<QString, osg::Matrixd> outerBind;

	for (auto it = linkMeshes.constBegin(); it != linkMeshes.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& absMesh = it.value();
		const QString bid = robotRootId + QStringLiteral("_") + linkName;

		auto mesh = std::make_shared<MeshBackendData>();
		mesh->setId(bid.toStdString());
		mesh->setName(linkName.toStdString());
		std::string loadErr;
		if (!mesh->loadFromFile(absMesh.toStdString(), &loadErr))
		{
			return fail(QStringLiteral("Failed to load mesh for link '%1': %2")
						   .arg(linkName, QString::fromStdString(loadErr)));
		}
		if (linkMaterialColors.contains(linkName))
		{
			mesh->setColor(linkMaterialColors.value(linkName));
		}
		double fileToLink16[16];
		if (!UrdfRobotLoader::linkMeshFileToLinkColumnMajor16(fileInfo.absoluteFilePath(), linkName, fileToLink16, &urdfErr))
		{
			return fail(urdfErr.isEmpty()
							? QStringLiteral("Could not resolve <visual> frame for link '%1'.").arg(linkName)
							: urdfErr);
		}
		// 顶点烘焙到 link 坐标系，与 meshInLinkFrame FK 一致
		mesh->transformVerticesColumnMajorHomogeneous4x4(fileToLink16);
		mesh->setTransformPivotAtOrigin(true);
		if (!backend.registerData(mesh))
		{
			return fail(QStringLiteral("Backend id collision for link '%1'.").arg(linkName));
		}
		ctx.urdfImportBackendSourcePath()[bid] = fileInfo.absoluteFilePath();
		ctx.urdfImportBackendSourceType()[bid] = QStringLiteral("URDF");

		if (osg)
		{
			QString sceneErr;
			if (!osg->loadMeshFromBackendData(*mesh, &sceneErr, true, true, true, true))
			{
				backend.unregisterData(mesh->id());
				return fail(QStringLiteral("OSG load failed for link '%1': %2").arg(linkName, sceneErr));
			}
		}

		linkToBackend.insert(linkName, bid);
	}

	QHash<QString, osg::Matrixd> Tq;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(fileInfo.absoluteFilePath(), q0, Tq, &urdfErr, true))
	{
		return fail(urdfErr.isEmpty() ? QStringLiteral("Forward kinematics (bind pose) failed.") : urdfErr);
	}
	for (auto it = linkToBackend.constBegin(); it != linkToBackend.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& bid = it.value();
		if (!Tq.contains(linkName))
		{
			continue;
		}
		fkT0.insert(linkName, Tq[linkName]);
		(void)bid;
	}

	QHash<QString, QString> urdfChildToParent;
	if (!UrdfRobotLoader::loadLinkChildToParentMap(fileInfo.absoluteFilePath(), urdfChildToParent, &urdfErr))
	{
		if (!urdfErr.isEmpty())
		{
			warn(QStringLiteral("URDF link parent map: %1").arg(urdfErr));
		}
	}
	else
	{
		// 跳过无 mesh 的 URDF link，挂到最近有 backend 的祖先
		const auto nearestMeshedAncestor = [&](const QString& linkName) -> QString {
			QString p = urdfChildToParent.value(linkName);
			while (!p.isEmpty() && !linkToBackend.contains(p))
			{
				p = urdfChildToParent.value(p);
			}
			return p;
		};
		for (auto it = linkToBackend.constBegin(); it != linkToBackend.constEnd(); ++it)
		{
			const QString& linkName = it.key();
			const QString parentLink = nearestMeshedAncestor(linkName);
			const QString parentBid = parentLink.isEmpty() ? robotRootId : linkToBackend.value(parentLink);
			const QString& childBid = it.value();
			if (parentBid.isEmpty() || parentBid == childBid)
			{
				continue;
			}
			if (!backend.attachChild(parentBid.toStdString(), childBid.toStdString()))
			{
				warn(QStringLiteral("URDF: could not attach link backend %1 under %2").arg(childBid, parentBid));
			}
		}
	}
	{
		QMap<QString, QString>& parentMirror = ctx.urdfImportBackendParentId();
		for (const auto& d : backend.listData())
		{
			if (!d)
			{
				continue;
			}
			const QString id = QString::fromStdString(d->id());
			const std::vector<std::string> ps = backend.parentsOf(d->id());
			parentMirror[id] = ps.empty() ? QString() : QString::fromStdString(ps.front());
		}
		if (osg)
		{
			QHash<QString, QString> backendIdToLink;
			for (auto lit = linkToBackend.constBegin(); lit != linkToBackend.constEnd(); ++lit)
			{
				backendIdToLink.insert(lit.value(), lit.key());
			}
			const std::vector<std::string> topoIds = backend.topoOrder();
			for (const std::string& bidStd : topoIds)
			{
				const QString bid = QString::fromStdString(bidStd);
				if (!backendIdToLink.contains(bid))
				{
					continue;
				}
				const std::vector<std::string> ps = backend.parentsOf(bidStd);
				const std::string pp = ps.empty() ? std::string() : ps.front();
				osg->setBackendParent(bidStd, pp);
			}
		}
	}

	if (osg)
	{
		QHash<QString, QString> backendIdToLink;
		for (auto it = linkToBackend.constBegin(); it != linkToBackend.constEnd(); ++it)
		{
			backendIdToLink.insert(it.value(), it.key());
		}
		const std::vector<std::string> topoIds = backend.topoOrder();
		for (const std::string& bidStd : topoIds)
		{
			const QString bid = QString::fromStdString(bidStd);
			const QString linkName = backendIdToLink.value(bid);
			if (linkName.isEmpty() || !Tq.contains(linkName))
			{
				continue;
			}
			osg->setBackendRootWorldMatrixFromWorld(bidStd, Tq[linkName]);
		}
	}

	outerBind.clear();
	auto maxMatAbsDiff = [](const osg::Matrixd& a, const osg::Matrixd& b) -> double {
		double m = 0.0;
		for (int r = 0; r < 4; ++r)
		{
			for (int c = 0; c < 4; ++c)
			{
				const double d = std::abs(static_cast<double>(a(r, c)) - static_cast<double>(b(r, c)));
				if (d > m)
				{
					m = d;
				}
			}
		}
		return m;
	};
	for (auto it = linkToBackend.constBegin(); it != linkToBackend.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& bid = it.value();
		const osg::Matrixd* fkMeshWorld = Tq.contains(linkName) ? &Tq[linkName] : nullptr;
		if (osg)
		{
			osg::Matrixd worldAtBind;
			if (osg->getBackendRootWorldMatrix(bid.toStdString(), worldAtBind))
			{
				// OSG 与 FK 偏差大时以外部 world 为准，避免 bind 漂移
				if (fkMeshWorld && maxMatAbsDiff(worldAtBind, *fkMeshWorld) > 0.5)
				{
					warn(QStringLiteral("URDF bind: OSG outer world for '%1' diverged from FK mesh world; using FK.")
							 .arg(linkName));
					outerBind.insert(bid, *fkMeshWorld);
				}
				else
				{
					outerBind.insert(bid, worldAtBind);
				}
				continue;
			}
		}
		if (fkMeshWorld)
		{
			outerBind.insert(bid, *fkMeshWorld);
		}
	}

	QString focusBackendId = linkToBackend.value(rootLink);
	if (focusBackendId.isEmpty() && !linkToBackend.isEmpty())
	{
		focusBackendId = linkToBackend.begin().value();
	}

	ctx.appendHierarchicalRobotSimulationContext(fileInfo.absoluteFilePath(), revoluteJointNames, jointLowerRad,
		jointUpperRad, QHash<QString, osg::MatrixTransform*>(), robotRootId, robotRootId);

	{
		QString defaultFlangeLink;
		QStringList revoluteChildLinks;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(fileInfo.absoluteFilePath(), revoluteChildLinks, nullptr);
		if (!revoluteChildLinks.isEmpty())
		{
			defaultFlangeLink = revoluteChildLinks.back();
		}
		const int instIdx = ctx.robotKinematicInstanceCount() - 1;
		if (instIdx >= 0)
		{
			ctx.robotCoordinateFramesForInstance(instIdx) =
				RobotCoordinate::makeDefaultFrameSet(defaultFlangeLink.toStdString());
		}
	}

	ctx.setRobotPerLinkKinematicsBinding(robotRootId + QStringLiteral("_ctx"), linkToBackend, fkT0, outerBind, true);

	if (!RobotSceneKinematics::applyJointAnglesFromDocument(robotDoc, ctx.urdfImportScenePoseSink(), q0))
	{
		warn(QStringLiteral("URDF: initial kinematics sync failed."));
	}

	if (osg)
	{
		osg->clearStagingGeometry();
		if (!focusBackendId.isEmpty())
		{
			osg->focusCameraOnBackend(focusBackendId.toStdString());
		}
	}

	core::RobotRegistrationDto out;
	out.ok = true;
	out.sceneRootBackendId = robotRootId;
	out.linkCount = linkMeshes.size();
	out.jointCount = revoluteJointNames.size();
	out.sourceDisplayName = fileInfo.fileName();
	out.warnings = std::move(warnings);
	return out;
}

} // namespace cloudsim::host
