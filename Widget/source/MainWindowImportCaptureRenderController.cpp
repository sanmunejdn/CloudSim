#include "MainWindowImportCaptureRenderController.h"

#include "MainWindow.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentPage.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "OsgWidgetCaptureController.h"
#include "PointCloudBackendData.h"
#include "RobotSceneKinematics.h"
#include "RunInfoPage.h"
#include "UrdfRobotLoader.h"

#include <QByteArray>
#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QMessageBox>
#include <QVector>

#include <osg/Matrixd>
#include <osg/Group>
#include <osg/MatrixTransform>

#include <vector>

bool MainWindowImportCaptureRenderController::registerBackendObject(
	MainWindow& mw,
	const QString& filePath,
	const QString& typeName,
	bool isPointCloud,
	bool quietUi)
{
	DocumentPage* doc = mw.currentPage();
	if (!doc)
	{
		return false;
	}
	OsgWidget* osg = doc->osgWidget();
	const QFileInfo fileInfo(filePath);
	const QByteArray nativeEnc = QFile::encodeName(filePath);
	const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));

	auto reportFail = [&](const QString& title, const QString& msg) -> bool {
		if (quietUi)
		{
			if (mw.m_runInfoPage)
			{
				mw.m_runInfoPage->appendWarning(title + QStringLiteral(": ") + msg);
			}
		}
		else
		{
			QMessageBox::warning(&mw, title, msg);
		}
		return false;
	};

	auto finish = [&](const std::shared_ptr<BackendDataBase>& obj) {
		const QString id = QString::fromStdString(obj->id());
		doc->backendSourcePath()[id] = filePath;
		doc->backendSourceType()[id] = typeName;
		doc->backendParentId()[id] = QString();
		if (mw.m_runInfoPage && !quietUi)
		{
			mw.m_runInfoPage->appendInfo(QStringLiteral("Backend object registered: %1").arg(fileInfo.fileName()));
		}
		mw.refreshBackendTree();
		mw.focusBackendInTree(obj);
	};

	auto createImportedFileParent = [&](std::shared_ptr<MeshBackendData>& parentOut) -> bool {
		parentOut = std::make_shared<MeshBackendData>();
		parentOut->setName(fileInfo.fileName().toStdString());
		if (!mw.registerExistingBackendObject(parentOut, filePath, typeName, QString(), false, QString()))
		{
			return false;
		}
		return true;
	};

	if (isPointCloud)
	{
		auto pc = std::make_shared<PointCloudBackendData>();
		pc->setName(fileInfo.fileName().toStdString());
		BackendColor color;
		color.r = 0.65f;
		color.g = 0.82f;
		color.b = 0.95f;
		color.a = 1.0f;
		pc->setColor(color);
		BackendVec3 pose{};
		pc->setPose(pose);
		BackendVec3 rot{};
		pc->setRotation(rot);

		std::string backendErr;
		const bool cgalOk = pc->loadFromFile(nativePath, &backendErr);
		if (!cgalOk)
		{
			const QString ext = fileInfo.suffix().toLower();
			if ((ext == QLatin1String("las") || ext == QLatin1String("laz")) && osg)
			{
				QString importErr;
				if (!osg->importPointCloudFile(filePath, &importErr))
				{
					return reportFail(QStringLiteral("Point cloud"),
						importErr.isEmpty() ? QStringLiteral("Failed to load LAS/LAZ.") : importErr);
				}
				if (!doc->backend().registerData(pc))
				{
					osg->clearStagingGeometry();
					return reportFail(QStringLiteral("Backend Register"),
						QStringLiteral("Failed to register backend object."));
				}
				QString capErr;
				if (!osg->captureImportedPointCloudBackend(*pc, &capErr) || pc->pointPositionsXyz().empty())
				{
					doc->backend().unregisterData(pc->id());
					osg->clearStagingGeometry();
					mw.refreshBackendTree();
					return reportFail(QStringLiteral("Point cloud"),
						QStringLiteral("Could not copy LAS/LAZ into backend.\n%1").arg(capErr));
				}
				QString sceneErr;
				if (!osg->loadPointCloudFromBackendData(*pc, &sceneErr, true) && mw.m_runInfoPage)
				{
					mw.m_runInfoPage->appendWarning(QStringLiteral("Point cloud display: %1").arg(sceneErr));
				}
				finish(pc);
				return true;
			}
			return reportFail(QStringLiteral("Point cloud"),
				QString::fromStdString(backendErr.empty() ? std::string("Failed to load point cloud.") : backendErr));
		}

		if (!doc->backend().registerData(pc))
		{
			return reportFail(QStringLiteral("Backend Register"),
				QStringLiteral("Failed to register backend object."));
		}
		if (osg)
		{
			QString sceneErr;
			if (!osg->loadPointCloudFromBackendData(*pc, &sceneErr, true) && mw.m_runInfoPage)
			{
				mw.m_runInfoPage->appendWarning(QStringLiteral("Point cloud display: %1").arg(sceneErr));
			}
		}
		finish(pc);
		return true;
	}

	auto mesh = std::make_shared<MeshBackendData>();
	mesh->setName(fileInfo.fileName().toStdString());
	const QString ext = fileInfo.suffix().toLower();
	if (ext == QLatin1String("dxf"))
	{
		std::vector<MeshHierarchyPart> dxfParts;
		std::string dxfErr;
		const bool dxfHierOk = MeshBackendData::loadDxfHierarchyFromFile(nativePath, dxfParts, &dxfErr);
		if (mw.m_runInfoPage)
		{
			mw.m_runInfoPage->appendInfo(
				QStringLiteral("DXF hierarchy parse: ok=%1, parts=%2, err=%3")
				.arg(dxfHierOk ? QStringLiteral("true") : QStringLiteral("false"))
				.arg(static_cast<int>(dxfParts.size()))
				.arg(QString::fromStdString(dxfErr)));
		}
		if (dxfHierOk && !dxfParts.empty())
		{
			std::shared_ptr<MeshBackendData> importParent;
			if (!createImportedFileParent(importParent))
			{
				return reportFail(QStringLiteral("Backend Register"),
					QStringLiteral("Failed to register DXF import parent object."));
			}
			const QString importParentId = QString::fromStdString(importParent->id());
			QHash<QString, QString> pathToBackendId;
			std::shared_ptr<BackendDataBase> firstRegistered;
			std::shared_ptr<MeshBackendData> lastLoadedMesh;
			for (const MeshHierarchyPart& p : dxfParts)
			{
				if (p.triangleSoup.empty())
				{
					continue;
				}
				auto partMesh = std::make_shared<MeshBackendData>();
				partMesh->setTriangleSoup(p.triangleSoup);
				const QString displayName = p.displayName.empty() ? fileInfo.completeBaseName() : QString::fromStdString(p.displayName);
				partMesh->setName(displayName.toStdString());
				const QString partPath = QString::fromStdString(p.partPath);
				const QString parentPartPath = QString::fromStdString(p.parentPartPath);
				const QString parentId = pathToBackendId.contains(parentPartPath) ? pathToBackendId.value(parentPartPath) : importParentId;
				if (!mw.registerExistingBackendObject(partMesh, filePath, typeName, QString(), false, parentId))
				{
					return reportFail(QStringLiteral("Backend Register"),
						QStringLiteral("Failed to register DXF hierarchical backend object."));
				}
				const QString selfId = QString::fromStdString(partMesh->id());
				pathToBackendId[partPath] = selfId;
				if (!firstRegistered)
				{
					firstRegistered = partMesh;
				}
				if (osg)
				{
					QString sceneErr;
					if (!osg->loadMeshFromBackendData(*partMesh, &sceneErr, false) && mw.m_runInfoPage)
					{
						mw.m_runInfoPage->appendWarning(
							QStringLiteral("Mesh display failed for DXF part '%1': %2")
							.arg(displayName, sceneErr));
					}
					else
					{
						lastLoadedMesh = partMesh;
					}
				}
			}
			if (firstRegistered)
			{
				if (osg && lastLoadedMesh)
				{
					QString homeErr;
					(void)osg->loadMeshFromBackendData(*lastLoadedMesh, &homeErr, true);
				}
				mw.refreshBackendTree();
				mw.focusBackendInTree(importParent);
				return true;
			}
		}
	}
	if (ext == QLatin1String("step") || ext == QLatin1String("stp"))
	{
		std::vector<MeshHierarchyPart> stepParts;
		std::string stepErr;
		if (MeshBackendData::loadStepHierarchyFromFile(nativePath, stepParts, &stepErr) && stepParts.size() > 1U)
		{
			std::shared_ptr<MeshBackendData> importParent;
			if (!createImportedFileParent(importParent))
			{
				return reportFail(QStringLiteral("Backend Register"),
					QStringLiteral("Failed to register STEP import parent object."));
			}
			const QString importParentId = QString::fromStdString(importParent->id());
			QHash<QString, QString> pathToBackendId;
			std::shared_ptr<BackendDataBase> firstRegistered;
			std::shared_ptr<MeshBackendData> lastLoadedMesh;
			for (const MeshHierarchyPart& p : stepParts)
			{
				if (p.triangleSoup.empty())
				{
					continue;
				}
				auto partMesh = std::make_shared<MeshBackendData>();
				partMesh->setTriangleSoup(p.triangleSoup);
				const QString displayName = p.displayName.empty() ? fileInfo.completeBaseName() : QString::fromStdString(p.displayName);
				partMesh->setName(displayName.toStdString());
				const QString partPath = QString::fromStdString(p.partPath);
				const QString parentPartPath = QString::fromStdString(p.parentPartPath);
				const QString parentId = pathToBackendId.contains(parentPartPath) ? pathToBackendId.value(parentPartPath) : importParentId;
				if (!mw.registerExistingBackendObject(partMesh, filePath, typeName, QString(), false, parentId))
				{
					return reportFail(QStringLiteral("Backend Register"),
						QStringLiteral("Failed to register STEP hierarchical backend object."));
				}
				const QString selfId = QString::fromStdString(partMesh->id());
				pathToBackendId[partPath] = selfId;
				if (!firstRegistered)
				{
					firstRegistered = partMesh;
				}
				if (osg)
				{
					QString sceneErr;
					if (!osg->loadMeshFromBackendData(*partMesh, &sceneErr, false) && mw.m_runInfoPage)
					{
						mw.m_runInfoPage->appendWarning(QStringLiteral("Mesh display: %1").arg(sceneErr));
					}
					else
					{
						lastLoadedMesh = partMesh;
					}
				}
			}
			if (firstRegistered)
			{
				if (osg && lastLoadedMesh)
				{
					QString homeErr;
					(void)osg->loadMeshFromBackendData(*lastLoadedMesh, &homeErr, true);
				}
				mw.refreshBackendTree();
				mw.focusBackendInTree(importParent);
				return true;
			}
		}
	}
	std::string backendErr;
	const bool cgalOk = mesh->loadFromFile(nativePath, &backendErr);
	if (!cgalOk)
	{
		static const QStringList kOsgOnly{
			QStringLiteral("dae"), QStringLiteral("3ds"), QStringLiteral("fbx")
			// STEP/IGES 走 Data 层的 OCCT/CGAL 导入；不要在这里走 OSG fallback。
			// 如果你后续实现了 IGES 支持，也同样应放到 Data 层。
		};
		if (!kOsgOnly.contains(ext) || !osg)
		{
			return reportFail(QStringLiteral("Model"),
				QString::fromStdString(backendErr.empty() ? std::string("Failed to load mesh.") : backendErr));
		}
		QString importErr;
		if (!osg->importModelFile(filePath, &importErr))
		{
			return reportFail(QStringLiteral("Import Model"),
				importErr.isEmpty() ? QStringLiteral("Failed to import model.") : importErr);
		}
		std::vector<MeshCapturedPart> parts;
		QString hierarchyErr;
		const bool hasHierarchy = osg->captureImportedMeshBackendHierarchy(parts, &hierarchyErr);
		if (hasHierarchy && parts.size() > 1U)
		{
			std::shared_ptr<MeshBackendData> importParent;
			if (!createImportedFileParent(importParent))
			{
				return reportFail(QStringLiteral("Backend Register"),
					QStringLiteral("Failed to register model import parent object."));
			}
			const QString importParentId = QString::fromStdString(importParent->id());
			QHash<QString, QString> pathToBackendId;
			std::shared_ptr<BackendDataBase> firstRegistered;
			std::shared_ptr<MeshBackendData> lastLoadedMesh;
			for (const MeshCapturedPart& p : parts)
			{
				if (p.triangleSoup.empty())
				{
					continue;
				}
				auto partMesh = std::make_shared<MeshBackendData>();
				partMesh->setTriangleSoup(p.triangleSoup);
				const QString displayName = p.displayName.isEmpty() ? fileInfo.completeBaseName() : p.displayName;
				partMesh->setName(displayName.toStdString());
				const QString parentId = pathToBackendId.contains(p.parentPartPath) ? pathToBackendId.value(p.parentPartPath) : importParentId;
				if (!mw.registerExistingBackendObject(partMesh, filePath, typeName, QString(), false, parentId))
				{
					return reportFail(QStringLiteral("Backend Register"),
						QStringLiteral("Failed to register hierarchical backend object."));
				}
				const QString selfId = QString::fromStdString(partMesh->id());
				pathToBackendId[p.partPath] = selfId;
				if (!firstRegistered)
				{
					firstRegistered = partMesh;
				}
				if (osg)
				{
					QString sceneErr;
					if (!osg->loadMeshFromBackendData(*partMesh, &sceneErr, false) && mw.m_runInfoPage)
					{
						mw.m_runInfoPage->appendWarning(QStringLiteral("Mesh display: %1").arg(sceneErr));
					}
					else
					{
						lastLoadedMesh = partMesh;
					}
				}
			}
			osg->clearStagingGeometry();
			if (firstRegistered)
			{
				if (osg && lastLoadedMesh)
				{
					QString homeErr;
					(void)osg->loadMeshFromBackendData(*lastLoadedMesh, &homeErr, true);
				}
				mw.refreshBackendTree();
				mw.focusBackendInTree(importParent);
				return true;
			}
		}
		if (!doc->backend().registerData(mesh))
		{
			osg->clearStagingGeometry();
			return reportFail(QStringLiteral("Backend Register"),
				QStringLiteral("Failed to register backend object."));
		}
		QString capErr;
		if (!osg->captureImportedMeshBackend(*mesh, &capErr) || mesh->triangleSoup().empty())
		{
			doc->backend().unregisterData(mesh->id());
			osg->clearStagingGeometry();
			mw.refreshBackendTree();
			return reportFail(QStringLiteral("Model"),
				QStringLiteral("Could not copy mesh into backend.\n%1").arg(capErr));
		}
		QString sceneErr;
		if (!osg->loadMeshFromBackendData(*mesh, &sceneErr, true) && mw.m_runInfoPage)
		{
			mw.m_runInfoPage->appendWarning(QStringLiteral("Mesh display: %1").arg(sceneErr));
		}
		finish(mesh);
		return true;
	}

	if (!doc->backend().registerData(mesh))
	{
		return reportFail(QStringLiteral("Backend Register"),
			QStringLiteral("Failed to register backend object."));
	}
	if (osg)
	{
		QString sceneErr;
		if (!osg->loadMeshFromBackendData(*mesh, &sceneErr, true) && mw.m_runInfoPage)
		{
			mw.m_runInfoPage->appendWarning(QStringLiteral("Mesh display: %1").arg(sceneErr));
		}
	}
	finish(mesh);
	return true;
}

bool MainWindowImportCaptureRenderController::registerUrdfRobot(MainWindow& mw, const QString& urdfFilePath, bool quietUi)
{
	DocumentPage* doc = mw.currentPage();
	if (!doc)
	{
		return false;
	}
	OsgWidget* osg = doc->osgWidget();
	const QFileInfo fileInfo(urdfFilePath);

	auto reportFail = [&](const QString& title, const QString& msg) -> bool {
		if (quietUi)
		{
			if (mw.m_runInfoPage)
			{
				mw.m_runInfoPage->appendWarning(title + QStringLiteral(": ") + msg);
			}
		}
		else
		{
			QMessageBox::warning(&mw, title, msg);
		}
		return false;
	};

	// Remove any prior per-link robot backends from this document.
	const QStringList prevLinkIds = doc->robotLinkBackendIds();
	for (const QString& id : prevLinkIds)
	{
		if (osg)
		{
			osg->removeBackendObjectVisual(id.toStdString());
		}
		doc->backend().unregisterData(id.toStdString());
		doc->backendSourcePath().remove(id);
		doc->backendSourceType().remove(id);
		doc->backendParentId().remove(id);
	}
	doc->clearRobotSimulationContext();

	QString urdfErr;
	QString rootLink;
	QHash<QString, QString> linkMeshes;
	if (!UrdfRobotLoader::enumerateLinkVisualMeshes(fileInfo.absoluteFilePath(), rootLink, linkMeshes, &urdfErr))
	{
		return reportFail(QStringLiteral("URDF"),
			urdfErr.isEmpty() ? QStringLiteral("Failed to enumerate link meshes from URDF.") : urdfErr);
	}
	if (linkMeshes.isEmpty())
	{
		return reportFail(QStringLiteral("URDF"), QStringLiteral("No mesh visuals found in URDF (need <visual><geometry><mesh>)."));
	}

	QStringList revoluteJointNames;
	QVector<double> jointLowerRad;
	QVector<double> jointUpperRad;
	if (!UrdfRobotLoader::loadRevoluteJointMeta(
			fileInfo.absoluteFilePath(), revoluteJointNames, jointLowerRad, jointUpperRad, &urdfErr))
	{
		if (mw.m_runInfoPage && !urdfErr.isEmpty())
		{
			mw.m_runInfoPage->appendWarning(QStringLiteral("URDF joint list: %1").arg(urdfErr));
		}
	}

	const QString sceneKey =
		QStringLiteral("RobotURDF_%1_%2").arg(fileInfo.completeBaseName()).arg(QDateTime::currentMSecsSinceEpoch());

	QVector<double> q0(revoluteJointNames.size(), 0.0);

	QHash<QString, QString> linkToBackend;
	QHash<QString, osg::Matrixd> fkT0;
	QHash<QString, osg::Matrixd> outerBind;

	for (auto it = linkMeshes.constBegin(); it != linkMeshes.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& absMesh = it.value();
		const QString bid = sceneKey + QStringLiteral("_link_") + linkName;

		auto mesh = std::make_shared<MeshBackendData>();
		mesh->setId(bid.toStdString());
		mesh->setName(linkName.toStdString());
		std::string loadErr;
		if (!mesh->loadFromFile(absMesh.toStdString(), &loadErr))
		{
			return reportFail(QStringLiteral("URDF"),
				QStringLiteral("Failed to load mesh for link '%1': %2")
					.arg(linkName, QString::fromStdString(loadErr)));
		}
		double fileToLink16[16];
		if (!UrdfRobotLoader::linkMeshFileToLinkColumnMajor16(fileInfo.absoluteFilePath(), linkName, fileToLink16, &urdfErr))
		{
			return reportFail(QStringLiteral("URDF"),
				urdfErr.isEmpty()
					? QStringLiteral("Could not resolve <visual> frame for link '%1'.").arg(linkName)
					: urdfErr);
		}
		mesh->transformVerticesColumnMajorHomogeneous4x4(fileToLink16);
		if (!doc->backend().registerData(mesh))
		{
			return reportFail(QStringLiteral("URDF"), QStringLiteral("Backend id collision for link '%1'.").arg(linkName));
		}
		doc->backendSourcePath()[bid] = fileInfo.absoluteFilePath();
		doc->backendSourceType()[bid] = QStringLiteral("URDF");

		if (osg)
		{
			QString sceneErr;
			if (!osg->loadMeshFromBackendData(*mesh, &sceneErr, true, true, true, true))
			{
				doc->backend().unregisterData(mesh->id());
				return reportFail(QStringLiteral("URDF"),
					QStringLiteral("OSG load failed for link '%1': %2").arg(linkName, sceneErr));
			}
		}

		linkToBackend.insert(linkName, bid);
	}

	QHash<QString, osg::Matrixd> Tq;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(fileInfo.absoluteFilePath(), q0, Tq, &urdfErr, true))
	{
		return reportFail(QStringLiteral("URDF"),
			urdfErr.isEmpty() ? QStringLiteral("Forward kinematics (bind pose) failed.") : urdfErr);
	}
	for (auto it = linkToBackend.constBegin(); it != linkToBackend.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& bid = it.value();
		if (!Tq.contains(linkName))
		{
			continue;
		}
		const osg::Matrixd& meshWorld0 = Tq[linkName];
		fkT0.insert(linkName, meshWorld0);
		outerBind.insert(bid, meshWorld0);
	}

	// Kinematic tree in BackendDataManager + UI mirror so the backend tree shows one assembled robot.
	QHash<QString, QString> urdfChildToParent;
	if (!UrdfRobotLoader::loadLinkChildToParentMap(fileInfo.absoluteFilePath(), urdfChildToParent, &urdfErr))
	{
		if (mw.m_runInfoPage && !urdfErr.isEmpty())
		{
			mw.m_runInfoPage->appendWarning(QStringLiteral("URDF link parent map: %1").arg(urdfErr));
		}
	}
	else
	{
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
			if (parentLink.isEmpty())
			{
				continue;
			}
			const QString parentBid = linkToBackend.value(parentLink);
			const QString& childBid = it.value();
			if (parentBid.isEmpty() || parentBid == childBid)
			{
				continue;
			}
			if (!doc->backend().attachChild(parentBid.toStdString(), childBid.toStdString()) && mw.m_runInfoPage)
			{
				mw.m_runInfoPage->appendWarning(
					QStringLiteral("URDF: could not attach link backend %1 under %2").arg(childBid, parentBid));
			}
		}
	}
	{
		QMap<QString, QString>& parentMirror = doc->backendParentId();
		for (const auto& d : doc->backend().listData())
		{
			if (!d)
			{
				continue;
			}
			const QString id = QString::fromStdString(d->id());
			const std::vector<std::string> ps = doc->backend().parentsOf(d->id());
			parentMirror[id] = ps.empty() ? QString() : QString::fromStdString(ps.front());
		}
		if (osg)
		{
			for (const auto& d : doc->backend().listData())
			{
				if (!d)
				{
					continue;
				}
				const std::vector<std::string> ps = doc->backend().parentsOf(d->id());
				const std::string pp = ps.empty() ? std::string() : ps.front();
				osg->setBackendParent(d->id(), pp);
			}
		}
	}

	QString sceneRootBackendId = linkToBackend.value(rootLink);
	if (sceneRootBackendId.isEmpty() && !linkToBackend.isEmpty())
	{
		sceneRootBackendId = linkToBackend.begin().value();
	}

	doc->appendHierarchicalRobotSimulationContext(
		fileInfo.absoluteFilePath(),
		revoluteJointNames,
		jointLowerRad,
		jointUpperRad,
		QHash<QString, osg::MatrixTransform*>(),
		sceneRootBackendId,
		sceneKey);

	doc->setRobotPerLinkKinematicsBinding(sceneKey + QStringLiteral("_ctx"), linkToBackend, fkT0, outerBind, true);

	if (osg && !RobotSceneKinematics::applyJointAnglesFromDocument(doc, osg, q0))
	{
		if (mw.m_runInfoPage)
		{
			mw.m_runInfoPage->appendWarning(QStringLiteral("URDF: initial kinematics sync failed."));
		}
	}

	if (osg)
	{
		osg->clearStagingGeometry();
		if (!sceneRootBackendId.isEmpty())
		{
			osg->focusCameraOnBackend(sceneRootBackendId.toStdString());
		}
	}

	mw.refreshSimulationJointListFromCurrentDoc();

	if (mw.m_runInfoPage && !quietUi)
	{
		mw.m_runInfoPage->appendInfo(
			QStringLiteral("URDF robot added (per-link backends): %1, Links: %2, Joints: %3, Key=%4")
				.arg(fileInfo.fileName())
				.arg(linkMeshes.size())
				.arg(revoluteJointNames.size())
				.arg(sceneKey));
		const QByteArray kd = qgetenv("ROBOT_KINEMATICS_DEBUG");
		if (kd.isEmpty() || kd == QByteArray("0"))
		{
			mw.m_runInfoPage->appendInfo(
				QStringLiteral("FK/OSG 调试：设置环境变量 ROBOT_KINEMATICS_DEBUG=1（或调试启动参数 --robot-kinematics-debug 1）"
							   " 后需重启程序；再导入 URDF 或拖动关节，本页与 exe 旁 logs\\PointCloudProcess.log 会出现 [RobotKinematicsDBG] 矩阵。"));
		}
	}

	mw.refreshBackendTree();
	return true;
}

