#include "MainWindowImportCaptureRenderController.h"

#include "MainWindow.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentPage.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "OsgWidgetCaptureController.h"
#include "PointCloudBackendData.h"
#include "RunInfoPage.h"
#include "UrdfRobotLoader.h"

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

	// 【中文】新架构：使用动态层级法构建 URDF 场景图
	// 1. 构建层级化场景图（三层分离模型）
	QHash<QString, osg::Node*> linkToGeometry;
	QHash<QString, osg::Group*> linkToContainer;
	QHash<QString, osg::MatrixTransform*> jointTransforms;
	QString urdfErr;

	osg::Group* robotAssembly = UrdfRobotLoader::buildHierarchicalRobotScene(
		urdfFilePath, linkToGeometry, linkToContainer, jointTransforms, &urdfErr);

	if (!robotAssembly)
	{
		return reportFail(QStringLiteral("URDF"),
			urdfErr.isEmpty() ? QStringLiteral("Failed to build hierarchical robot scene.") : urdfErr);
	}

	// 无 <visual> mesh 的连杆不会进入 linkToGeometry，但仍会有容器与场景图；不得以「无几何」为由拒绝导入。
	if (linkToContainer.isEmpty())
	{
		return reportFail(QStringLiteral("URDF"), QStringLiteral("No link containers built from URDF."));
	}

	// 【中文】2. 将机器人场景添加到 OSG
	QString robotBackendId;
	if (osg)
	{
		robotBackendId = osg->addHierarchicalRobotScene(robotAssembly, fileInfo.fileName());
		if (robotBackendId.isEmpty())
		{
			return reportFail(QStringLiteral("URDF"), QStringLiteral("Failed to add robot scene to OSG."));
		}
		osg->clearStagingGeometry();

		// Units 树与属性面板依赖 backend().listData()；无三角形 soup 的占位 Model 与 OSG 分支 id 一致
		auto robotBackend = std::make_shared<MeshBackendData>();
		robotBackend->setId(robotBackendId.toStdString());
		robotBackend->setName((fileInfo.completeBaseName() + QStringLiteral(" (URDF)")).toStdString());
		if (!doc->backend().registerData(robotBackend))
		{
			osg->removeBackendObjectVisual(robotBackendId.toStdString());
			return reportFail(QStringLiteral("URDF"),
				QStringLiteral("Failed to register robot in backend list (id collision)."));
		}
		doc->backendSourcePath()[robotBackendId] = fileInfo.absoluteFilePath();
		doc->backendSourceType()[robotBackendId] = QStringLiteral("URDF");
		doc->backendParentId()[robotBackendId] = QString();
	}
	if (robotBackendId.isEmpty())
	{
		// 无 OSG 时仍需唯一 backendId，避免多实例关节前缀 "::" 冲突
		robotBackendId =
			QStringLiteral("RobotScene_headless_%1").arg(reinterpret_cast<quintptr>(robotAssembly), 0, 16);
	}

	// 【中文】3. 获取关节信息并设置仿真上下文
	QStringList revoluteJointNames;
	QVector<double> jointLowerRad;
	QVector<double> jointUpperRad;
	QString jointListErr;
	if (!UrdfRobotLoader::loadRevoluteJointMeta(
			fileInfo.absoluteFilePath(), revoluteJointNames, jointLowerRad, jointUpperRad, &jointListErr))
	{
		if (mw.m_runInfoPage && !jointListErr.isEmpty())
		{
			mw.m_runInfoPage->appendWarning(QStringLiteral("URDF joint list: %1").arg(jointListErr));
		}
	}

	// 【中文】4. 追加层级化仿真上下文（多台机器人并存；关节键 = 场景 backendId + "::" + URDF 关节名）
	const QString jkPrefix = robotBackendId + QStringLiteral("::");
	QHash<QString, osg::MatrixTransform*> prefixedJoints;
	prefixedJoints.reserve(jointTransforms.size());
	for (auto it = jointTransforms.constBegin(); it != jointTransforms.constEnd(); ++it)
	{
		prefixedJoints.insert(jkPrefix + it.key(), it.value());
	}
	doc->appendHierarchicalRobotSimulationContext(
		fileInfo.absoluteFilePath(),
		revoluteJointNames,
		jointLowerRad,
		jointUpperRad,
		prefixedJoints,
		robotBackendId);

	// 【中文】同步关节滑块列表
	mw.refreshSimulationJointListFromCurrentDoc();

	if (mw.m_runInfoPage && !quietUi)
	{
		mw.m_runInfoPage->appendInfo(
			QStringLiteral("URDF robot added (hierarchical): %1, Links: %2, Mesh parts: %3, Joints: %4 (instance %5)")
				.arg(fileInfo.fileName())
				.arg(linkToContainer.size())
				.arg(linkToGeometry.size())
				.arg(jointTransforms.size())
				.arg(robotBackendId));
	}

	// 【中文】刷新后端树
	mw.refreshBackendTree();

	return true;
}

