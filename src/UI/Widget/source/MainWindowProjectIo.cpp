#include "MainWindow.h"

#include "BackendSceneDocumentFacade.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <QByteArray>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QList>
#include <QMessageBox>
#include <QSet>
#include <QTemporaryDir>

#include <osg/Matrixd>
#include <osg/Vec3f>

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"
#include "DocumentPage.h"
#include "FollowAttachmentComponent.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "ProjectPackageZip.h"
#include "RobotInstructionFactory.h"
#include "RobotProgramStore.h"
#include "RobotCoordinateFrames.h"
#include "RobotSceneKinematics.h"
#include "RunInfoPage.h"
#include "UrdfRobotLoader.h"
#include "../RobotWidget/inc/IRobotDocumentHost.h"
#include "../RobotWidget/inc/RobotAxisControlWidget.h"
#include "../RobotWidget/inc/RobotProjectIoAdapter.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"
#include "MainWindowRobotHost.h"

namespace {
QJsonObject stdJsonToQJsonObject(const nlohmann::json& in)
{
	const std::string payload = in.dump();
	const QJsonDocument doc = QJsonDocument::fromJson(
		QByteArray(payload.data(), static_cast<int>(payload.size())));
	return doc.isObject() ? doc.object() : QJsonObject{};
}

nlohmann::json qJsonObjectToStdJson(const QJsonObject& in)
{
	const QByteArray payload = QJsonDocument(in).toJson(QJsonDocument::Compact);
	try
	{
		return nlohmann::json::parse(payload.constData(), payload.constData() + payload.size());
	}
	catch (...)
	{
		return nlohmann::json::object();
	}
}

bool restorePerLinkRobotKinematicsFromJson(
	DocumentPage* page,
	OsgWidget* osg,
	RunInfoPage* runInfo,
	const QJsonObject& rk,
	QVector<double>& outAllJointAnglesRad)
{
	if (!page || !osg || rk.value(QStringLiteral("mode")).toString() != QStringLiteral("perLink"))
	{
		return false;
	}
	const QString urdf = rk.value(QStringLiteral("urdf")).toString();
	const QString sceneRoot = rk.value(QStringLiteral("sceneRootBackendId")).toString();
	const QString jointRoot = rk.value(QStringLiteral("jointPrefixRoot")).toString();
	const QString importKey = rk.value(QStringLiteral("importKey")).toString();
	const QJsonObject linksJ = rk.value(QStringLiteral("links")).toObject();
	QHash<QString, QString> linkMap;
	for (auto it = linksJ.constBegin(); it != linksJ.constEnd(); ++it)
	{
		linkMap.insert(it.key(), it.value().toString());
	}
	for (auto it = linkMap.constBegin(); it != linkMap.constEnd(); ++it)
	{
		if (!page->backend().contains(it.value().toStdString()))
		{
			return false;
		}
	}
	if (urdf.isEmpty() || !QFileInfo::exists(urdf) || sceneRoot.isEmpty() || jointRoot.isEmpty() || linkMap.isEmpty())
	{
		return false;
	}
	const bool meshInLinkFrame = rk.value(QStringLiteral("meshInLinkFrame")).toBool();
	QStringList jn;
	QVector<double> lo;
	QVector<double> hi;
	(void)UrdfRobotLoader::loadRevoluteJointMeta(urdf, jn, lo, hi, nullptr);
	QVector<double> q0(jn.size(), 0.0);
	const QJsonArray savedJoints = rk.value(QStringLiteral("jointAnglesRad")).toArray();
	if (savedJoints.size() == jn.size())
	{
		for (int i = 0; i < jn.size(); ++i)
		{
			q0[i] = savedJoints.at(i).toDouble(0.0);
		}
	}
	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(urdf, q0, Tq, &fkErr, meshInLinkFrame))
	{
		if (runInfo)
		{
			runInfo->appendWarning(QStringLiteral("robotKinematics: FK restore failed: %1").arg(fkErr));
		}
		return false;
	}
	QHash<QString, osg::Matrixd> fkT0;
	QHash<QString, osg::Matrixd> outer;
	for (auto it = linkMap.constBegin(); it != linkMap.constEnd(); ++it)
	{
		const QString& lname = it.key();
		if (Tq.contains(lname))
		{
			const osg::Matrixd& meshWorld0 = Tq[lname];
			fkT0.insert(lname, meshWorld0);
			outer.insert(it.value(), meshWorld0);
		}
	}
	page->appendHierarchicalRobotSimulationContext(
		urdf, jn, lo, hi, QHash<QString, osg::MatrixTransform*>(), sceneRoot, jointRoot);
	page->setRobotPerLinkKinematicsBinding(importKey, linkMap, fkT0, outer, meshInLinkFrame);
	const QJsonObject cfObj = rk.value(QStringLiteral("coordinateFrames")).toObject();
	if (!cfObj.isEmpty())
	{
		const QByteArray raw = QJsonDocument(cfObj).toJson(QJsonDocument::Compact);
		try
		{
			const nlohmann::json cfJ = nlohmann::json::parse(raw.constData(), raw.constData() + raw.size());
			RobotCoordinate::RobotCoordinateFrameSet frames;
			if (RobotCoordinate::readCoordinateFrameSetFromJson(cfJ, frames))
			{
				const int instIdx = page->robotKinematicInstanceCount() - 1;
				if (instIdx >= 0)
				{
					page->robotCoordinateFramesForInstance(instIdx) = std::move(frames);
				}
			}
		}
		catch (...)
		{
		}
	}
	else
	{
		const int instIdx = page->robotKinematicInstanceCount() - 1;
		if (instIdx >= 0)
		{
			QString defaultFlange;
			QStringList childLinks;
			(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdf, childLinks, nullptr);
			if (!childLinks.isEmpty())
			{
				defaultFlange = childLinks.back();
			}
			page->robotCoordinateFramesForInstance(instIdx) =
				RobotCoordinate::makeDefaultFrameSet(defaultFlange.toStdString());
		}
	}
	outAllJointAnglesRad += q0;
	return true;
}

void rebuildLegacyParentMirror(DocumentPage* page)
{
	if (!page)
	{
		return;
	}
	QMap<QString, QString>& parentMap = page->backendParentId();
	parentMap.clear();
	const auto all = page->backend().listData();
	for (const auto& data : all)
	{
		if (!data)
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		const std::vector<std::string> parents = page->backend().parentsOf(data->id());
		if (parents.empty())
		{
			parentMap[id] = QString();
			continue;
		}
		parentMap[id] = QString::fromStdString(parents.front());
	}
}

void applyPointCloudPoseFromJson(PointCloudBackendData& pc, OsgWidget* osgWidget, const QJsonObject& obj)
{
	const QJsonObject pose = obj.value(QStringLiteral("pose")).toObject();
	const QJsonObject rot = obj.value(QStringLiteral("rotation")).toObject();
	const QJsonObject col = obj.value(QStringLiteral("color")).toObject();
	const BackendVec3 p{
		pose.value(QStringLiteral("x")).toDouble(),
		pose.value(QStringLiteral("y")).toDouble(),
		pose.value(QStringLiteral("z")).toDouble()
	};
	const BackendVec3 r{
		rot.value(QStringLiteral("x")).toDouble(),
		rot.value(QStringLiteral("y")).toDouble(),
		rot.value(QStringLiteral("z")).toDouble()
	};
	const BackendColor c{
		static_cast<float>(col.value(QStringLiteral("r")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("g")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("b")).toDouble(1.0)),
		static_cast<float>(col.value(QStringLiteral("a")).toDouble(1.0))
	};
	pc.setPose(p);
	pc.setRotation(r);
	pc.setColor(c);
	if (osgWidget)
	{
		osgWidget->setSelectedPosition(osg::Vec3f(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)));
		osgWidget->setSelectedRotationEulerDeg(osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z)));
		osgWidget->setSelectedColor(c.r, c.g, c.b, c.a);
	}
}

} // namespace

void MainWindow::onSaveProject()
{
	DocumentPage* doc = currentPage();
	if (!doc)
	{
		return;
	}
	const QString savePath = QFileDialog::getSaveFileName(
		this,
		i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
		QString(),
		QStringLiteral("Point Cloud Package (*.pcp);;PointCloud Project (*.pcproj.json);;JSON Files (*.json);;All Files (*.*)"));
	if (savePath.isEmpty())
	{
		return;
	}

	const QFileInfo saveFileInfo(savePath);
	const bool packageMode = saveFileInfo.suffix().compare(QStringLiteral("pcp"), Qt::CaseInsensitive) == 0;
	QTemporaryDir packageTemp;
	const QString workRoot = packageMode ? packageTemp.path() : saveFileInfo.absolutePath();
	if (packageMode && !packageTemp.isValid())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
			i18n(QStringLiteral("Cannot create a temporary folder to build the package."),
				QStringLiteral("无法创建临时目录以打包工程。")));
		return;
	}
	const QString jsonWritePath = packageMode ? QDir(workRoot).filePath(QStringLiteral("project.json")) : savePath;

	QJsonObject root;
	root.insert(QStringLiteral("version"), 4);
	if (packageMode)
	{
		root.insert(QStringLiteral("bundle"), QStringLiteral("zip"));
	}
	root.insert(QStringLiteral("language"), m_useChinese ? QStringLiteral("zh") : QStringLiteral("en"));

	QJsonArray objects;
	const auto dataList = doc->backend().listData();
	const int pointCloudObjectCount = static_cast<int>(std::count_if(dataList.begin(), dataList.end(),
		[](const std::shared_ptr<BackendDataBase>& d) {
			return d && std::dynamic_pointer_cast<PointCloudBackendData>(d);
		}));

	for (const auto& data : dataList)
	{
		if (!data) continue;
		const std::string id = data->id();
		const QString idQs = QString::fromStdString(id);
		const QString srcPath = doc->backendSourcePath().count(idQs) ? doc->backendSourcePath()[idQs] : QString();
		const QString sourceType = doc->backendSourceType().count(idQs) ? doc->backendSourceType()[idQs] : QString();
		const std::vector<std::string> parentIds = doc->backend().parentsOf(id);

		if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(data))
		{
			if (pc->pointPositionsXyz().empty() && pointCloudObjectCount == 1 && doc->osgWidget())
			{
				QString resyncErr;
				if (!doc->osgWidget()->captureImportedPointCloudBackend(*pc, &resyncErr) && m_runInfoPage)
				{
					m_runInfoPage->appendWarning(QStringLiteral("Save: could not embed point cloud from viewer: %1").arg(resyncErr));
				}
			}
			if (pc->pointPositionsXyz().empty())
			{
				QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
					i18n(QStringLiteral("Point cloud has no coordinates in the backend; cannot save. Re-import the file while it still exists on disk."),
						QStringLiteral("后端没有点云坐标，无法保存。请在源文件仍在磁盘上时重新导入。")));
				return;
			}
		}

		QJsonObject obj = stdJsonToQJsonObject(data->saveToJson());
		obj.insert(QStringLiteral("sourcePath"), srcPath);
		obj.insert(QStringLiteral("sourceType"), sourceType);
		obj.insert(QStringLiteral("parentId"), parentIds.empty() ? QString() : QString::fromStdString(parentIds.front()));
		objects.push_back(obj);
	}
	root.insert(QStringLiteral("objects"), objects);
	QJsonArray edgeArray;
	for (const auto& edge : doc->backend().listEdges())
	{
		QJsonObject edgeObj;
		edgeObj.insert(QStringLiteral("parentId"), QString::fromStdString(edge.first));
		edgeObj.insert(QStringLiteral("childId"), QString::fromStdString(edge.second));
		edgeArray.push_back(edgeObj);
	}
	root.insert(QStringLiteral("edges"), edgeArray);

	if (m_robotHost)
	{
		if (IRobotDocumentHost* robotDoc = m_robotHost->document())
		{
			const QVector<double>* jointAngles = nullptr;
			QVector<double> anglesToSave;
			if (m_robotSimulation)
			{
				anglesToSave = m_robotSimulation->aggregatedJointAnglesRad();
			}
			if (anglesToSave.isEmpty() && m_robotHost->robotAxisControlPage()
				&& robotDoc->hasRobotSimulationContext())
			{
				const int total = robotDoc->robotRevoluteJointNames().size();
				if (m_robotHost->robotAxisControlPage()->jointCount() == total)
				{
					anglesToSave = m_robotHost->robotAxisControlPage()->jointAnglesRad();
				}
			}
			if (!anglesToSave.isEmpty())
			{
				jointAngles = &anglesToSave;
			}
			RobotProjectIo::writeRobotKinematicsAndPrograms(root, robotDoc, jointAngles);
		}
	}

	QJsonArray annArray;
	if (OsgWidget* w = doc->osgWidget())
	{
		const auto snapshots = w->annotationSnapshots();
		for (const auto& s : snapshots)
		{
			QJsonObject a;
			a.insert(QStringLiteral("id"), s.id);
			a.insert(QStringLiteral("displayText"), s.displayText);
			a.insert(QStringLiteral("backendId"), s.backendId);
			QJsonObject local;
			local.insert(QStringLiteral("x"), s.localCentered.x());
			local.insert(QStringLiteral("y"), s.localCentered.y());
			local.insert(QStringLiteral("z"), s.localCentered.z());
			a.insert(QStringLiteral("localCentered"), local);
			if (s.hasWorldAnchor)
			{
				QJsonObject w;
				w.insert(QStringLiteral("x"), s.worldAnchor.x());
				w.insert(QStringLiteral("y"), s.worldAnchor.y());
				w.insert(QStringLiteral("z"), s.worldAnchor.z());
				a.insert(QStringLiteral("worldAnchor"), w);
			}
			a.insert(QStringLiteral("visible"), s.visible);
			annArray.push_back(a);
		}
	}
	root.insert(QStringLiteral("annotations"), annArray);
	if (OsgWidget* camW = doc->osgWidget())
	{
		root.insert(QStringLiteral("cameraFollowBackendId"), QString::fromStdString(camW->cameraFollowBackendId()));
	}

	QFile file(jsonWritePath);
	if (!file.open(QIODevice::WriteOnly))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
			i18n(QStringLiteral("Failed to write project file."), QStringLiteral("写入工程文件失败。")));
		return;
	}
	file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
	file.close();

	if (packageMode)
	{
		QString zipErr;
		if (!project_package_zip::zipDirectoryTree(savePath, workRoot, &zipErr))
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
				i18n(QStringLiteral("Failed to write package (.pcp): %1").arg(zipErr),
					QStringLiteral("写入打包文件 (.pcp) 失败：%1").arg(zipErr)));
			return;
		}
	}

	if (m_runInfoPage) m_runInfoPage->appendInfo(QStringLiteral("Project saved: %1").arg(savePath));
	doc->setProjectFilePath(savePath);
	if (m_documentTabs)
	{
		const int idx = m_documentTabs->indexOf(doc);
		if (idx >= 0)
		{
			m_documentTabs->setTabText(idx, saveFileInfo.fileName());
		}
	}
}

void MainWindow::onOpenProjectFile()
{
	DocumentPage* page = currentPage();
	if (!page)
	{
		return;
	}
	const QString openPath = QFileDialog::getOpenFileName(
		this,
		i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
		QString(),
		QStringLiteral("Point Cloud Package (*.pcp);;PointCloud Project (*.pcproj.json);;JSON Files (*.json);;All Files (*.*)"));
	if (openPath.isEmpty())
	{
		return;
	}

	QTemporaryDir zipExtractDir;
	QString projectJsonPath = openPath;
	QString projectDir = QFileInfo(openPath).absolutePath();
	if (project_package_zip::isZipArchiveFile(openPath))
	{
		if (!zipExtractDir.isValid())
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
				i18n(QStringLiteral("Cannot create a temporary folder to unpack the project."),
					QStringLiteral("无法创建临时目录解压工程。")));
			return;
		}
		QString unpackErr;
		if (!project_package_zip::extractZipArchive(openPath, zipExtractDir.path(), &unpackErr))
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")), unpackErr);
			return;
		}
		projectJsonPath = QDir(zipExtractDir.path()).filePath(QStringLiteral("project.json"));
		projectDir = zipExtractDir.path();
		if (!QFileInfo::exists(projectJsonPath))
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
				i18n(QStringLiteral("The archive does not contain project.json."),
					QStringLiteral("压缩包中没有 project.json。")));
			return;
		}
	}

	QFile file(projectJsonPath);
	if (!file.open(QIODevice::ReadOnly))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
			i18n(QStringLiteral("Failed to open project file."), QStringLiteral("打开工程文件失败。")));
		return;
	}
	const QJsonDocument jsonDoc = QJsonDocument::fromJson(file.readAll());
	file.close();
	if (!jsonDoc.isObject())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
			i18n(QStringLiteral("Invalid project format."), QStringLiteral("工程文件格式无效。")));
		return;
	}

	page->backend().clear();
	page->clearRobotSimulationContext();
	page->backendSourcePath().clear();
	page->backendSourceType().clear();
	page->backendParentId().clear();
	if (OsgWidget* wClear = page->osgWidget())
	{
		wClear->clearAllAnnotations();
		wClear->clearCameraFollowBackendId();
	}

	OsgWidget* osg = page->osgWidget();

	const QJsonObject root = jsonDoc.object();
	bool projectHadPrograms = false;
	bool projectRobotKinematicsRestored = false;
	QVector<double> projectLoadedJointAngles;
	const int projectVersion = root.value(QStringLiteral("version")).toInt(0);
	if (projectVersion != 4)
	{
		QMessageBox::warning(this,
			i18n(QStringLiteral("Open Project"), QStringLiteral("打开工程")),
			i18n(QStringLiteral("Unsupported project version: %1. This build only supports project.json v4.")
					 .arg(projectVersion),
				QStringLiteral("不支持的工程版本：%1。当前版本仅支持 project.json v4。").arg(projectVersion)));
		if (m_runInfoPage)
		{
			m_runInfoPage->appendWarning(
				QStringLiteral("Project load aborted: unsupported project version %1 (expected v4).")
					.arg(projectVersion));
		}
		return;
	}
	const QJsonArray edgesJson = root.value(QStringLiteral("edges")).toArray();
	std::vector<std::pair<QString, QString>> pendingEdges;
	pendingEdges.reserve(static_cast<std::size_t>(edgesJson.size()));
	for (const QJsonValue& v : edgesJson)
	{
		if (!v.isObject())
		{
			continue;
		}
		const QJsonObject edgeObj = v.toObject();
		const QString parentId = edgeObj.value(QStringLiteral("parentId")).toString();
		const QString childId = edgeObj.value(QStringLiteral("childId")).toString();
		if (parentId.isEmpty() || childId.isEmpty() || parentId == childId)
		{
			continue;
		}
		pendingEdges.emplace_back(parentId, childId);
	}
	const bool useEdgesRelation = !pendingEdges.empty();
	const QJsonObject rk = root.value(QStringLiteral("robotKinematics")).toObject();
	const QJsonArray robotKinematicsInstances = root.value(QStringLiteral("robotKinematicsInstances")).toArray();
	QSet<QString> robotLinkMeshBackendIds;
	const auto collectRobotLinkIds = [&](const QJsonObject& rkObj) {
		if (rkObj.value(QStringLiteral("mode")).toString() != QStringLiteral("perLink")
			|| !rkObj.value(QStringLiteral("meshInLinkFrame")).toBool())
		{
			return;
		}
		const QJsonObject linksHint = rkObj.value(QStringLiteral("links")).toObject();
		for (auto it = linksHint.constBegin(); it != linksHint.constEnd(); ++it)
		{
			robotLinkMeshBackendIds.insert(it.value().toString());
		}
	};
	for (const QJsonValue& rv : robotKinematicsInstances)
	{
		if (rv.isObject())
		{
			collectRobotLinkIds(rv.toObject());
		}
	}
	if (!rk.isEmpty())
	{
		collectRobotLinkIds(rk);
	}
	ensureBackendBuiltinsRegistered();
	const QJsonArray objects = root.value(QStringLiteral("objects")).toArray();
	for (const auto& v : objects)
	{
		if (!v.isObject()) continue;
		const QJsonObject obj = v.toObject();
		const QString sourcePath = obj.value(QStringLiteral("sourcePath")).toString();
		const QString assetRelativePath = obj.value(QStringLiteral("assetRelativePath")).toString();
		const QString sourceType = obj.value(QStringLiteral("sourceType")).toString();
		const QString legacyParentId = obj.value(QStringLiteral("parentId")).toString();
		const QString persistedId = obj.value(QStringLiteral("id")).toString();
		const QString classNameVal = obj.value(QStringLiteral("className")).toString();
		const QJsonObject emb = obj.value(QStringLiteral("geometry")).toObject();
		const bool hasEmb = !emb.isEmpty();

		if (classNameVal == QStringLiteral("Compass")
			|| sourceType.compare(QStringLiteral("Compass"), Qt::CaseInsensitive) == 0)
		{
			continue;
		}

		if (classNameVal.isEmpty())
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(QStringLiteral("Skip object with empty className: %1").arg(persistedId));
			}
			continue;
		}

		if (!hasEmb && sourcePath.isEmpty() && assetRelativePath.isEmpty())
		{
			continue;
		}

		bool loaded = false;
		std::shared_ptr<BackendDataBase> backendObject = BackendRegistry::instance().create(classNameVal.toStdString());
		if (!backendObject)
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(
					QStringLiteral("Unknown backend className: %1 (id=%2)").arg(classNameVal, persistedId));
			}
			continue;
		}
		std::string loadErr;
		if (!backendObject->loadFromJson(qJsonObjectToStdJson(obj), &loadErr))
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(
					QStringLiteral("Object decode failed (%1): %2")
						.arg(persistedId, QString::fromStdString(loadErr)));
			}
			continue;
		}

		QString visualErr;
		const auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(backendObject);
		const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(backendObject);
		if (pc)
		{
			loaded = osg && osg->loadPointCloudFromBackendData(*pc, &visualErr);
		}
		else if (mesh)
		{
			loaded = osg
				&& osg->loadMeshFromBackendData(*mesh, &visualErr, true, true, true, robotLinkMeshBackendIds.contains(persistedId));
		}
		else
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(
					QStringLiteral("Unsupported backend object type (id=%1, class=%2)").arg(persistedId, classNameVal));
			}
		}

		if (loaded
			&& registerExistingBackendObject(
				backendObject,
				sourcePath,
				sourceType.isEmpty() ? (pc ? QStringLiteral("PointCloud") : QStringLiteral("Model")) : sourceType,
				persistedId,
				false,
				useEdgesRelation ? QString() : legacyParentId))
		{
			continue;
		}
		if (m_runInfoPage && !visualErr.isEmpty())
		{
			m_runInfoPage->appendWarning(
				QStringLiteral("Embedded backend visual load failed (id=%1): %2").arg(persistedId, visualErr));
		}

		QString loadPath;
		if (!sourcePath.isEmpty() && QFileInfo::exists(sourcePath))
		{
			loadPath = sourcePath;
		}
		else if (!assetRelativePath.isEmpty())
		{
			const QString bundled = QDir(projectDir).filePath(assetRelativePath);
			if (QFileInfo::exists(bundled))
			{
				loadPath = QDir::cleanPath(bundled);
			}
		}
		if (loadPath.isEmpty())
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(QStringLiteral("Missing data (no usable embedded geometry and file missing): %1")
					.arg(sourcePath.isEmpty() ? assetRelativePath : sourcePath));
			}
			continue;
		}
		const bool ok = registerBackendObject(loadPath,
			sourceType.compare(QStringLiteral("PointCloud"), Qt::CaseInsensitive) == 0
				? QStringLiteral("PointCloud")
				: QStringLiteral("Model"),
			sourceType.compare(QStringLiteral("PointCloud"), Qt::CaseInsensitive) == 0,
			true);
		if (!ok)
		{
			if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(QStringLiteral("Failed to load object from file: %1").arg(loadPath));
			}
			continue;
		}

		const auto all = page->backend().listData();
		if (!all.empty())
		{
			auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(all.back());
			if (pc)
			{
				applyPointCloudPoseFromJson(*pc, osg, obj);
			}
		}
	}

	if (useEdgesRelation)
	{
		for (const auto& edge : pendingEdges)
		{
			const std::string parentId = edge.first.toStdString();
			const std::string childId = edge.second.toStdString();
			if (!page->backend().contains(parentId) || !page->backend().contains(childId))
			{
				if (m_runInfoPage)
				{
					m_runInfoPage->appendWarning(
						QStringLiteral("Skip dangling edge: %1 -> %2").arg(edge.first, edge.second));
				}
				continue;
			}
			if (!page->backend().attachChild(parentId, childId) && m_runInfoPage)
			{
				m_runInfoPage->appendWarning(
					QStringLiteral("Skip invalid edge (cycle or duplicate): %1 -> %2").arg(edge.first, edge.second));
			}
		}
	}
	rebuildLegacyParentMirror(page);

	if (osg)
	{
		QVector<double> allQ0;
		int restored = 0;
		for (const QJsonValue& rv : robotKinematicsInstances)
		{
			if (!rv.isObject())
			{
				continue;
			}
			if (restorePerLinkRobotKinematicsFromJson(page, osg, m_runInfoPage, rv.toObject(), allQ0))
			{
				++restored;
			}
		}
		if (restored == 0 && !rk.isEmpty())
		{
			if (restorePerLinkRobotKinematicsFromJson(page, osg, m_runInfoPage, rk, allQ0))
			{
				restored = 1;
			}
			else if (m_runInfoPage)
			{
				m_runInfoPage->appendWarning(
					QStringLiteral("robotKinematics: skipped restore (missing URDF, backends, or metadata)."));
			}
		}
		if (restored > 0 && !allQ0.isEmpty())
		{
			projectRobotKinematicsRestored = true;
			projectLoadedJointAngles = allQ0;
			(void)RobotSceneKinematics::applyJointAnglesFromDocument(page, page->sceneFacade().poseSink(), allQ0);
			refreshSimulationJointListFromCurrentDoc();
			if (m_robotSimulation)
			{
				m_robotSimulation->restoreAggregatedJointStateAfterProjectLoad(allQ0);
			}
			if (simulationCommandPage() && page->robotKinematicInstanceCount() > 0)
			{
				const int instIdx = simulationCommandPage()->currentRobotInstanceIndex();
				if (instIdx >= 0)
				{
					syncRobotFrameSettingsFromDocument(instIdx);
					refreshRobotCoordinateFrameOverlays();
				}
			}
		}
	}

	if (m_robotHost)
	{
		if (IRobotDocumentHost* robotDoc = m_robotHost->document())
		{
			projectHadPrograms = !root.value(QStringLiteral("robotPrograms")).toArray().isEmpty();
			RobotProjectIo::loadRobotPrograms(
				root,
				robotDoc,
				[this](const QString& msg) {
					if (m_runInfoPage)
					{
						m_runInfoPage->appendWarning(msg);
					}
				});
			if (projectHadPrograms && simulationCommandPage())
			{
				refreshSimulationJointListFromCurrentDoc();
				simulationCommandPage()->refreshInstructionList();
			}
		}
	}

	if (osg)
	{
		for (const auto& data : page->backend().listData())
		{
			if (!data)
			{
				continue;
			}
			const std::vector<std::string> parents = page->backend().parentsOf(data->id());
			const std::string parent = parents.empty() ? std::string() : parents.front();
			osg->setBackendParent(data->id(), parent);
		}
	}
	if (useEdgesRelation)
	{
		for (const auto& edge : pendingEdges)
		{
			const QString childQ = edge.second;
			const std::shared_ptr<BackendDataBase> childData = page->backend().getData(childQ.toStdString());
			if (childData && childData->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
			{
				continue;
			}
			applyHierarchyFollowBinding(page, childQ.toStdString(), edge.first.toStdString());
		}
	}

	page->invalidateFollowReverseIndex();

	QList<OsgWidget::AnnotationSnapshot> snapshots;
	const QJsonArray annArray = root.value(QStringLiteral("annotations")).toArray();
	for (const QJsonValue& v : annArray)
	{
		if (!v.isObject()) continue;
		const QJsonObject a = v.toObject();
		OsgWidget::AnnotationSnapshot s;
		s.id = a.value(QStringLiteral("id")).toString();
		s.displayText = a.value(QStringLiteral("displayText")).toString();
		s.backendId = a.value(QStringLiteral("backendId")).toString();
		const QJsonObject local = a.value(QStringLiteral("localCentered")).toObject();
		s.localCentered = osg::Vec3f(
			static_cast<float>(local.value(QStringLiteral("x")).toDouble()),
			static_cast<float>(local.value(QStringLiteral("y")).toDouble()),
			static_cast<float>(local.value(QStringLiteral("z")).toDouble()));
		const QJsonObject world = a.value(QStringLiteral("worldAnchor")).toObject();
		if (!world.isEmpty())
		{
			s.worldAnchor = osg::Vec3f(
				static_cast<float>(world.value(QStringLiteral("x")).toDouble()),
				static_cast<float>(world.value(QStringLiteral("y")).toDouble()),
				static_cast<float>(world.value(QStringLiteral("z")).toDouble()));
			s.hasWorldAnchor = true;
		}
		s.visible = a.value(QStringLiteral("visible")).toBool(true);
		snapshots.push_back(s);
	}
	if (osg)
	{
		osg->restoreAnnotations(snapshots);
		osg->setCameraFollowBackendId(root.value(QStringLiteral("cameraFollowBackendId")).toString().toStdString());
	}

	if (page && osg)
	{
		page->requestFollowSolveForced();
		runBackendFollowSolveAndSync(*page, *osg);
	}

	if (m_robotSimulation && projectHadPrograms)
	{
		m_robotSimulation->applyProgramStartPoseAfterProjectLoad();
	}
	else if (m_robotSimulation && projectRobotKinematicsRestored && !projectLoadedJointAngles.isEmpty() && page)
	{
		(void)RobotSceneKinematics::applyJointAnglesFromDocument(
			page, page->sceneFacade().poseSink(), projectLoadedJointAngles);
		m_robotSimulation->restoreAggregatedJointStateAfterProjectLoad(projectLoadedJointAngles);
	}

	refreshBackendTree();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("Project opened: %1").arg(openPath));
	}
	page->setProjectFilePath(openPath);
	if (m_documentTabs)
	{
		const int idx = m_documentTabs->indexOf(page);
		if (idx >= 0)
		{
			m_documentTabs->setTabText(idx, QFileInfo(openPath).fileName());
		}
	}
}
