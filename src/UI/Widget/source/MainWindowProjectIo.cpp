#include "MainWindow.h"

#include "BackendHierarchyFollow.h"
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
#include "WidgetDocumentAccess.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "ProjectPackageZip.h"
#include "RobotInstructionFactory.h"
#include "RobotProgramStore.h"
#include "RobotSceneKinematics.h"
#include "RunInfoPage.h"
#include "../RobotWidget/inc/IRobotDocumentHost.h"
#include "../RobotWidget/inc/RobotAxisControlWidget.h"
#include "../RobotWidget/inc/RobotProjectIoAdapter.h"
#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/SimulationCommandWidget.h"
#include "MainWindowRobotHost.h"

#include "DocumentHostEvents.h"
#include "DocumentImportFacade.h"
#include "BackendProjectObjectIo.h"
#include "ProjectPackageIo.h"
#include "CoreTypes.h"
#include "IRobotService.h"
#include "RobotProjectKinematicsRestore.h"

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

	const QString languageCode = m_useChinese ? QStringLiteral("zh") : QStringLiteral("en");
	const cloudsim::host::ProjectSaveBuildResult built = cloudsim::host::buildProjectSaveRoot(*doc, languageCode);
	if (!built.abortMessage.isEmpty())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Save Project"), QStringLiteral("保存工程")),
			i18n(built.abortMessage,
				QStringLiteral("后端没有点云坐标，无法保存。请在源文件仍在磁盘上时重新导入。")));
		return;
	}
	if (m_runInfoPage)
	{
		for (const QString& w : built.warnings)
		{
			m_runInfoPage->appendWarning(w);
		}
	}
	QJsonObject root = built.root;
	if (packageMode)
	{
		root.insert(QStringLiteral("bundle"), QStringLiteral("zip"));
	}

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
			cloudsim::host::mergeRobotKinematicsIntoProjectRoot(robotDoc, root, jointAngles);
			cloudsim::host::mergeRobotProgramsIntoProjectRoot(*doc, root);
		}
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
	if (OsgWidget* wClear = widgetOsgFromPage(page))
	{
		wClear->clearAllAnnotations();
		wClear->clearCameraFollowBackendId();
	}

	OsgWidget* osg = widgetOsgFromPage(page);

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
	const QVector<cloudsim::host::ProjectHierarchyEdge> pendingEdges =
		cloudsim::host::parseProjectEdgesJson(root.value(QStringLiteral("edges")).toArray());
	const bool useEdgesRelation = !pendingEdges.isEmpty();
	const QSet<QString> robotLinkMeshBackendIds = cloudsim::host::collectRobotLinkMeshBackendIds(root);
	ensureBackendBuiltinsRegistered();
	beginBackendTreeEventRefreshSuppress();
	cloudsim::host::ProjectObjectLoadOptions loadOpts;
	loadOpts.projectDir = projectDir;
	loadOpts.useEdgesRelation = useEdgesRelation;
	loadOpts.robotLinkMeshBackendIds = robotLinkMeshBackendIds;
	cloudsim::host::ProjectObjectLoadCallbacks loadCbs;
	loadCbs.legacyParentFollow = [page](const std::string& childId, const std::string& parentId) {
		cloudsim::host::applyHierarchyFollowBinding(*page, childId, parentId);
	};
	loadCbs.pointCloudWidgetImport = [](cloudsim::host::DocumentHost& host, const QString& loadPath, const QString& persistedId,
									   QString& outImportedId, QString* outError) -> bool {
		cloudsim::core::ImportOptionsDto opt;
		opt.quietUi = true;
		opt.resetViewToHome = false;
		opt.persistedId = persistedId;
		opt.catalogTypeName = QStringLiteral("PointCloud");
		const cloudsim::host::ImportFileResult imported =
			cloudsim::host::importFileIntoDocument(host, loadPath, cloudsim::host::ImportFileKind::PointCloud, opt, outError);
		outImportedId = imported.rootBackendId;
		return imported.ok;
	};
	QStringList objectLoadWarnings;
	cloudsim::host::loadProjectObjectsFromJson(*page, root.value(QStringLiteral("objects")).toArray(), loadOpts, loadCbs,
		&objectLoadWarnings);
	QStringList hierarchyWarnings;
	cloudsim::host::finalizeProjectHierarchyAfterObjects(*page, useEdgesRelation, pendingEdges, &hierarchyWarnings);
	const auto appendLoadWarnings = [this](const QStringList& warnings) {
		if (!m_runInfoPage)
		{
			return;
		}
		for (const QString& w : warnings)
		{
			m_runInfoPage->appendWarning(w);
		}
	};
	appendLoadWarnings(objectLoadWarnings);
	appendLoadWarnings(hierarchyWarnings);

	if (osg)
	{
		const cloudsim::host::RobotKinematicsRestoreResult rkResult =
			cloudsim::host::restoreRobotKinematicsFromProjectJson(*page, root);
		appendLoadWarnings(rkResult.warnings);
		if (rkResult.restoredInstanceCount > 0 && !rkResult.aggregatedJointAnglesRad.isEmpty())
		{
			projectRobotKinematicsRestored = true;
			projectLoadedJointAngles = rkResult.aggregatedJointAnglesRad;
			(void)RobotSceneKinematics::applyJointAnglesFromDocument(
				page, page->sceneFacade().poseSink(), rkResult.aggregatedJointAnglesRad);
			refreshSimulationJointListFromCurrentDoc();
			if (m_robotSimulation)
			{
				m_robotSimulation->restoreAggregatedJointStateAfterProjectLoad(rkResult.aggregatedJointAnglesRad);
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

	{
		QString loadErr;
		projectHadPrograms = cloudsim::host::loadRobotProgramsFromProjectJson(*page, root, &loadErr);
		if (m_runInfoPage && !loadErr.isEmpty())
		{
			m_runInfoPage->appendWarning(loadErr);
		}
		if (projectHadPrograms && simulationCommandPage())
		{
			refreshSimulationJointListFromCurrentDoc();
			simulationCommandPage()->refreshInstructionList();
		}
	}

	if (osg)
	{
		cloudsim::host::FollowSolveContext solveCtx = makeFollowSolveContext(*osg);
		cloudsim::host::finalizeProjectLoadFollowAndViewport(*page, *osg, root, useEdgesRelation, pendingEdges, &solveCtx);
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

	endBackendTreeEventRefreshSuppress();
	refreshBackendTree();
	cloudsim::host::publishProjectLoaded(*page, openPath);
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
