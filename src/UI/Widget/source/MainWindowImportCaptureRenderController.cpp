#include "MainWindowImportCaptureRenderController.h"

#include "CoreTypes.h"
#include "DocumentImportFacade.h"
#include "IRobotService.h"
#include "JobSystem.h"
#include "MainWindow.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendSceneDocumentFacade.h"
#include "DocumentPage.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PlyIo.h"
#include "PointCloudBackendData.h"
#include "RunInfoPage.h"
#include "WidgetDocumentAccess.h"

#include <QByteArray>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <QPointer>
#include <vector>

#include <memory>

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
		if (mw.m_runInfoPage && !quietUi)
		{
			mw.m_runInfoPage->appendInfo(QStringLiteral("Backend object registered: %1").arg(fileInfo.fileName()));
		}
		mw.focusBackendInTreeAfterImport(obj);
		if (obj && doc)
		{
			doc->sceneFacade().ensureSelectionVisualForBackend(*obj);
			if (OsgWidget* osg = widgetOsgFromPage(doc))
			{
				osg->requestRedraw();
			}
		}
		// 导入后刷新轨迹生成页工件下拉（与 URDF 导入路径一致）
		mw.refreshSimulationJointListFromCurrentDoc();
	};

	if (isPointCloud)
	{
		const QString extLower = fileInfo.suffix().toLower();
		const bool isLasLaz = (extLower == QLatin1String("las") || extLower == QLatin1String("laz"));

		const auto makeConfiguredPointCloud = [fileInfo]() {
			auto p = std::make_shared<PointCloudBackendData>();
			p->setName(fileInfo.fileName().toStdString());
			BackendColor color;
			color.r = 0.65f;
			color.g = 0.82f;
			color.b = 0.95f;
			color.a = 1.0f;
			p->setColor(color);
			BackendVec3 pose{};
			p->setPose(pose);
			BackendVec3 rot{};
			p->setRotation(rot);
			return p;
		};

		if (extLower == QLatin1String("ply") && plyFileHasTriangleFaces(nativePath))
		{
			cloudsim::core::ImportOptionsDto meshOpt;
			meshOpt.quietUi = quietUi;
			meshOpt.resetViewToHome = true;
			meshOpt.catalogTypeName = QStringLiteral("Model");
			QString importErr;
			const cloudsim::host::ImportFileResult imported =
				cloudsim::host::importFileIntoDocument(*doc, filePath, cloudsim::host::ImportFileKind::Mesh, meshOpt, &importErr);
			if (!imported.ok)
			{
				return reportFail(QStringLiteral("Point cloud"),
					importErr.isEmpty() ? QStringLiteral("Failed to load PLY mesh.") : importErr);
			}
			const std::shared_ptr<BackendDataBase> obj = doc->backend().getData(imported.rootBackendId.toStdString());
			if (!obj)
			{
				return reportFail(QStringLiteral("Point cloud"), QStringLiteral("Imported object not found in backend."));
			}
			finish(obj);
			return true;
		}

		// Job 回调可能在栈帧返回后执行，fileInfo 须按值捕获
		if (!isLasLaz && mw.jobSystem())
		{
			struct CgalPcPayload
			{
				bool ok = false;
				std::string err;
				std::shared_ptr<PointCloudBackendData> pc;
			};
			const auto payload = std::make_shared<CgalPcPayload>();
			const QPointer<MainWindow> mwPtr(&mw);
			const QPointer<DocumentPage> docPtr(doc);

			mw.jobSystem()->enqueue(
				QStringLiteral("Point cloud: %1").arg(fileInfo.fileName()),
				[payload, nativePath, makeConfiguredPointCloud](const JobProgressSink& sink) {
					sink(0.05, QStringLiteral("CGAL read / decode..."));
					payload->pc = makeConfiguredPointCloud();
					payload->ok = payload->pc->loadFromFile(nativePath, &payload->err);
					sink(1.0, QString());
				},
				[payload, mwPtr, docPtr, filePath, typeName, quietUi, fileInfo](bool threw, const QString& throwMsg) {
					if (!mwPtr || !docPtr)
					{
						return;
					}
					MainWindow& mwRef = *mwPtr;
					DocumentPage& docRef = *docPtr;
					const auto uiFail = [&](const QString& title, const QString& msg) {
						if (quietUi)
						{
							if (mwRef.m_runInfoPage)
							{
								mwRef.m_runInfoPage->appendWarning(title + QStringLiteral(": ") + msg);
							}
						}
						else
						{
							QMessageBox::warning(&mwRef, title, msg);
						}
					};
					if (threw)
					{
						uiFail(QStringLiteral("Point cloud"),
							throwMsg.isEmpty() ? QStringLiteral("Background import failed.") : throwMsg);
						return;
					}
					if (!payload->ok)
					{
						uiFail(QStringLiteral("Point cloud"),
							QString::fromStdString(payload->err.empty() ? std::string("Failed to load point cloud.") : payload->err));
						return;
					}
					const std::shared_ptr<PointCloudBackendData> pc = payload->pc;
					cloudsim::host::AdoptPointCloudOptions adoptOpt;
					adoptOpt.sourcePath = filePath;
					adoptOpt.catalogTypeName = typeName;
					adoptOpt.resetViewToHome = true;
					QString regErr;
					const cloudsim::host::AdoptRegistrationResult adopted =
						cloudsim::host::registerAdoptedPointCloud(docRef, pc, adoptOpt, &regErr);
					if (!adopted.ok)
					{
						uiFail(QStringLiteral("Backend Register"),
							regErr.isEmpty() ? QStringLiteral("Failed to register point cloud.") : regErr);
						return;
					}
					if (mwRef.m_runInfoPage && !quietUi)
					{
						mwRef.m_runInfoPage->appendInfo(
							QStringLiteral("Backend object registered: %1").arg(fileInfo.fileName()));
					}
					mwRef.focusBackendInTreeAfterImport(pc);
				});
			return true;
		}

		cloudsim::core::ImportOptionsDto opt;
		opt.quietUi = quietUi;
		opt.resetViewToHome = true;
		opt.catalogTypeName = typeName;
		QString importErr;
		const cloudsim::host::ImportFileResult imported =
			cloudsim::host::importFileIntoDocument(*doc, filePath, cloudsim::host::ImportFileKind::PointCloud, opt, &importErr);
		if (!imported.ok)
		{
			return reportFail(QStringLiteral("Point cloud"),
				importErr.isEmpty() ? QStringLiteral("Failed to load point cloud.") : importErr);
		}
		const std::shared_ptr<BackendDataBase> obj = doc->backend().getData(imported.rootBackendId.toStdString());
		if (!obj)
		{
			return reportFail(QStringLiteral("Point cloud"), QStringLiteral("Imported object not found in backend."));
		}
		finish(obj);
		return true;
	}

	const QString ext = fileInfo.suffix().toLower();
	if (ext == QLatin1String("dxf") && mw.m_runInfoPage)
	{
		std::vector<MeshHierarchyPart> dxfParts;
		std::string dxfErr;
		const bool dxfHierOk = MeshBackendData::loadDxfHierarchyFromFile(nativePath, dxfParts, &dxfErr);
		mw.m_runInfoPage->appendInfo(
			QStringLiteral("DXF hierarchy parse: ok=%1, parts=%2, err=%3")
				.arg(dxfHierOk ? QStringLiteral("true") : QStringLiteral("false"))
				.arg(static_cast<int>(dxfParts.size()))
				.arg(QString::fromStdString(dxfErr)));
	}

	const MainWindow::ScopedBackendTreeRefreshSuppress treeSuppressGuard(mw);
	cloudsim::core::ImportOptionsDto importOpt;
	importOpt.quietUi = quietUi;
	importOpt.resetViewToHome = true;
	importOpt.catalogTypeName = typeName;
	QString importErr;
	const cloudsim::host::ImportFileResult imported =
		cloudsim::host::importFileIntoDocument(*doc, filePath, cloudsim::host::ImportFileKind::Mesh, importOpt, &importErr);
	if (!imported.ok)
	{
		return reportFail(QStringLiteral("Model"), importErr.isEmpty() ? QStringLiteral("Import failed.") : importErr);
	}

	mw.refreshBackendTree();
	if (imported.hierarchyImport)
	{
		if (imported.hierarchyDetail.importParent)
		{
			mw.focusBackendInTreeAfterImport(imported.hierarchyDetail.importParent);
		}
		else if (imported.hierarchyDetail.lastRegisteredMesh)
		{
			finish(imported.hierarchyDetail.lastRegisteredMesh);
		}
		return true;
	}

	const std::shared_ptr<BackendDataBase> obj = doc->backend().getData(imported.rootBackendId.toStdString());
	if (!obj)
	{
		return reportFail(QStringLiteral("Model"), QStringLiteral("Imported object not found in backend."));
	}
	finish(obj);
	return true;
}

bool MainWindowImportCaptureRenderController::registerUrdfRobot(MainWindow& mw, const QString& urdfFilePath, bool quietUi)
{
	DocumentPage* doc = mw.currentPage();
	if (!doc)
	{
		return false;
	}

	cloudsim::core::ImportOptionsDto options;
	options.quietUi = quietUi;

	const cloudsim::core::RobotRegistrationDto result = doc->robot().registerUrdfRobot(urdfFilePath, options);

	if (!result.ok)
	{
		const QString title = QStringLiteral("URDF");
		const QString& msg = result.error;
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
	}

	if (mw.m_runInfoPage)
	{
		for (const QString& w : result.warnings)
		{
			mw.m_runInfoPage->appendWarning(w);
		}
	}

	mw.refreshSimulationJointListFromCurrentDoc();

	if (mw.m_runInfoPage && !quietUi)
	{
		mw.m_runInfoPage->appendInfo(
			QStringLiteral("URDF robot added (per-link backends): %1, Links: %2, Joints: %3, Root=%4")
				.arg(result.sourceDisplayName)
				.arg(result.linkCount)
				.arg(result.jointCount)
				.arg(result.sceneRootBackendId));
	}

	return true;
}
