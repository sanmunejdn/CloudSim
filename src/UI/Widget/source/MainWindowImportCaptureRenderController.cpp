#include "MainWindowImportCaptureRenderController.h"

#include "CoreTypes.h"
#include "DocumentImportFacade.h"
#include "IRobotService.h"
#include "JobSystem.h"
#include "MainWindow.h"
#include "PlyIo.h"

#include "DocumentPage.h"
#include "IRenderView.h"
#include "RunInfoPage.h"

#include <QByteArray>
#include <QFile>
#include <QFileInfo>
#include <QMessageBox>
#include <QPointer>

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

	auto finish = [&](const QString& backendId) {
		if (backendId.isEmpty())
		{
			return;
		}
		if (mw.m_runInfoPage && !quietUi)
		{
			mw.m_runInfoPage->appendInfo(QStringLiteral("Backend object registered: %1").arg(fileInfo.fileName()));
		}
		mw.focusBackendInTreeAfterImport(backendId);
		doc->render().ensureSelectionVisualForBackend(backendId, false);
		doc->render().requestRedraw();
		mw.refreshSimulationJointListFromCurrentDoc();
	};

	if (isPointCloud)
	{
		const QString extLower = fileInfo.suffix().toLower();
		const bool isLasLaz = (extLower == QLatin1String("las") || extLower == QLatin1String("laz"));
		const QByteArray nativeEnc = QFile::encodeName(filePath);
		const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));

		if (extLower == QLatin1String("ply") && plyFileHasTriangleFaces(nativePath))
		{
			cloudsim::core::ImportOptionsDto meshOpt;
			meshOpt.quietUi = quietUi;
			meshOpt.resetViewToHome = true;
			meshOpt.catalogTypeName = QStringLiteral("Model");
			QString importErr;
			const cloudsim::host::ImportFileResult imported = cloudsim::host::importFileIntoDocument(
				*doc, filePath, cloudsim::host::ImportFileKind::Mesh, meshOpt, &importErr);
			if (!imported.ok)
			{
				return reportFail(QStringLiteral("Point cloud"),
					importErr.isEmpty() ? QStringLiteral("Failed to load PLY mesh.") : importErr);
			}
			finish(imported.rootBackendId);
			return true;
		}

		if (!isLasLaz && mw.jobSystem())
		{
			const auto loadState = std::make_shared<cloudsim::host::PointCloudBackgroundLoadState>(
				filePath, fileInfo.fileName());
			const auto loadOk = std::make_shared<bool>(false);
			const auto loadErr = std::make_shared<QString>();
			const QPointer<MainWindow> mwPtr(&mw);
			const QPointer<DocumentPage> docPtr(doc);

			mw.jobSystem()->enqueue(
				QStringLiteral("Point cloud: %1").arg(fileInfo.fileName()),
				[loadState, loadOk, loadErr](const JobProgressSink& sink) {
					*loadOk = loadState->executeLoad(
						[&](const double progress01, const QString& status) { sink(progress01, status); },
						loadErr.get());
				},
				[loadState, loadOk, loadErr, mwPtr, docPtr, filePath, typeName, quietUi, fileInfo](
					const bool threw, const QString& throwMsg) {
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
					if (!*loadOk)
					{
						uiFail(QStringLiteral("Point cloud"),
							loadErr->isEmpty() ? QStringLiteral("Failed to load point cloud.") : *loadErr);
						return;
					}
					cloudsim::host::AdoptPointCloudOptions adoptOpt;
					adoptOpt.sourcePath = filePath;
					adoptOpt.catalogTypeName = typeName;
					adoptOpt.resetViewToHome = true;
					QString regErr;
					const cloudsim::host::AdoptRegistrationResult adopted =
						loadState->adoptIntoDocument(docRef, adoptOpt, &regErr);
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
					mwRef.focusBackendInTreeAfterImport(adopted.backendId);
					docRef.render().ensureSelectionVisualForBackend(adopted.backendId, false);
					docRef.render().requestRedraw();
					mwRef.refreshSimulationJointListFromCurrentDoc();
				});
			return true;
		}

		cloudsim::core::ImportOptionsDto opt;
		opt.quietUi = quietUi;
		opt.resetViewToHome = true;
		opt.catalogTypeName = typeName;
		QString importErr;
		const cloudsim::host::ImportFileResult imported = cloudsim::host::importFileIntoDocument(
			*doc, filePath, cloudsim::host::ImportFileKind::PointCloud, opt, &importErr);
		if (!imported.ok)
		{
			return reportFail(QStringLiteral("Point cloud"),
				importErr.isEmpty() ? QStringLiteral("Failed to load point cloud.") : importErr);
		}
		finish(imported.rootBackendId);
		return true;
	}

	if (mw.jobSystem())
	{
		const auto loadState = std::make_shared<cloudsim::host::ModelBackgroundLoadState>(
			filePath, fileInfo.fileName(), typeName, mw.meshImportQuality());
		const auto loadOk = std::make_shared<bool>(false);
		const auto loadErr = std::make_shared<QString>();
		const QPointer<MainWindow> mwPtr(&mw);
		const QPointer<DocumentPage> docPtr(doc);

		mw.jobSystem()->enqueue(
			QStringLiteral("Import model: %1").arg(fileInfo.fileName()),
			[loadState, loadOk, loadErr](const JobProgressSink& sink) {
				*loadOk = loadState->executeLoad(
					[&](const double progress01, const QString& status) { sink(progress01, status); },
					loadErr.get());
			},
			[loadState, loadOk, loadErr, mwPtr, docPtr, filePath, typeName, quietUi, fileInfo](
				const bool threw, const QString& throwMsg) {
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
					uiFail(QStringLiteral("Model"),
						throwMsg.isEmpty() ? QStringLiteral("Background import failed.") : throwMsg);
					return;
				}
				if (!*loadOk)
				{
					uiFail(QStringLiteral("Model"),
						loadErr->isEmpty() ? QStringLiteral("Failed to load model.") : *loadErr);
					return;
				}
				const MainWindow::ScopedBackendTreeRefreshSuppress treeSuppressGuard(mwRef);
				cloudsim::core::ImportOptionsDto importOpt;
				importOpt.quietUi = quietUi;
				importOpt.resetViewToHome = true;
				importOpt.catalogTypeName = typeName;
				importOpt.meshImportQuality = mwRef.meshImportQuality();
				QString importErr;
				const cloudsim::host::ImportFileResult imported =
					loadState->finishIntoDocument(docRef, importOpt, &importErr);
				if (!imported.ok)
				{
					uiFail(QStringLiteral("Model"),
						importErr.isEmpty() ? QStringLiteral("Import failed.") : importErr);
					return;
				}
				mwRef.refreshBackendTree();
				if (imported.hierarchyImport)
				{
					const QString focusId = imported.hierarchyFocusBackendId();
					if (!focusId.isEmpty())
					{
						mwRef.focusBackendInTreeAfterImport(focusId);
					}
					else
					{
						const QString meshId = imported.hierarchyLastMeshBackendId();
						if (!meshId.isEmpty())
						{
							mwRef.focusBackendInTreeAfterImport(meshId);
							docRef.render().ensureSelectionVisualForBackend(meshId, false);
						}
					}
					docRef.render().requestRedraw();
					mwRef.refreshSimulationJointListFromCurrentDoc();
					if (loadState->needsPickArtifactWarm() && mwRef.jobSystem())
					{
						mwRef.jobSystem()->enqueue(
							QStringLiteral("BREP pick warm: %1").arg(fileInfo.fileName()),
							[loadState](const JobProgressSink&) {
								(void)loadState->warmPickArtifacts(nullptr);
							},
							[](const bool, const QString&) {});
					}
					return;
				}
				if (!imported.rootBackendId.isEmpty())
				{
					if (mwRef.m_runInfoPage && !quietUi)
					{
						mwRef.m_runInfoPage->appendInfo(
							QStringLiteral("Backend object registered: %1").arg(fileInfo.fileName()));
					}
					mwRef.focusBackendInTreeAfterImport(imported.rootBackendId);
					docRef.render().ensureSelectionVisualForBackend(imported.rootBackendId, false);
					docRef.render().requestRedraw();
					mwRef.refreshSimulationJointListFromCurrentDoc();
				}
				if (loadState->needsPickArtifactWarm() && mwRef.jobSystem())
				{
					mwRef.jobSystem()->enqueue(
						QStringLiteral("BREP pick warm: %1").arg(fileInfo.fileName()),
						[loadState](const JobProgressSink&) {
							(void)loadState->warmPickArtifacts(nullptr);
						},
						[](const bool, const QString&) {});
				}
			});
		return true;
	}

	const MainWindow::ScopedBackendTreeRefreshSuppress treeSuppressGuard(mw);
	cloudsim::core::ImportOptionsDto importOpt;
	importOpt.quietUi = quietUi;
	importOpt.resetViewToHome = true;
	importOpt.catalogTypeName = typeName;
	importOpt.meshImportQuality = mw.meshImportQuality();
	QString importErr;
	const cloudsim::host::ImportFileResult imported = cloudsim::host::importFileIntoDocument(
		*doc, filePath, cloudsim::host::ImportFileKind::Mesh, importOpt, &importErr);
	if (!imported.ok)
	{
		return reportFail(QStringLiteral("Model"), importErr.isEmpty() ? QStringLiteral("Import failed.") : importErr);
	}

	mw.refreshBackendTree();
	if (imported.hierarchyImport)
	{
		const QString focusId = imported.hierarchyFocusBackendId();
		if (!focusId.isEmpty())
		{
			mw.focusBackendInTreeAfterImport(focusId);
		}
		else
		{
			const QString meshId = imported.hierarchyLastMeshBackendId();
			if (!meshId.isEmpty())
			{
				finish(meshId);
			}
		}
		return true;
	}

	finish(imported.rootBackendId);
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
