/// @file PluginHostContext.cpp
/// @brief PluginHostContext 实现

#include "PluginHostContext.h"

#include "Ai/AiAssistantHostImpl.h"
#include "Ai/AiTrajectoryFeatureCatalog.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "BackendPrimitiveGeometry.h"
#include "BackendRegistry.h"
#include "BackendSceneDocumentFacade.h"
#include "BrepBackendData.h"
#include "BackendFileImport.h"
#include "CloudSimPluginVersion.h"
#include "CoreTypes.h"
#include "DocumentHost.h"
#include "DocumentImportFacade.h"
#include "GeometryRef.h"
#include "IAiAssistantHost.h"
#include "IDataService.h"
#include "IPluginMainWindowHost.h"
#include "IRenderView.h"
#include "MeshBackendData.h"
#include "MeshBoolean.h"
#include "PrimitiveBrep.h"
#include "PluginDelegatedBackend.h"
#include "PluginDocumentAdapter.h"
#include "PluginGeometryHostImpl.h"
#include "PluginLabelingHostImpl.h"
#include "PluginPointCloudHostImpl.h"
#include "RunLogger.h"

#include <QAction>
#include <QCoreApplication>
#include <QDockWidget>
#include <QFileInfo>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMetaObject>
#include <QTabWidget>
#include <QThread>
#include <QUuid>

namespace
{
BackendPrimitiveGeometry::PrimitiveKind toDataKind(PluginPrimitiveKind kind)
{
	switch (kind)
	{
	case PluginPrimitiveKind::Cylinder:
		return BackendPrimitiveGeometry::PrimitiveKind::Cylinder;
	case PluginPrimitiveKind::Sphere:
		return BackendPrimitiveGeometry::PrimitiveKind::Sphere;
	case PluginPrimitiveKind::Cone:
		return BackendPrimitiveGeometry::PrimitiveKind::Cone;
	case PluginPrimitiveKind::Box:
	default:
		return BackendPrimitiveGeometry::PrimitiveKind::Box;
	}
}

BackendPrimitiveGeometry::PrimitiveMeshParams toDataParams(const PluginPrimitiveMeshParams& params)
{
	BackendPrimitiveGeometry::PrimitiveMeshParams out;
	out.kind = toDataKind(params.kind);
	out.lengthMm = params.lengthMm;
	out.widthMm = params.widthMm;
	out.heightMm = params.heightMm;
	out.radiusMm = params.radiusMm;
	out.radiusTopMm = params.radiusTopMm;
	return out;
}

BackendPrimitiveGeometry::PrimitiveMeshQuality toDataQuality(const PluginPrimitiveMeshQuality& quality)
{
	BackendPrimitiveGeometry::PrimitiveMeshQuality out;
	out.segments = quality.segments;
	out.rings = quality.rings;
	return out;
}

void transformSoupByMat4(std::vector<float>& soup, const BackendMat4& M)
{
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3U)
	{
		const double x = static_cast<double>(soup[i]);
		const double y = static_cast<double>(soup[i + 1]);
		const double z = static_cast<double>(soup[i + 2]);
		soup[i] = static_cast<float>(M.v[0] * x + M.v[4] * y + M.v[8] * z + M.v[12]);
		soup[i + 1] = static_cast<float>(M.v[1] * x + M.v[5] * y + M.v[9] * z + M.v[13]);
		soup[i + 2] = static_cast<float>(M.v[2] * x + M.v[6] * y + M.v[10] * z + M.v[14]);
	}
}

BackendMat4 placementWorldMatrix(const PluginMeshCreateOptions& placement)
{
	const BackendVec3 pose{placement.poseMm.x, placement.poseMm.y, placement.poseMm.z};
	const BackendVec3 rot{placement.rotationDeg.x, placement.rotationDeg.y, placement.rotationDeg.z};
	return backend_world_mat_from_pose(pose, rot);
}

std::vector<float> worldTriangleSoupForMesh(const MeshBackendData& mesh, BackendDataManager& mgr)
{
	std::vector<float> soup = mesh.triangleSoup();
	if (soup.empty())
		return soup;
	const BackendMat4 world = mesh.worldMatrix(&mgr);
	transformSoupByMat4(soup, world);
	return soup;
}

MeshBooleanOp toMeshBooleanOp(PluginMeshBooleanOp op)
{
	switch (op)
	{
	case PluginMeshBooleanOp::Union:
		return MeshBooleanOp::Union;
	case PluginMeshBooleanOp::Intersection:
		return MeshBooleanOp::Intersection;
	case PluginMeshBooleanOp::Difference:
	default:
		return MeshBooleanOp::Difference;
	}
}
} // namespace

PluginHostContext::PluginHostContext(IPluginMainWindowHost* mainWindowHost, QObject* parent)
	: QObject(parent), m_mainWindowHost(mainWindowHost),
	  m_pointCloudHost(std::make_unique<PluginPointCloudHostImpl>(this)),
	  m_geometryHost(std::make_unique<PluginGeometryHostImpl>(this)),
	  m_labelingHost(std::make_unique<PluginLabelingHostImpl>(this)),
	  m_aiHost(std::make_unique<AiAssistantHostImpl>(this))
{
}

PluginHostContext::~PluginHostContext() = default;

void PluginHostContext::attachDocumentTabSignals()
{
	if (!m_mainWindowHost || !m_mainWindowHost->documentTabs())
	{
		return;
	}
	QTabWidget* tabs = m_mainWindowHost->documentTabs();
	connect(tabs, &QTabWidget::currentChanged, this,
			[this](int)
			{
				refreshDocumentAdapters();
				IPluginDocument* doc = activeDocument();
				for (const auto& cb : m_docChangeCallbacks)
				{
					if (cb)
					{
						cb(doc);
					}
				}
			});
	refreshDocumentAdapters();
}

void PluginHostContext::refreshDocumentAdapters()
{
	m_documents.clear();
	if (!m_mainWindowHost || !m_mainWindowHost->documentTabs())
	{
		return;
	}
	QTabWidget* tabs = m_mainWindowHost->documentTabs();
	for (int i = 0; i < tabs->count(); ++i)
	{
		if (auto* host = m_mainWindowHost->documentHostAt(i))
		{
			m_documents.push_back(std::make_unique<PluginDocumentAdapter>(host));
		}
	}
}

unsigned int PluginHostContext::hostVersion() const
{
	return cloudsimPluginHostVersion();
}

QString PluginHostContext::applicationDirPath() const
{
	return QCoreApplication::applicationDirPath();
}

void PluginHostContext::logInfo(const QString& message) const
{
	RunLogger::info(message.toStdString());
}

void PluginHostContext::logWarn(const QString& message) const
{
	RunLogger::warn(message.toStdString());
}

void PluginHostContext::logError(const QString& message) const
{
	RunLogger::error(message.toStdString());
}

int PluginHostContext::documentCount() const
{
	return m_mainWindowHost ? m_mainWindowHost->documentTabCount() : 0;
}

IPluginDocument* PluginHostContext::activeDocument()
{
	if (!m_mainWindowHost)
	{
		return nullptr;
	}
	cloudsim::host::DocumentHost* host = m_mainWindowHost->currentDocumentHost();
	if (!host)
	{
		return nullptr;
	}
	for (const auto& ad : m_documents)
	{
		if (ad && ad->documentHost() == host)
		{
			return ad.get();
		}
	}
	return nullptr;
}

const IPluginDocument* PluginHostContext::activeDocument() const
{
	return const_cast<PluginHostContext*>(this)->activeDocument();
}

IPluginDocument* PluginHostContext::documentAt(int index)
{
	if (index < 0 || index >= static_cast<int>(m_documents.size()))
	{
		return nullptr;
	}
	return m_documents[static_cast<std::size_t>(index)].get();
}

const IPluginDocument* PluginHostContext::documentAt(int index) const
{
	return const_cast<PluginHostContext*>(this)->documentAt(index);
}

void PluginHostContext::onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback)
{
	if (callback)
	{
		m_docChangeCallbacks.push_back(std::move(callback));
	}
}

void PluginHostContext::invokeOnUiThread(std::function<void()> fn)
{
	if (!fn)
	{
		return;
	}
	if (QThread::currentThread() == thread())
	{
		fn();
		return;
	}
	QMetaObject::invokeMethod(
		this, [fn]() { fn(); }, Qt::QueuedConnection);
}

void PluginHostContext::enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
								   std::function<void(bool threw, const QString& throwMessage)> onFinished)
{
	if (!m_mainWindowHost)
	{
		if (onFinished)
		{
			onFinished(true, QStringLiteral("JobSystem not available"));
		}
		return;
	}
	m_mainWindowHost->enqueueBackgroundJob(title, std::move(work), std::move(onFinished));
}

QDockWidget* PluginHostContext::registerDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area)
{
	if (!m_mainWindowHost || !widget)
	{
		return nullptr;
	}
	if (area == Qt::RightDockWidgetArea)
	{
		logWarn(QStringLiteral(
			"registerDockWidget(Right) is deprecated; rebuild the plugin and call registerSidePanelTab."));
		QTabWidget* tabs = m_mainWindowHost ? m_mainWindowHost->rightPanelTabs() : nullptr;
		if (tabs && tabs->indexOf(widget) >= 0)
		{
			return nullptr;
		}
		const QByteArray titleUtf8 = title.toUtf8();
		(void)registerSidePanelTab(titleUtf8.constData(), widget);
		return nullptr;
	}
	auto* dock = m_mainWindowHost->addPluginDockWidget(title, widget, area);
	if (dock)
	{
		m_ownedDocks.push_back(dock);
	}
	return dock;
}

QWidget* PluginHostContext::sidePanelTabParent() const
{
	return m_mainWindowHost ? m_mainWindowHost->rightPanelTabs() : nullptr;
}

int PluginHostContext::registerSidePanelTab(const char* titleUtf8, QWidget* widget)
{
	if (!m_mainWindowHost || !widget || !titleUtf8)
	{
		return -1;
	}
	QTabWidget* tabs = m_mainWindowHost->rightPanelTabs();
	if (tabs && reinterpret_cast<const void*>(titleUtf8) == static_cast<const void*>(tabs))
	{
		logError(QStringLiteral(
			"registerSidePanelTab: invalid title pointer (plugin/host ABI mismatch — rebuild the plugin)."));
		return -1;
	}
	return m_mainWindowHost->addPluginSidePanelTab(QString::fromUtf8(titleUtf8), widget);
}

void PluginHostContext::unregisterSidePanelTab(QWidget* widget)
{
	if (!m_mainWindowHost || !widget)
	{
		return;
	}
	m_mainWindowHost->removePluginSidePanelTab(widget);
}

QMenu* PluginHostContext::registerMenuPath(const QStringList& path)
{
	if (!m_mainWindowHost || path.isEmpty())
	{
		return nullptr;
	}
	QMenuBar* bar = m_mainWindowHost->menuBar();
	if (!bar)
	{
		return nullptr;
	}
	QMenu* current = nullptr;
	for (const QString& segment : path)
	{
		QMenu* found = nullptr;
		if (!current)
		{
			for (QAction* action : bar->actions())
			{
				if (action && action->menu() && action->menu()->title() == segment)
				{
					found = action->menu();
					break;
				}
			}
			if (!found)
			{
				found = bar->addMenu(segment);
			}
		}
		else
		{
			for (QAction* action : current->actions())
			{
				if (action && action->menu() && action->menu()->title() == segment)
				{
					found = action->menu();
					break;
				}
			}
			if (!found)
			{
				found = current->addMenu(segment);
			}
		}
		current = found;
	}
	return current;
}

QAction* PluginHostContext::registerAction(QMenu* menu, const QString& text, std::function<void()> handler)
{
	if (!menu || !handler)
	{
		return nullptr;
	}
	QAction* action = menu->addAction(text);
	if (QObject* parent = m_mainWindowHost->pluginActionParent())
	{
		QObject::connect(action, &QAction::triggered, parent, [handler]() { handler(); });
	}
	return action;
}

bool PluginHostContext::createPrimitiveMesh(const PluginPrimitiveMeshParams& params,
											const PluginPrimitiveMeshQuality& quality,
											const PluginMeshCreateOptions& options, QString* outError,
											QString* outBackendId)
{
	(void)quality;
	if (!m_mainWindowHost)
	{
		if (outError)
			*outError = QStringLiteral("MainWindow not available.");
		return false;
	}
	cloudsim::host::DocumentHost* doc = m_mainWindowHost->currentDocumentHost();
	if (!doc)
	{
		if (outError)
			*outError = QStringLiteral("No active document.");
		return false;
	}

	geoalgo::PrimitiveBrepParams bp;
	switch (params.kind)
	{
	case PluginPrimitiveKind::Cylinder:
		bp.kind = geoalgo::PrimitiveBrepKind::Cylinder;
		break;
	case PluginPrimitiveKind::Cone:
		bp.kind = geoalgo::PrimitiveBrepKind::Cone;
		break;
	case PluginPrimitiveKind::Sphere:
		bp.kind = geoalgo::PrimitiveBrepKind::Sphere;
		break;
	case PluginPrimitiveKind::Box:
	default:
		bp.kind = geoalgo::PrimitiveBrepKind::Box;
		break;
	}
	bp.lengthMm = params.lengthMm;
	bp.widthMm = params.widthMm;
	bp.heightMm = params.heightMm;
	bp.radiusMm = params.radiusMm;
	bp.radiusTopMm = params.radiusTopMm;

	// 注册 BrepModel，才能走线面特征 / 轨迹工件解析
	geoalgo::ShapeHandle shape = geoalgo::makePrimitiveShape(bp);
	if (shape.isNull())
	{
		if (outError)
			*outError = QStringLiteral("Failed to build primitive B-rep.");
		return false;
	}

	const QString displayName = options.displayName.isEmpty() ? QStringLiteral("PluginPrimitive") : options.displayName;
	// 每实例唯一路径，避免轨迹页按 sourcePath 去重吞掉多个 AI 基本体
	const QString sourcePath = options.sourcePath.isEmpty()
								   ? QStringLiteral("ai://primitive/%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces))
								   : options.sourcePath;

	auto brep = std::make_shared<BrepBackendData>();
	brep->setName(displayName.toStdString());
	brep->setShape(std::move(shape));

	BackendVec3 pos;
	pos.x = options.poseMm.x;
	pos.y = options.poseMm.y;
	pos.z = options.poseMm.z;
	brep->setPose(pos);

	BackendVec3 rot;
	rot.x = options.rotationDeg.x;
	rot.y = options.rotationDeg.y;
	rot.z = options.rotationDeg.z;
	brep->setRotation(rot);

	QString regErr;
	if (!cloudsim::host::registerAdoptedBrepAndLoadScene(*doc, brep, sourcePath, QStringLiteral("BrepModel"), QString(),
														options.resetViewToHome, &regErr))
	{
		if (outError)
			*outError = regErr.isEmpty() ? QStringLiteral("Failed to register B-rep in backend.") : regErr;
		return false;
	}
	if (outBackendId)
		*outBackendId = QString::fromStdString(brep->id());
	if (options.selectInTree)
		m_mainWindowHost->focusBackendInTree(brep->id());
	return true;
}

bool PluginHostContext::buildPrimitiveMeshSoup(const PluginPrimitiveMeshParams& params,
											   const PluginPrimitiveMeshQuality& quality,
											   const PluginMeshCreateOptions& placement,
											   std::vector<float>& outWorldSoup, QString* outError)
{
	outWorldSoup.clear();
	std::vector<float> soup =
		BackendPrimitiveGeometry::makePrimitiveTriangleSoup(toDataParams(params), toDataQuality(quality));
	if (soup.empty())
	{
		if (outError)
			*outError = QStringLiteral("Primitive mesh soup is empty.");
		return false;
	}
	const BackendMat4 world = placementWorldMatrix(placement);
	transformSoupByMat4(soup, world);
	outWorldSoup = std::move(soup);
	return true;
}

bool PluginHostContext::booleanSoupsAndRegister(const std::vector<float>& targetWorldSoup,
												const std::vector<float>& toolWorldSoup, PluginMeshBooleanOp op,
												const PluginBooleanMeshOptions& options,
												std::string* outResultBackendId, QString* outError)
{
	if (outResultBackendId)
		outResultBackendId->clear();
	if (targetWorldSoup.empty() || toolWorldSoup.empty())
	{
		if (outError)
			*outError = QStringLiteral("Target or tool soup is empty.");
		return false;
	}
	std::vector<float> resultSoup;
	std::string boolErr;
	if (!MeshBoolean::compute(targetWorldSoup, toolWorldSoup, toMeshBooleanOp(op), resultSoup, &boolErr))
	{
		if (outError)
			*outError = QString::fromStdString(boolErr);
		return false;
	}
	PluginMeshCreateOptions regOpt;
	regOpt.displayName = options.resultName.isEmpty() ? QStringLiteral("BooleanResult") : options.resultName;
	regOpt.sourcePath = QStringLiteral("ai://boolean");
	regOpt.selectInTree = options.selectInTree;
	regOpt.resetViewToHome = options.resetViewToHome;
	regOpt.poseMm = {};
	regOpt.rotationDeg = {};
	QString regErr;
	QString resultId;
	if (!registerMeshFromSoup(std::move(resultSoup), regOpt, &regErr, &resultId))
	{
		if (outError)
			*outError = regErr;
		return false;
	}
	if (outResultBackendId)
		*outResultBackendId = resultId.toStdString();
	return true;
}

bool PluginHostContext::booleanMeshSoups(PluginMeshBooleanOp op, const std::vector<float>& targetWorldSoup,
										 const std::vector<float>& toolWorldSoup,
										 const PluginBooleanMeshOptions& options, std::string* outResultBackendId,
										 QString* outError)
{
	if (!m_mainWindowHost)
	{
		if (outError)
			*outError = QStringLiteral("MainWindow not available.");
		return false;
	}
	if (!m_mainWindowHost->currentDocumentHost())
	{
		if (outError)
			*outError = QStringLiteral("No active document.");
		return false;
	}
	return booleanSoupsAndRegister(targetWorldSoup, toolWorldSoup, op, options, outResultBackendId, outError);
}

bool PluginHostContext::booleanPrimitiveMeshes(PluginMeshBooleanOp op, const PluginPrimitiveMeshParams& targetParams,
											   const PluginPrimitiveMeshQuality& targetQuality,
											   const PluginMeshCreateOptions& targetPlacement,
											   const PluginPrimitiveMeshParams& toolParams,
											   const PluginPrimitiveMeshQuality& toolQuality,
											   const PluginMeshCreateOptions& toolPlacement,
											   const PluginBooleanMeshOptions& options, std::string* outResultBackendId,
											   QString* outError)
{
	std::vector<float> targetSoup;
	std::vector<float> toolSoup;
	if (!buildPrimitiveMeshSoup(targetParams, targetQuality, targetPlacement, targetSoup, outError))
		return false;
	if (!buildPrimitiveMeshSoup(toolParams, toolQuality, toolPlacement, toolSoup, outError))
		return false;
	return booleanMeshSoups(op, targetSoup, toolSoup, options, outResultBackendId, outError);
}

bool PluginHostContext::booleanMesh(PluginMeshBooleanOp op, const std::string& targetBackendId,
									const std::string& toolBackendId, const PluginBooleanMeshOptions& options,
									std::string* outResultBackendId, QString* outError)
{
	if (outResultBackendId)
		outResultBackendId->clear();
	if (!m_mainWindowHost)
	{
		if (outError)
			*outError = QStringLiteral("MainWindow not available.");
		return false;
	}
	cloudsim::host::DocumentHost* doc = m_mainWindowHost->currentDocumentHost();
	if (!doc)
	{
		if (outError)
			*outError = QStringLiteral("No active document.");
		return false;
	}
	BackendDataManager& mgr = doc->backend();
	const auto targetData = mgr.getData(targetBackendId);
	const auto toolData = mgr.getData(toolBackendId);
	const auto targetMesh = std::dynamic_pointer_cast<MeshBackendData>(targetData);
	const auto toolMesh = std::dynamic_pointer_cast<MeshBackendData>(toolData);
	if (!targetMesh || !toolMesh || targetMesh->triangleSoup().empty() || toolMesh->triangleSoup().empty())
	{
		if (outError)
			*outError = QStringLiteral("Target or tool mesh not found or has no geometry.");
		return false;
	}
	const std::vector<float> targetSoup = worldTriangleSoupForMesh(*targetMesh, mgr);
	const std::vector<float> toolSoup = worldTriangleSoupForMesh(*toolMesh, mgr);
	if (!booleanSoupsAndRegister(targetSoup, toolSoup, op, options, outResultBackendId, outError))
		return false;
	if (options.hideOperands)
	{
		std::vector<std::string> hideIds;
		hideIds.push_back(targetBackendId);
		hideIds.push_back(toolBackendId);
		doc->sceneFacade().setBackendsVisible(hideIds, false);
	}
	return true;
}

bool PluginHostContext::registerTriangleMesh(const std::vector<float>& triangleSoup,
											 const PluginMeshCreateOptions& options, QString* outError)
{
	// 异常长度多为插件未随宿主 1.4.0 重编译导致 vtable 错位
	constexpr std::size_t kMaxSoupFloats = 50'000'000U;
	if (triangleSoup.size() > kMaxSoupFloats)
	{
		if (outError)
		{
			*outError = QStringLiteral("Triangle soup size invalid. Rebuild plugins after upgrading host to 1.4.0.");
		}
		return false;
	}
	if (triangleSoup.empty() || (triangleSoup.size() % 9U) != 0U)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid triangle soup (need 9 floats per triangle).");
		}
		return false;
	}
	return registerMeshFromSoup(std::vector<float>(triangleSoup), options, outError, nullptr);
}

bool PluginHostContext::registerMeshFromSoup(std::vector<float> soup, const PluginMeshCreateOptions& options,
											 QString* outError, QString* outBackendId)
{
	if (!m_mainWindowHost)
	{
		if (outError)
		{
			*outError = QStringLiteral("MainWindow not available.");
		}
		return false;
	}
	cloudsim::host::DocumentHost* doc = m_mainWindowHost->currentDocumentHost();
	if (!doc)
	{
		if (outError)
		{
			*outError = QStringLiteral("No active document.");
		}
		return false;
	}

	const QString displayName = options.displayName.isEmpty() ? QStringLiteral("PluginMesh") : options.displayName;
	const QString sourcePath = options.sourcePath.isEmpty() ? QStringLiteral("plugin://mesh") : options.sourcePath;

	auto mesh = std::make_shared<MeshBackendData>();
	mesh->setName(displayName.toStdString());
	mesh->setTriangleSoup(std::move(soup));

	BackendVec3 pos;
	pos.x = options.poseMm.x;
	pos.y = options.poseMm.y;
	pos.z = options.poseMm.z;
	mesh->setPose(pos);

	BackendVec3 rot;
	rot.x = options.rotationDeg.x;
	rot.y = options.rotationDeg.y;
	rot.z = options.rotationDeg.z;
	mesh->setRotation(rot);

	cloudsim::host::AdoptMeshOptions adoptOpt;
	adoptOpt.sourcePath = sourcePath;
	adoptOpt.catalogTypeName = QStringLiteral("Model");
	adoptOpt.resetViewToHome = options.resetViewToHome;
	QString regErr;
	const cloudsim::host::AdoptRegistrationResult adopted =
		cloudsim::host::registerAdoptedMesh(*doc, mesh, adoptOpt, &regErr);
	if (!adopted.ok)
	{
		if (outError)
		{
			*outError = regErr.isEmpty() ? QStringLiteral("Failed to register mesh in backend.") : regErr;
		}
		return false;
	}
	if (outBackendId)
		*outBackendId = adopted.backendId;
	if (options.selectInTree)
	{
		m_mainWindowHost->focusBackendInTree(mesh->id());
	}
	return true;
}

std::string PluginHostContext::importFileIntoActiveDocument(const std::string& pathUtf8, const bool isPointCloud,
															std::string* outError)
{
	if (!m_mainWindowHost)
	{
		if (outError)
		{
			*outError = "MainWindow not available";
		}
		return std::string();
	}
	cloudsim::host::DocumentHost* doc = m_mainWindowHost->currentDocumentHost();
	if (!doc)
	{
		if (outError)
		{
			*outError = "No active document";
		}
		return std::string();
	}
	const QString path = QString::fromStdString(pathUtf8);
	if (path.isEmpty())
	{
		if (outError)
		{
			*outError = "Empty path";
		}
		return std::string();
	}
	cloudsim::core::ImportOptionsDto opt;
	opt.quietUi = true;
	opt.resetViewToHome = false;
	opt.catalogTypeName = isPointCloud ? QStringLiteral("PointCloud") : QStringLiteral("Model");
	opt.isPointCloud = isPointCloud;
	QString importErr;
	const cloudsim::host::ImportFileKind kind =
		isPointCloud ? cloudsim::host::ImportFileKind::PointCloud : cloudsim::host::ImportFileKind::Mesh;
	const cloudsim::host::ImportFileResult imported =
		cloudsim::host::importFileIntoDocument(*doc, path, kind, opt, &importErr);
	if (!imported.ok)
	{
		if (outError)
		{
			*outError = importErr.toStdString();
		}
		return std::string();
	}
	return imported.rootBackendId.toStdString();
}

IPluginPointCloudHost* PluginHostContext::pointCloudHost()
{
	return m_pointCloudHost.get();
}

const IPluginPointCloudHost* PluginHostContext::pointCloudHost() const
{
	return m_pointCloudHost.get();
}

IPluginGeometryHost* PluginHostContext::geometryHost()
{
	return m_geometryHost.get();
}

const IPluginGeometryHost* PluginHostContext::geometryHost() const
{
	return m_geometryHost.get();
}

IPluginLabelingHost* PluginHostContext::labelingHost()
{
	return m_labelingHost.get();
}

const IPluginLabelingHost* PluginHostContext::labelingHost() const
{
	return m_labelingHost.get();
}

QString PluginHostContext::selectedBackendId() const
{
	return m_mainWindowHost ? m_mainWindowHost->selectedBackendId() : QString();
}

IAiAssistantHost* PluginHostContext::aiAssistantHost()
{
	return m_aiHost.get();
}

const IAiAssistantHost* PluginHostContext::aiAssistantHost() const
{
	return m_aiHost.get();
}

bool PluginHostContext::captureActiveViewportPng(QByteArray& outPng, QString* outError)
{
	if (!m_mainWindowHost)
	{
		if (outError)
			*outError = QStringLiteral("主窗口未就绪");
		return false;
	}
	cloudsim::host::DocumentHost* host = m_mainWindowHost->currentDocumentHost();
	if (!host)
	{
		if (outError)
		{
			*outError = QStringLiteral("请先打开含 3D 视口的文档");
		}
		return false;
	}
	return host->render().captureViewportPng(outPng, outError);
}

bool PluginHostContext::resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath, QString* outError)
{
	if (!m_mainWindowHost)
	{
		if (outError)
		{
			*outError = QStringLiteral("主窗口未就绪");
		}
		return false;
	}
	if (!m_mainWindowHost->resolveTrajectoryWorkpieceForAi(&outBackendId, &outStepPath))
	{
		if (outError)
		{
			*outError = QStringLiteral("请先在轨迹生成页选择 STEP 工件");
		}
		return false;
	}
	return true;
}

bool PluginHostContext::buildTrajectoryFeatureCatalogSlice(const QString& backendId, const QString& stepPathUtf8,
														   const QString& userText, QByteArray& outFullCatalogUtf8,
														   QByteArray& outSliceUtf8, QString* outError)
{
	outFullCatalogUtf8.clear();
	outSliceUtf8.clear();
	if (backendId.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("工件无效");
		}
		return false;
	}
	cloudsim::host::DocumentHost* page = m_mainWindowHost ? m_mainWindowHost->currentDocumentHost() : nullptr;
	if (!page)
	{
		if (outError)
		{
			*outError = QStringLiteral("无活动文档");
		}
		return false;
	}
	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef wp;
	std::string err;
	const geometry_backend_ops::WorkpieceShapeSource src = geometry_backend_ops::resolveWorkpieceShape(
		backendId.toStdString(), page->backend(), stepPathUtf8.toStdString(), shape, wp, &err);
	if (src == geometry_backend_ops::WorkpieceShapeSource::Unavailable)
	{
		if (outError)
		{
			*outError = err.empty() ? QStringLiteral("无法解析工件 B-rep") : QString::fromStdString(err);
		}
		return false;
	}
	geoalgo::FeatureCatalog catalog;
	if (!geometry_backend_ops::enumerateFeatureCatalog(wp, shape, catalog, &err))
	{
		if (outError)
		{
			*outError = QString::fromStdString(err);
		}
		return false;
	}
	outFullCatalogUtf8 = QByteArray::fromStdString(geometry_backend_ops::featureCatalogToJson(catalog));
	const AiFeatureAxis axis = AiTrajectoryFeatureCatalog::inferFeatureAxisFromText(userText);
	outSliceUtf8 = AiTrajectoryFeatureCatalog::buildCatalogSliceJson(catalog, axis);
	return true;
}

bool PluginHostContext::showAiFeatureCandidatePreview(const QByteArray& previewJsonUtf8, QString* outError)
{
	if (!m_mainWindowHost)
	{
		if (outError)
		{
			*outError = QStringLiteral("主窗口未就绪");
		}
		return false;
	}
	return m_mainWindowHost->showAiFeatureCandidatePreviewForAi(
		std::string(previewJsonUtf8.constData(), static_cast<std::size_t>(previewJsonUtf8.size())), outError);
}

void PluginHostContext::clearAiFeatureCandidatePreview()
{
	if (m_mainWindowHost)
	{
		m_mainWindowHost->clearAiFeatureCandidatePreviewForAi();
	}
}

bool PluginHostContext::commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* outSummary,
												   QString* outError)
{
	if (!m_mainWindowHost)
	{
		if (outError)
		{
			*outError = QStringLiteral("主窗口未就绪");
		}
		return false;
	}
	return m_mainWindowHost->commitAiTrajectoryFeaturesForAi(
		std::string(featurePlanJsonUtf8.constData(), static_cast<std::size_t>(featurePlanJsonUtf8.size())), outSummary,
		outError);
}

bool PluginHostContext::useChinese() const
{
	return m_mainWindowHost && m_mainWindowHost->useChinese();
}

void PluginHostContext::onLanguageChanged(std::function<void(bool useChinese)> callback)
{
	if (callback)
	{
		m_languageCallbacks.push_back(std::move(callback));
	}
}

void PluginHostContext::setSidePanelTabTitle(QWidget* widget, const char* titleUtf8)
{
	if (!m_mainWindowHost || !widget || !titleUtf8)
	{
		return;
	}
	m_mainWindowHost->setPluginSidePanelTabTitle(widget, QString::fromUtf8(titleUtf8));
}

void PluginHostContext::notifyLanguageChanged()
{
	const bool chinese = useChinese();
	for (const auto& cb : m_languageCallbacks)
	{
		if (cb)
		{
			cb(chinese);
		}
	}
}

bool PluginHostContext::registerBackendType(const PluginBackendMeta& meta, QString* outError)
{
	if (meta.className.empty() || !meta.factory)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid PluginBackendMeta (className/factory required).");
		}
		return false;
	}

	BackendMeta reg;
	reg.className = meta.className;
	reg.displayName = meta.displayName.empty() ? meta.className : meta.displayName;
	reg.supportsTransform = meta.supportsTransform;
	reg.supportsVisibility = meta.supportsVisibility;
	reg.factory = [factory = meta.factory]() -> std::shared_ptr<BackendDataBase>
	{
		const std::shared_ptr<IPluginBackendObject> delegate = factory();
		if (!delegate)
		{
			return nullptr;
		}
		return std::make_shared<PluginDelegatedBackend>(delegate);
	};
	if (meta.propertyRowsProvider)
	{
		reg.propertyEditorFactory = [provider = meta.propertyRowsProvider](BackendDataBase* base) -> void*
		{
			(void)base;
			(void)provider;
			return nullptr;
		};
	}

	BackendRegistry::instance().registerType(reg);
	return true;
}
