#include "PluginLabelingHostImpl.h"

#include "DocumentPointCloudOps.h"
#include "DocumentHost.h"
#include "LabelingSession.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PluginDocumentAdapter.h"
#include "IPluginPointCloudHost.h"
#include "PluginHostContext.h"
#include "PointCloudBackendData.h"
#include "PointCloudBackendOps.h"
#include "WidgetDocumentAccess.h"

#include <QMetaObject>
#include <QObject>
#include <QVector>

#include <algorithm>
#include <memory>

namespace
{

cloudsim::host::DocumentHost* pageFromDoc(IPluginDocument* doc)
{
	if (!doc)
	{
		return nullptr;
	}
	const auto* adapter = dynamic_cast<const PluginDocumentAdapter*>(doc);
	return adapter ? adapter->documentHost() : nullptr;
}

LabelingSessionConfig toLabelingConfig(const PluginLabelingSessionConfig& cfg)
{
	LabelingSessionConfig out;
	out.unlabeledClassId = cfg.unlabeledClassId;
	for (const PluginLabelingClassDef& c : cfg.classes)
	{
		LabelingClassDef d;
		d.classId = c.classId;
		d.nameUtf8 = c.nameUtf8;
		d.colorRgb[0] = c.colorRgb[0];
		d.colorRgb[1] = c.colorRgb[1];
		d.colorRgb[2] = c.colorRgb[2];
		out.classes.push_back(d);
	}
	return out;
}

LabelingDatasetExportOptions toExportOptions(const PluginLabelingDatasetExportOptions& o)
{
	LabelingDatasetExportOptions out;
	out.sampleNameUtf8 = o.sampleNameUtf8;
	out.numClasses = o.numClasses;
	out.meshSampleCount = o.meshSampleCount;
	return out;
}

PluginLabelingDatasetExportResult toPluginExportResult(const LabelingDatasetExportResult& r)
{
	PluginLabelingDatasetExportResult out;
	out.ok = r.ok;
	out.plyRelativePath = r.plyRelativePath;
	out.labelRelativePath = r.labelRelativePath;
	out.datasetJsonlPath = r.datasetJsonlPath;
	return out;
}

} // namespace

PluginLabelingHostImpl::PluginLabelingHostImpl(PluginHostContext* hostContext)
	: m_host(hostContext)
{
}

PluginLabelingHostImpl::SessionEntry* PluginLabelingHostImpl::findSession(const PluginLabelingSessionId sessionId)
{
	const auto it = m_sessions.find(sessionId);
	return it != m_sessions.end() ? &it->second : nullptr;
}

const PluginLabelingHostImpl::SessionEntry* PluginLabelingHostImpl::findSession(const PluginLabelingSessionId sessionId) const
{
	const auto it = m_sessions.find(sessionId);
	return it != m_sessions.end() ? &it->second : nullptr;
}

PluginLabelingSessionId PluginLabelingHostImpl::beginLabelingSession(
	IPluginDocument* doc,
	const std::string& backendIdUtf8,
	const PluginLabelingSessionConfig& config,
	QString* outError)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page)
	{
		if (outError)
		{
			*outError = QStringLiteral("No active document");
		}
		return kInvalidLabelingSessionId;
	}

	std::string resolveErr;
	SessionEntry entry;
	entry.doc = doc;
	entry.backendId = backendIdUtf8;
	entry.session = std::make_unique<LabelingSession>();
	const LabelingSessionConfig lc = toLabelingConfig(config);

	if (const auto pc = document_point_cloud_ops::resolvePointCloud(page, backendIdUtf8, &resolveErr))
	{
		if (!entry.session->beginPointCloud(pc->pointPositionsXyz(), lc))
		{
			if (outError)
			{
				*outError = QStringLiteral("Failed to init point cloud session");
			}
			return kInvalidLabelingSessionId;
		}
		entry.kind = PluginLabelingGeometryKind::PointCloud;
	}
	else if (const auto mesh = document_point_cloud_ops::resolveMesh(page, backendIdUtf8, &resolveErr))
	{
		if (!entry.session->beginTriangleMesh(mesh->triangleSoup(), lc))
		{
			if (outError)
			{
				*outError = QStringLiteral("Failed to init mesh session");
			}
			return kInvalidLabelingSessionId;
		}
		entry.kind = PluginLabelingGeometryKind::TriangleMesh;
	}
	else
	{
		if (outError)
		{
			*outError = QString::fromStdString(resolveErr.empty() ? "Unsupported backend" : resolveErr);
		}
		return kInvalidLabelingSessionId;
	}

	const PluginLabelingSessionId id = m_nextSessionId.fetch_add(1U);
	entry.id = id;
	m_sessions[id] = std::move(entry);

	OsgWidget* osg = widgetOsgFromPage(page);
	if (osg)
	{
		osg->syncSelectionForBackendId(backendIdUtf8);
	}

	QString syncErr;
	(void)syncLabelVisualization(id, &syncErr);
	return id;
}

void PluginLabelingHostImpl::clearLabelingSession(const PluginLabelingSessionId sessionId)
{
	if (m_activePickSessionId == sessionId)
	{
		abandonActiveLabelingPick();
	}
	m_sessions.erase(sessionId);
	if (m_activePickSessionId == sessionId)
	{
		m_activePickSessionId = 0U;
	}
}

bool PluginLabelingHostImpl::getSessionSummary(
	const PluginLabelingSessionId sessionId,
	PluginLabelingSessionSummary& outSummary,
	QString* outError) const
{
	const SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid labeling session");
		}
		return false;
	}
	outSummary.geometryKind = entry->kind;
	outSummary.totalElements = entry->session->totalElements();
	outSummary.labeledElements = entry->session->labeledCount();
	outSummary.activeClassId = entry->session->activeClassId();
	const auto hist = entry->session->classHistogram();
	for (const auto& kv : hist)
	{
		outSummary.classHistogram[kv.first] = kv.second;
	}
	return true;
}

bool PluginLabelingHostImpl::setActiveClass(const PluginLabelingSessionId sessionId, const int classId, QString* outError)
{
	SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid labeling session");
		}
		return false;
	}
	entry->session->setActiveClassId(classId);
	return true;
}

bool PluginLabelingHostImpl::syncSessionConfig(
	const PluginLabelingSessionId sessionId,
	const PluginLabelingSessionConfig& config,
	QString* outError)
{
	SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid labeling session");
		}
		return false;
	}
	entry->session->updateSessionConfig(toLabelingConfig(config));
	return refreshBackendColors(*entry, outError);
}

bool PluginLabelingHostImpl::applyLabels(
	const PluginLabelingSessionId sessionId,
	const PluginLabelingSelectionResult& selection,
	const int classId,
	const bool erase,
	QString* outError)
{
	SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid labeling session");
		}
		return false;
	}
	const int cid = erase ? entry->session->config().unlabeledClassId : classId;
	bool ok = false;
	if (entry->kind == PluginLabelingGeometryKind::PointCloud)
	{
		ok = entry->session->applyPointLabels(selection.pointIndices, cid, erase);
	}
	else
	{
		ok = entry->session->applyTriangleLabels(selection.triangleIndices, cid, erase);
	}
	if (!ok)
	{
		return true;
	}
	return refreshBackendColors(*entry, outError);
}

bool PluginLabelingHostImpl::undo(const PluginLabelingSessionId sessionId, QString* outError)
{
	SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session || !entry->session->undo())
	{
		if (outError)
		{
			*outError = QStringLiteral("Nothing to undo");
		}
		return false;
	}
	return refreshBackendColors(*entry, outError);
}

bool PluginLabelingHostImpl::redo(const PluginLabelingSessionId sessionId, QString* outError)
{
	SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session || !entry->session->redo())
	{
		if (outError)
		{
			*outError = QStringLiteral("Nothing to redo");
		}
		return false;
	}
	return refreshBackendColors(*entry, outError);
}

bool PluginLabelingHostImpl::refreshBackendColors(SessionEntry& entry, QString* outError)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(entry.doc);
	if (!page)
	{
		if (outError)
		{
			*outError = QStringLiteral("Document unavailable");
		}
		return false;
	}
	if (entry.kind == PluginLabelingGeometryKind::PointCloud)
	{
		const auto pc = document_point_cloud_ops::resolvePointCloud(page, entry.backendId, nullptr);
		if (!pc)
		{
			return false;
		}
		std::vector<float> rgba;
		entry.session->buildPointCloudRgba(rgba);
		pc->setPointBuffers(pc->pointPositionsXyz(), rgba, pc->pointNormalsNxNyNz());
		document_point_cloud_ops::commitPointCloudVisual(page, *pc);
		return true;
	}
	const auto mesh = document_point_cloud_ops::resolveMesh(page, entry.backendId, nullptr);
	if (!mesh)
	{
		return false;
	}
	std::vector<float> rgb;
	entry.session->buildMeshVertexRgb(rgb);
	mesh->setTriangleSoupWithVertexColors(mesh->triangleSoup(), rgb);
	OsgWidget* osg = widgetOsgFromPage(page);
	if (osg)
	{
		QString err;
		(void)osg->loadMeshFromBackendData(*mesh, &err, false, true, true);
	}
	return true;
}

bool PluginLabelingHostImpl::syncLabelVisualization(const PluginLabelingSessionId sessionId, QString* outError)
{
	SessionEntry* entry = findSession(sessionId);
	if (!entry)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid labeling session");
		}
		return false;
	}
	return refreshBackendColors(*entry, outError);
}

bool PluginLabelingHostImpl::importPerPointLabels(
	const PluginLabelingSessionId sessionId,
	const std::vector<int>& labels,
	const int numClasses,
	QString* outError)
{
	SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid labeling session");
		}
		return false;
	}
	if (!entry->session->importPointLabels(labels, numClasses))
	{
		if (outError)
		{
			*outError = QStringLiteral("Label count mismatch");
		}
		return false;
	}
	return refreshBackendColors(*entry, outError);
}

bool PluginLabelingHostImpl::exportPointNetDataset(
	const PluginLabelingSessionId sessionId,
	const std::string& outputDirUtf8,
	const PluginLabelingDatasetExportOptions& options,
	PluginLabelingDatasetExportResult& outResult,
	QString* outError)
{
	const SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid labeling session");
		}
		return false;
	}
	LabelingDatasetExportResult r;
	std::string err;
	const bool ok = entry->session->exportPointNetDataset(outputDirUtf8, toExportOptions(options), r, &err);
	outResult = toPluginExportResult(r);
	if (!ok && outError)
	{
		*outError = QString::fromStdString(err);
	}
	return ok;
}

void PluginLabelingHostImpl::setPickCancelledNotifier(PluginLabelingPickCancelledFn notifier)
{
	m_pickCancelledNotifier = std::move(notifier);
}

void PluginLabelingHostImpl::clearActivePickState(const bool notify)
{
	if (!m_pickState)
	{
		return;
	}
	OsgWidget* widget = m_pickState->viewportWidget;
	if (widget)
	{
		widget->setLabelingClickPickMode(false, m_pickState->meshFace);
		widget->setLabelingBrushPickMode(false, m_pickState->meshFace, m_pickState->brushRadius);
	}
	QObject::disconnect(m_pickState->clickConn);
	QObject::disconnect(m_pickState->brushStrokeConn);
	QObject::disconnect(m_pickState->brushFinishConn);
	QObject::disconnect(m_pickState->cancelConn);
	if (m_pickState->brushFinished)
	{
		m_pickState->brushFinished(false, QString(), {});
		m_pickState->brushFinished = nullptr;
	}
	m_pickState.reset();
	m_activePickSessionId = 0U;
	if (notify && m_pickCancelledNotifier)
	{
		m_pickCancelledNotifier();
	}
}

void PluginLabelingHostImpl::cancelActiveLabelingPick()
{
	clearActivePickState(true);
}

void PluginLabelingHostImpl::abandonActiveLabelingPick()
{
	clearActivePickState(false);
}

void PluginLabelingHostImpl::pickPointsOnce(const PluginLabelingSessionId sessionId, PluginLabelingPickFinishedFn onFinished)
{
	if (!onFinished)
	{
		return;
	}
	const SessionEntry* entry = findSession(sessionId);
	if (!entry)
	{
		onFinished(false, QStringLiteral("Invalid session"), {});
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(entry->doc);
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg || !m_host)
	{
		onFinished(false, QStringLiteral("Viewport unavailable"), {});
		return;
	}

	abandonActiveLabelingPick();
	m_pickState = std::make_unique<ActivePickState>();
	m_pickState->viewportWidget = osg;
	m_pickState->meshFace = false;
	m_pickState->sessionId = sessionId;
	m_activePickSessionId = sessionId;
	osg->setLabelingClickPickMode(true, false);

	m_pickState->clickConn = QObject::connect(
		osg,
		&OsgWidget::labelingClickCommitted,
		m_host,
		[=](const PickResult& pick) {
			PluginLabelingSelectionResult sel;
			if (pick.hit && pick.pointIndex >= 0)
			{
				sel.pointIndices.push_back(static_cast<std::size_t>(pick.pointIndex));
			}
			onFinished(pick.hit, pick.hit ? QString() : QStringLiteral("No point hit"), sel);
		});

	m_pickState->cancelConn = QObject::connect(
		osg,
		&OsgWidget::labelingPickCanceled,
		m_host,
		[this]() {
			clearActivePickState(true);
		});
}

void PluginLabelingHostImpl::brushStroke(
	const PluginLabelingSessionId sessionId,
	const float radiusPx,
	PluginLabelingBrushStrokeFn onStroke,
	PluginLabelingBrushFinishedFn onFinished)
{
	if (!onFinished)
	{
		return;
	}
	const SessionEntry* entry = findSession(sessionId);
	if (!entry)
	{
		onFinished(false, QStringLiteral("Invalid session"), {});
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(entry->doc);
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg || !m_host)
	{
		onFinished(false, QStringLiteral("Viewport unavailable"), {});
		return;
	}

	abandonActiveLabelingPick();
	m_pickState = std::make_unique<ActivePickState>();
	m_pickState->viewportWidget = osg;
	m_pickState->meshFace = false;
	m_pickState->brushRadius = radiusPx;
	m_pickState->sessionId = sessionId;
	m_pickState->brushFinished = onFinished;
	m_activePickSessionId = sessionId;
	osg->setLabelingBrushPickMode(true, false, radiusPx);

	m_pickState->brushStrokeConn = QObject::connect(
		osg,
		&OsgWidget::labelingBrushStroke,
		m_host,
		[=](const QVector<int>& indices) {
			if (!onStroke)
			{
				return;
			}
			PluginLabelingSelectionResult stroke;
			stroke.pointIndices.reserve(static_cast<std::size_t>(indices.size()));
			for (int idx : indices)
			{
				if (idx >= 0)
				{
					stroke.pointIndices.push_back(static_cast<std::size_t>(idx));
				}
			}
			onStroke(stroke);
		});

	m_pickState->brushFinishConn = QObject::connect(
		osg,
		&OsgWidget::labelingBrushFinished,
		m_host,
		[]()
		{
			// 单次刷选结束，保持刷选模式直至 Esc
		});

	m_pickState->cancelConn = QObject::connect(
		osg,
		&OsgWidget::labelingPickCanceled,
		m_host,
		[this]() {
			clearActivePickState(true);
		});
}

void PluginLabelingHostImpl::pickPolylineRegion(const PluginLabelingSessionId sessionId, PluginLabelingPickFinishedFn onFinished)
{
	if (!onFinished || !m_host)
	{
		return;
	}
	const SessionEntry* entry = findSession(sessionId);
	if (!entry || !entry->session)
	{
		onFinished(false, QStringLiteral("Invalid session"), {});
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(entry->doc);
	if (!page)
	{
		onFinished(false, QStringLiteral("No document"), {});
		return;
	}

	m_host->pointCloudHost()->pickPolylineFromViewport(
		entry->doc,
		[=](const bool ok, const QString& err, const PluginPointCloudPolylinePickResult& poly) {
			if (!ok)
			{
				onFinished(false, err, {});
				return;
			}
			PluginLabelingSelectionResult sel;
			if (entry->kind == PluginLabelingGeometryKind::PointCloud)
			{
				const auto pc = document_point_cloud_ops::resolvePointCloud(page, entry->backendId, nullptr);
				if (!pc)
				{
					onFinished(false, QStringLiteral("Point cloud gone"), {});
					return;
				}
				std::vector<std::size_t> kept;
				double modelToWorld[16] = {
					1.0, 0.0, 0.0, 0.0,
					0.0, 1.0, 0.0, 0.0,
					0.0, 0.0, 1.0, 0.0,
					0.0, 0.0, 0.0, 1.0};
				if (OsgWidget* osg = widgetOsgFromPage(page))
				{
					(void)osg->tryGetBackendPointLocalToWorldMatrix(entry->backendId, modelToWorld);
				}
				std::string cropErr;
				(void)point_cloud_backend_ops::collectPointCloudIndicesByPolyline2D(
					*pc,
					poly.polylineScreenXy,
					poly.mvpMatrix,
					modelToWorld,
					poly.viewportWidth,
					poly.viewportHeight,
					true,
					kept,
					&cropErr);
				sel.pointIndices = std::move(kept);
			}
			onFinished(true, QString(), sel);
		});
}

void PluginLabelingHostImpl::pickMeshFaceOnce(const PluginLabelingSessionId sessionId, PluginLabelingPickFinishedFn onFinished)
{
	if (!onFinished)
	{
		return;
	}
	const SessionEntry* entry = findSession(sessionId);
	if (!entry)
	{
		onFinished(false, QStringLiteral("Invalid session"), {});
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(entry->doc);
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg || !m_host)
	{
		onFinished(false, QStringLiteral("Viewport unavailable"), {});
		return;
	}

	abandonActiveLabelingPick();
	m_pickState = std::make_unique<ActivePickState>();
	m_pickState->viewportWidget = osg;
	m_pickState->meshFace = true;
	m_pickState->sessionId = sessionId;
	m_activePickSessionId = sessionId;
	osg->setLabelingClickPickMode(true, true);

	m_pickState->clickConn = QObject::connect(
		osg,
		&OsgWidget::labelingClickCommitted,
		m_host,
		[=](const PickResult& pick) {
			PluginLabelingSelectionResult sel;
			if (pick.hit && pick.meshTriangleIndex >= 0)
			{
				sel.triangleIndices.push_back(pick.meshTriangleIndex);
			}
			onFinished(pick.hit, pick.hit ? QString() : QStringLiteral("No face hit"), sel);
		});

	m_pickState->cancelConn = QObject::connect(
		osg,
		&OsgWidget::labelingPickCanceled,
		m_host,
		[this]() {
			clearActivePickState(true);
		});
}

void PluginLabelingHostImpl::brushMeshFaces(
	const PluginLabelingSessionId sessionId,
	const float radiusPx,
	PluginLabelingBrushStrokeFn onStroke,
	PluginLabelingBrushFinishedFn onFinished)
{
	if (!onFinished)
	{
		return;
	}
	const SessionEntry* entry = findSession(sessionId);
	if (!entry)
	{
		onFinished(false, QStringLiteral("Invalid session"), {});
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(entry->doc);
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg || !m_host)
	{
		onFinished(false, QStringLiteral("Viewport unavailable"), {});
		return;
	}

	abandonActiveLabelingPick();
	m_pickState = std::make_unique<ActivePickState>();
	m_pickState->viewportWidget = osg;
	m_pickState->meshFace = true;
	m_pickState->brushRadius = radiusPx;
	m_pickState->sessionId = sessionId;
	m_pickState->brushFinished = onFinished;
	m_activePickSessionId = sessionId;
	osg->setLabelingBrushPickMode(true, true, radiusPx);

	m_pickState->brushStrokeConn = QObject::connect(
		osg,
		&OsgWidget::labelingBrushStroke,
		m_host,
		[=](const QVector<int>& indices) {
			if (!onStroke)
			{
				return;
			}
			PluginLabelingSelectionResult stroke;
			for (int idx : indices)
			{
				if (idx >= 0)
				{
					stroke.triangleIndices.push_back(idx);
				}
			}
			onStroke(stroke);
		});

	m_pickState->brushFinishConn = QObject::connect(
		osg,
		&OsgWidget::labelingBrushFinished,
		m_host,
		[]()
		{
		});

	m_pickState->cancelConn = QObject::connect(
		osg,
		&OsgWidget::labelingPickCanceled,
		m_host,
		[this]() {
			clearActivePickState(true);
		});
}
