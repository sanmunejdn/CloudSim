/// @file GeometricModelingSolidFeatures.cpp
/// @brief 圆角/倒角/旋转/阵列/镜像/放样/抽壳侧栏流程

#include "GeometricModelingPlugin.h"

#include "BodyHistoryCmd.h"
#include "CommandStack.h"
#include "GeometricModelingPage.h"
#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"
#include "SketchGeom.h"

#include <QSignalBlocker>
#include <QInputDialog>

#include <algorithm>
#include <cmath>

namespace
{
bool appendUniqueEdge(std::vector<int>& edges, int idx)
{
	if (idx < 0)
		return false;
	if (std::find(edges.begin(), edges.end(), idx) != edges.end())
		return false;
	edges.push_back(idx);
	return true;
}

bool appendUniqueFace(std::vector<int>& faces, int idx)
{
	return appendUniqueEdge(faces, idx);
}

double pointPlaneDistMm(const PluginPoint3d& p, const PluginSketchPlane& plane)
{
	const double dx = p.x - plane.origin.x;
	const double dy = p.y - plane.origin.y;
	const double dz = p.z - plane.origin.z;
	const double nx = plane.normal.x;
	const double ny = plane.normal.y;
	const double nz = plane.normal.z;
	const double nlen = std::sqrt(nx * nx + ny * ny + nz * nz);
	if (nlen < 1e-9)
		return 1e9;
	return std::abs(dx * nx + dy * ny + dz * nz) / nlen;
}

bool polylineOnPlane(const std::vector<float>& pl, const PluginSketchPlane& plane, double tolMm)
{
	if (pl.size() < 6)
		return false;
	for (std::size_t i = 0; i + 2 < pl.size(); i += 3)
	{
		const PluginPoint3d w{pl[i], pl[i + 1], pl[i + 2]};
		if (pointPlaneDistMm(w, plane) > tolMm)
			return false;
	}
	return true;
}

int projectPolylineToSketch(SketchDocument2d& skDoc, const PluginSketchPlane& plane, const std::vector<float>& pl)
{
	SkVec2 prevUv{};
	bool hasPrev = false;
	int added = 0;
	for (std::size_t i = 0; i + 2 < pl.size(); i += 3)
	{
		const PluginPoint3d w{pl[i], pl[i + 1], pl[i + 2]};
		const SkVec2 uv = skDoc.worldToUv(plane, w);
		if (hasPrev)
		{
			const int p1 = skDoc.addPoint(prevUv.u, prevUv.v);
			const int p2 = skDoc.addPoint(uv.u, uv.v);
			skDoc.addLine(p1, p2, false);
			++added;
		}
		prevUv = uv;
		hasPrev = true;
	}
	return added;
}

int projectBoundarySegToSketch(SketchDocument2d& skDoc, const PluginSketchPlane& plane,
							   const PluginFaceBoundarySeg& seg)
{
	auto uvAt = [&](std::size_t i) -> SkVec2 {
		const PluginPoint3d w{seg.xyz[i], seg.xyz[i + 1], seg.xyz[i + 2]};
		return skDoc.worldToUv(plane, w);
	};

	if (seg.kind == PluginFaceBoundarySegKind::Line && seg.xyz.size() >= 6)
	{
		const SkVec2 a = uvAt(0);
		const SkVec2 b = uvAt(3);
		const int p1 = skDoc.addPoint(a.u, a.v);
		const int p2 = skDoc.addPoint(b.u, b.v);
		skDoc.addLine(p1, p2, false);
		return 1;
	}
	if (seg.kind == PluginFaceBoundarySegKind::Arc && seg.xyz.size() >= 9)
	{
		const SkVec2 a = uvAt(0);
		const SkVec2 m = uvAt(3);
		const SkVec2 b = uvAt(6);
		const int p1 = skDoc.addPoint(a.u, a.v);
		const int p2 = skDoc.addPoint(m.u, m.v);
		const int p3 = skDoc.addPoint(b.u, b.v);
		skDoc.addArc(p1, p2, p3, false);
		return 1;
	}
	if (seg.kind == PluginFaceBoundarySegKind::Circle && seg.xyz.size() >= 6)
	{
		const SkVec2 c = uvAt(0);
		const SkVec2 rim = uvAt(3);
		const double du = rim.u - c.u;
		const double dv = rim.v - c.v;
		double r = std::sqrt(du * du + dv * dv);
		if (r < 1e-9 && seg.radiusMm > 0.0)
			r = seg.radiusMm;
		if (r < 1e-9)
			return 0;
		const int cid = skDoc.addPoint(c.u, c.v, true);
		const int circleId = skDoc.addCircle(cid, r, false);
		skDoc.addConstraint({SkConstraintKind::Radius, circleId, -1, r});
		return 1;
	}
	if (seg.xyz.size() >= 6)
		return projectPolylineToSketch(skDoc, plane, seg.xyz);
	return 0;
}
} // namespace

PluginSketchPlane GeometricModelingPlugin::originMirrorPlane(int planeIndex) const
{
	PluginSketchPlane p{};
	p.isPlanar = true;
	p.origin = {0, 0, 0};
	if (planeIndex == 1)
	{
		p.axisX = {1, 0, 0};
		p.axisY = {0, 0, 1};
		p.normal = {0, 1, 0};
	}
	else if (planeIndex == 2)
	{
		p.axisX = {0, 1, 0};
		p.axisY = {0, 0, 1};
		p.normal = {1, 0, 0};
	}
	else
	{
		p.axisX = {1, 0, 0};
		p.axisY = {0, 1, 0};
		p.normal = {0, 0, 1};
	}
	return p;
}

void GeometricModelingPlugin::clearSolidFeaturePreviewUi()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (geo && doc)
		geo->clearSketchExtrudePreview(doc);
	m_solidPreviewActive = false;
	m_solidCutMode = false;
	m_solidPanel = SideToolPanel::None;
	m_pickedEdgeIndices.clear();
	m_pickedFaceIndices.clear();
	m_revolveProfile.clear();
	m_loftProfileA.clear();
	m_loftProfileB.clear();
	m_revolveAxisPicked = false;
	m_circPatternAxisPicked = false;
	if (GeometricModelingPage* page = ensurePageForActiveDocument())
	{
		page->setFilletUi(false);
		page->setChamferUi(false);
		page->setRevolveUi(false, false);
		page->setPatternUi(false);
		page->setCircularPatternUi(false);
		page->setMirror3dUi(false);
		page->setLoftUi(false, false);
		page->setShellUi(false);
		page->setDraftUi(false);
	}
}


void GeometricModelingPlugin::beginFilletPanel()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Fillet requires an active Parametric Body."),
						 QStringLiteral("\u5706\u89d2\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::Fillet;
	m_pickedEdgeIndices.clear();
	page->setFilletUi(true);
	page->setFilletEdgeCount(0);
	hostLogInfo(i18n(QStringLiteral("Pick edges for fillet."), QStringLiteral("\u8bf7\u70b9\u9009\u5706\u89d2\u8fb9\u3002")));
}

void GeometricModelingPlugin::onPickFilletEdge()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_solidPreviewActive || m_solidPanel != SideToolPanel::Fillet)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Edge;
	req.backendIdUtf8 = page->activeBodyId().toStdString();
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			if (appendUniqueEdge(m_pickedEdgeIndices, ref.edgeIndex))
			{
				page->setFilletEdgeCount(static_cast<int>(m_pickedEdgeIndices.size()));
				refreshFilletPreview();
			}
		});
}

void GeometricModelingPlugin::refreshFilletPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive || m_pickedEdgeIndices.empty())
		return;

	PluginSketchFilletParams params;
	params.radiusMm = page->filletRadiusMm();
	params.edgeIndices = m_pickedEdgeIndices;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	QString previewErr;
	if (!geo->previewFilletEdges(doc, params, &previewErr))
		hostLogWarn(previewErr.isEmpty() ? i18n(QStringLiteral("Fillet preview failed."), QStringLiteral("\u5706\u89d2\u9884\u89c8\u5931\u8d25\u3002"))
										 : previewErr);
}

void GeometricModelingPlugin::onConfirmFillet()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || m_pickedEdgeIndices.empty())
	{
		hostLogWarn(i18n(QStringLiteral("Select at least one edge."), QStringLiteral("\u8bf7\u81f3\u5c11\u9009\u62e9\u4e00\u6761\u8fb9\u3002")));
		return;
	}

	refreshFilletPreview();
	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchFilletParams params;
	params.radiusMm = page->filletRadiusMm();
	params.edgeIndices = m_pickedEdgeIndices;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.resultNameUtf8 = "ParametricBody";
	const std::vector<int> edges = m_pickedEdgeIndices;
	const double radius = params.radiusMm;
	clearSolidFeaturePreviewUi();

	geo->filletEdgesToBrep(
		doc, params,
		[this, page, doc, beforeHist, edges, radius](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Fillet created on body: %1"), QStringLiteral("\u5df2\u5728\u5b9e\u4f53 %1 \u4e0a\u521b\u5efa\u5706\u89d2"))
							.arg(page->activeBodyId()));
			(void)edges;
			(void)radius;
		});
}

void GeometricModelingPlugin::onCancelFillet()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Fillet cancelled."), QStringLiteral("\u5706\u89d2\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::beginChamferPanel()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Chamfer requires an active Parametric Body."),
						 QStringLiteral("\u5012\u89d2\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::Chamfer;
	m_pickedEdgeIndices.clear();
	page->setChamferUi(true);
	page->setChamferEdgeCount(0);
	hostLogInfo(i18n(QStringLiteral("Pick edges for chamfer."), QStringLiteral("\u8bf7\u70b9\u9009\u5012\u89d2\u8fb9\u3002")));
}

void GeometricModelingPlugin::onPickChamferEdge()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_solidPreviewActive || m_solidPanel != SideToolPanel::Chamfer)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Edge;
	req.backendIdUtf8 = page->activeBodyId().toStdString();
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			if (appendUniqueEdge(m_pickedEdgeIndices, ref.edgeIndex))
			{
				page->setChamferEdgeCount(static_cast<int>(m_pickedEdgeIndices.size()));
				refreshChamferPreview();
			}
		});
}

void GeometricModelingPlugin::refreshChamferPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive || m_pickedEdgeIndices.empty())
		return;

	PluginSketchChamferParams params;
	params.distanceMm = page->chamferDistanceMm();
	params.edgeIndices = m_pickedEdgeIndices;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	QString previewErr;
	if (!geo->previewChamferEdges(doc, params, &previewErr))
		hostLogWarn(previewErr.isEmpty() ? i18n(QStringLiteral("Chamfer preview failed."), QStringLiteral("\u5012\u89d2\u9884\u89c8\u5931\u8d25\u3002"))
										 : previewErr);
}

void GeometricModelingPlugin::onConfirmChamfer()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || m_pickedEdgeIndices.empty())
	{
		hostLogWarn(i18n(QStringLiteral("Select at least one edge."), QStringLiteral("\u8bf7\u81f3\u5c11\u9009\u62e9\u4e00\u6761\u8fb9\u3002")));
		return;
	}

	refreshChamferPreview();
	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchChamferParams params;
	params.distanceMm = page->chamferDistanceMm();
	params.edgeIndices = m_pickedEdgeIndices;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.resultNameUtf8 = "ParametricBody";
	clearSolidFeaturePreviewUi();

	geo->chamferEdgesToBrep(
		doc, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Chamfer created."), QStringLiteral("\u5012\u89d2\u5df2\u521b\u5efa\u3002")));
		});
}

void GeometricModelingPlugin::onCancelChamfer()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Chamfer cancelled."), QStringLiteral("\u5012\u89d2\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::beginRevolvePanel(bool cut)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (cut && page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("RevolveCut requires an active Parametric Body."),
						 QStringLiteral("\u65cb\u8f6c\u5207\u9664\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::Revolve;
	m_solidCutMode = cut;
	m_revolveAxisPicked = false;
	page->fillRevolveSketchCombo();
	page->setRevolveUi(true, cut);
	page->setRevolveAxisLabel(i18n(QStringLiteral("Default: sketch origin + Y"),
								   QStringLiteral("\u9ed8\u8ba4\uff1a\u8349\u56fe\u539f\u70b9 + Y")));
	refreshRevolvePreview();
	hostLogInfo(i18n(QStringLiteral("Select profile sketch for revolve."),
					 QStringLiteral("\u8bf7\u9009\u62e9\u8f6e\u5ed3\u8349\u56fe\u8fdb\u884c\u65cb\u8f6c\u3002")));
}

void GeometricModelingPlugin::fillRevolveAxisParams(GeometricModelingPage* page, const GeomodelingFeature& sk,
													PluginSketchRevolveParams& params) const
{
	params.axisOx = sk.plane.origin.x;
	params.axisOy = sk.plane.origin.y;
	params.axisOz = sk.plane.origin.z;
	const int mode = page ? page->revolveAxisMode() : 0;
	if (mode == 1)
	{
		params.axisDx = sk.plane.axisX.x;
		params.axisDy = sk.plane.axisX.y;
		params.axisDz = sk.plane.axisX.z;
	}
	else if (mode == 2 && m_revolveAxisPicked)
	{
		params.axisOx = m_revolveAxisOx;
		params.axisOy = m_revolveAxisOy;
		params.axisOz = m_revolveAxisOz;
		params.axisDx = m_revolveAxisDx;
		params.axisDy = m_revolveAxisDy;
		params.axisDz = m_revolveAxisDz;
	}
	else
	{
		params.axisDx = sk.plane.axisY.x;
		params.axisDy = sk.plane.axisY.y;
		params.axisDz = sk.plane.axisY.z;
	}
}

void GeometricModelingPlugin::onPickRevolveAxis()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_solidPreviewActive || m_solidPanel != SideToolPanel::Revolve)
		return;
	if (page->revolveAxisMode() != 2)
	{
		hostLogInfo(i18n(QStringLiteral("Switch axis mode to Pick edge first."),
						 QStringLiteral("\u8bf7\u5148\u5c06\u65cb\u8f6c\u8f74\u5207\u6362\u4e3a\u62fe\u53d6\u8fb9\u3002")));
		return;
	}

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Edge;
	req.backendIdUtf8 = page->activeBodyId().toStdString();
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			if (!ref.hasEdgeEnds)
			{
				hostLogWarn(i18n(QStringLiteral("Edge endpoints unavailable."),
								 QStringLiteral("\u65e0\u6cd5\u83b7\u53d6\u8fb9\u7aef\u70b9\u3002")));
				return;
			}
			m_revolveAxisOx = ref.edgeEndAMm.x;
			m_revolveAxisOy = ref.edgeEndAMm.y;
			m_revolveAxisOz = ref.edgeEndAMm.z;
			m_revolveAxisDx = ref.edgeEndBMm.x - ref.edgeEndAMm.x;
			m_revolveAxisDy = ref.edgeEndBMm.y - ref.edgeEndAMm.y;
			m_revolveAxisDz = ref.edgeEndBMm.z - ref.edgeEndAMm.z;
			m_revolveAxisPicked = true;
			page->setRevolveAxisLabel(
				i18n(QStringLiteral("Axis from picked edge"), QStringLiteral("\u5df2\u7528\u62fe\u53d6\u8fb9\u4f5c\u8f74")));
			refreshRevolvePreview();
		});
}

void GeometricModelingPlugin::refreshRevolvePreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive)
		return;

	const QString skId = page->revolveSketchId();
	const GeomodelingFeature* sk = page->features().find(skId);
	if (!sk)
		return;

	QString err;
	if (!loadSketchPolyline(*sk, false, m_revolveProfile, &err))
	{
		page->setRevolveStatus(err);
		geo->clearSketchExtrudePreview(doc);
		return;
	}
	m_revolvePlane = sk->plane;

	PluginSketchRevolveParams params;
	params.mode = m_solidCutMode ? PluginSketchRevolveMode::Cut : PluginSketchRevolveMode::Boss;
	params.angleDeg = page->revolveAngleDeg();
	fillRevolveAxisParams(page, *sk, params);
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.sketchIdUtf8 = skId.toStdString();
	params.sketchDocumentJsonUtf8 = QString::fromUtf8(sk->sketchDocumentUtf8).toStdString();
	params.plane = sk->plane;

	QString previewErr;
	if (!geo->previewSketchRevolve(doc, m_revolveProfile, params, &previewErr))
	{
		page->setRevolveStatus(previewErr);
		return;
	}
	page->setRevolveStatus(QString());
}

void GeometricModelingPlugin::onConfirmRevolve()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || m_revolveProfile.size() < 12)
		return;

	refreshRevolvePreview();
	const QString skId = page->revolveSketchId();
	const GeomodelingFeature* sk = page->features().find(skId);
	if (!sk)
		return;

	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchRevolveParams params;
	params.mode = m_solidCutMode ? PluginSketchRevolveMode::Cut : PluginSketchRevolveMode::Boss;
	params.angleDeg = page->revolveAngleDeg();
	fillRevolveAxisParams(page, *sk, params);
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	if (!m_solidCutMode && page->activeBodyId().isEmpty())
		params.targetParametricBackendIdUtf8.clear();
	params.sketchIdUtf8 = skId.toStdString();
	params.sketchDocumentJsonUtf8 = QString::fromUtf8(sk->sketchDocumentUtf8).toStdString();
	params.plane = sk->plane;
	params.resultNameUtf8 = "ParametricBody";
	const std::vector<float> profile = m_revolveProfile;
	clearSolidFeaturePreviewUi();

	geo->revolveSketchProfileToBrep(
		doc, profile, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Revolve feature created."), QStringLiteral("\u65cb\u8f6c\u7279\u5f81\u5df2\u521b\u5efa\u3002")));
		});
}

void GeometricModelingPlugin::onCancelRevolve()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Revolve cancelled."), QStringLiteral("\u65cb\u8f6c\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::beginPatternPanel()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Linear pattern requires an active Parametric Body."),
						 QStringLiteral("\u7ebf\u6027\u9635\u5217\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::Pattern;
	page->setPatternUi(true);
	refreshPatternPreview();
}

void GeometricModelingPlugin::refreshPatternPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive)
		return;

	PluginSketchLinearPatternParams params;
	params.count = page->patternCount();
	params.dxMm = page->patternDxMm();
	params.dyMm = page->patternDyMm();
	params.dzMm = page->patternDzMm();
	params.sourceFeatureIdUtf8 = page->patternSourceFeatureId().toStdString();
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	QString previewErr;
	if (!geo->previewLinearPattern(doc, params, &previewErr))
		hostLogWarn(previewErr.isEmpty() ? i18n(QStringLiteral("Pattern preview failed."), QStringLiteral("\u9635\u5217\u9884\u89c8\u5931\u8d25\u3002"))
										 : previewErr);
}

void GeometricModelingPlugin::onConfirmPattern()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo)
		return;

	refreshPatternPreview();
	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchLinearPatternParams params;
	params.count = page->patternCount();
	params.dxMm = page->patternDxMm();
	params.dyMm = page->patternDyMm();
	params.dzMm = page->patternDzMm();
	params.sourceFeatureIdUtf8 = page->patternSourceFeatureId().toStdString();
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.resultNameUtf8 = "ParametricBody";
	clearSolidFeaturePreviewUi();

	geo->linearPatternBodyToBrep(
		doc, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Linear pattern created."), QStringLiteral("\u7ebf\u6027\u9635\u5217\u5df2\u521b\u5efa\u3002")));
		});
}

void GeometricModelingPlugin::onCancelPattern()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Pattern cancelled."), QStringLiteral("\u9635\u5217\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::beginCircularPatternPanel()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Circular pattern requires an active Parametric Body."),
						 QStringLiteral("\u5706\u5468\u9635\u5217\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::CircularPattern;
	m_circPatternAxisPicked = false;
	m_circPatternOx = 0;
	m_circPatternOy = 0;
	m_circPatternOz = 0;
	m_circPatternDx = 0;
	m_circPatternDy = 0;
	m_circPatternDz = 1;
	page->setCircularPatternUi(true);
	page->setCircularPatternAxisLabel(i18n(QStringLiteral("Axis: pick a model edge (default Z)"),
										   QStringLiteral("\u8f74\uff1a\u70b9\u9009\u6a21\u578b\u8fb9\uff08\u9ed8\u8ba4 Z\uff09")));
	refreshCircularPatternPreview();
}

void GeometricModelingPlugin::onPickCircularPatternAxis()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_solidPreviewActive || m_solidPanel != SideToolPanel::CircularPattern)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Edge;
	req.backendIdUtf8 = page->activeBodyId().toStdString();
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			if (!ref.hasEdgeEnds)
			{
				hostLogWarn(i18n(QStringLiteral("Edge endpoints unavailable."),
								 QStringLiteral("\u65e0\u6cd5\u83b7\u53d6\u8fb9\u7aef\u70b9\u3002")));
				return;
			}
			m_circPatternOx = ref.edgeEndAMm.x;
			m_circPatternOy = ref.edgeEndAMm.y;
			m_circPatternOz = ref.edgeEndAMm.z;
			m_circPatternDx = ref.edgeEndBMm.x - ref.edgeEndAMm.x;
			m_circPatternDy = ref.edgeEndBMm.y - ref.edgeEndAMm.y;
			m_circPatternDz = ref.edgeEndBMm.z - ref.edgeEndAMm.z;
			m_circPatternAxisPicked = true;
			page->setCircularPatternAxisLabel(
				i18n(QStringLiteral("Axis from picked edge"), QStringLiteral("\u5df2\u7528\u62fe\u53d6\u8fb9\u4f5c\u8f74")));
			refreshCircularPatternPreview();
		});
}

void GeometricModelingPlugin::refreshCircularPatternPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive)
		return;

	PluginSketchCircularPatternParams params;
	params.count = page->circularPatternCount();
	params.angleDeg = page->circularPatternAngleDeg();
	params.axisOx = m_circPatternOx;
	params.axisOy = m_circPatternOy;
	params.axisOz = m_circPatternOz;
	params.axisDx = m_circPatternDx;
	params.axisDy = m_circPatternDy;
	params.axisDz = m_circPatternDz;
	params.sourceFeatureIdUtf8 = page->circularPatternSourceFeatureId().toStdString();
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	QString previewErr;
	if (!geo->previewCircularPattern(doc, params, &previewErr))
		hostLogWarn(previewErr.isEmpty()
						? i18n(QStringLiteral("Circular pattern preview failed."),
							   QStringLiteral("\u5706\u5468\u9635\u5217\u9884\u89c8\u5931\u8d25\u3002"))
						: previewErr);
}

void GeometricModelingPlugin::onConfirmCircularPattern()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo)
		return;

	refreshCircularPatternPreview();
	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchCircularPatternParams params;
	params.count = page->circularPatternCount();
	params.angleDeg = page->circularPatternAngleDeg();
	params.axisOx = m_circPatternOx;
	params.axisOy = m_circPatternOy;
	params.axisOz = m_circPatternOz;
	params.axisDx = m_circPatternDx;
	params.axisDy = m_circPatternDy;
	params.axisDz = m_circPatternDz;
	params.sourceFeatureIdUtf8 = page->circularPatternSourceFeatureId().toStdString();
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.resultNameUtf8 = "ParametricBody";
	clearSolidFeaturePreviewUi();

	geo->circularPatternBodyToBrep(
		doc, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Circular pattern created."),
							 QStringLiteral("\u5706\u5468\u9635\u5217\u5df2\u521b\u5efa\u3002")));
		});
}

void GeometricModelingPlugin::onCancelCircularPattern()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Circular pattern cancelled."), QStringLiteral("\u5706\u5468\u9635\u5217\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::beginMirror3dPanel()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Mirror requires an active Parametric Body."),
						 QStringLiteral("\u955c\u50cf\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::Mirror3D;
	page->setMirror3dUi(true);
	refreshMirror3dPreview();
}

void GeometricModelingPlugin::refreshMirror3dPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive)
		return;

	PluginSketchMirror3dParams params;
	params.plane = originMirrorPlane(page->mirror3dPlaneIndex());
	params.keepOriginal = page->mirror3dKeepOriginal();
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	QString previewErr;
	if (!geo->previewMirror3d(doc, params, &previewErr))
		hostLogWarn(previewErr.isEmpty() ? i18n(QStringLiteral("Mirror preview failed."), QStringLiteral("\u955c\u50cf\u9884\u89c8\u5931\u8d25\u3002"))
										 : previewErr);
}

void GeometricModelingPlugin::onConfirmMirror3d()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo)
		return;

	refreshMirror3dPreview();
	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchMirror3dParams params;
	params.plane = originMirrorPlane(page->mirror3dPlaneIndex());
	params.keepOriginal = page->mirror3dKeepOriginal();
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.resultNameUtf8 = "ParametricBody";
	clearSolidFeaturePreviewUi();

	geo->mirror3dBodyToBrep(
		doc, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Mirror feature created."), QStringLiteral("\u955c\u50cf\u7279\u5f81\u5df2\u521b\u5efa\u3002")));
		});
}

void GeometricModelingPlugin::onCancelMirror3d()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Mirror cancelled."), QStringLiteral("\u955c\u50cf\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::beginLoftPanel(bool cut)
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (cut && page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("LoftCut requires an active Parametric Body."),
						 QStringLiteral("\u653e\u6837\u5207\u9664\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::Loft;
	m_solidCutMode = cut;
	page->fillLoftSketchCombos();
	page->setLoftUi(true, cut);
	refreshLoftPreview();
}

void GeometricModelingPlugin::refreshLoftPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive)
		return;

	const QString aId = page->loftSketchAId();
	const QString bId = page->loftSketchBId();
	if (aId.isEmpty() || bId.isEmpty() || aId == bId)
	{
		page->setLoftStatus(i18n(QStringLiteral("Select two different sketches."),
								 QStringLiteral("\u8bf7\u9009\u62e9\u4e24\u5f20\u4e0d\u540c\u8349\u56fe\u3002")));
		geo->clearSketchExtrudePreview(doc);
		return;
	}
	const GeomodelingFeature* skA = page->features().find(aId);
	const GeomodelingFeature* skB = page->features().find(bId);
	if (!skA || !skB)
		return;

	QString err;
	if (!loadSketchPolyline(*skA, false, m_loftProfileA, &err) || !loadSketchPolyline(*skB, false, m_loftProfileB, &err))
	{
		page->setLoftStatus(err);
		geo->clearSketchExtrudePreview(doc);
		return;
	}

	PluginSketchLoftParams params;
	params.mode = m_solidCutMode ? PluginSketchLoftMode::Cut : PluginSketchLoftMode::Boss;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.sketchAIdUtf8 = aId.toStdString();
	params.sketchBIdUtf8 = bId.toStdString();
	params.sketchADocumentJsonUtf8 = QString::fromUtf8(skA->sketchDocumentUtf8).toStdString();
	params.sketchBDocumentJsonUtf8 = QString::fromUtf8(skB->sketchDocumentUtf8).toStdString();
	params.planeA = skA->plane;
	params.planeB = skB->plane;

	QString previewErr;
	if (!geo->previewSketchLoft(doc, m_loftProfileA, m_loftProfileB, params, &previewErr))
	{
		page->setLoftStatus(previewErr);
		return;
	}
	page->setLoftStatus(QString());
}

void GeometricModelingPlugin::onConfirmLoft()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || m_loftProfileA.size() < 12 || m_loftProfileB.size() < 12)
		return;

	refreshLoftPreview();
	const QString aId = page->loftSketchAId();
	const QString bId = page->loftSketchBId();
	const GeomodelingFeature* skA = page->features().find(aId);
	const GeomodelingFeature* skB = page->features().find(bId);
	if (!skA || !skB)
		return;

	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchLoftParams params;
	params.mode = m_solidCutMode ? PluginSketchLoftMode::Cut : PluginSketchLoftMode::Boss;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	if (!m_solidCutMode && page->activeBodyId().isEmpty())
		params.targetParametricBackendIdUtf8.clear();
	params.sketchAIdUtf8 = aId.toStdString();
	params.sketchBIdUtf8 = bId.toStdString();
	params.sketchADocumentJsonUtf8 = QString::fromUtf8(skA->sketchDocumentUtf8).toStdString();
	params.sketchBDocumentJsonUtf8 = QString::fromUtf8(skB->sketchDocumentUtf8).toStdString();
	params.planeA = skA->plane;
	params.planeB = skB->plane;
	params.resultNameUtf8 = "ParametricBody";
	const std::vector<float> profA = m_loftProfileA;
	const std::vector<float> profB = m_loftProfileB;
	clearSolidFeaturePreviewUi();

	geo->loftSketchProfilesToBrep(
		doc, profA, profB, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Loft feature created."), QStringLiteral("\u653e\u6837\u7279\u5f81\u5df2\u521b\u5efa\u3002")));
		});
}

void GeometricModelingPlugin::onCancelLoft()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Loft cancelled."), QStringLiteral("\u653e\u6837\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::beginShellPanel()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Shell requires an active Parametric Body."),
						 QStringLiteral("\u62bd\u58f3\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::Shell;
	m_pickedFaceIndices.clear();
	page->setShellUi(true);
	page->setShellFaceCount(0);
	hostLogInfo(i18n(QStringLiteral("Pick faces to remove for shell."), QStringLiteral("\u8bf7\u70b9\u9009\u8981\u62bd\u58f3\u7684\u9762\u3002")));
}

void GeometricModelingPlugin::onPickShellFace()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_solidPreviewActive || m_solidPanel != SideToolPanel::Shell)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	req.backendIdUtf8 = page->activeBodyId().toStdString();
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			if (appendUniqueFace(m_pickedFaceIndices, ref.faceIndex))
			{
				page->setShellFaceCount(static_cast<int>(m_pickedFaceIndices.size()));
				refreshShellPreview();
			}
		});
}

void GeometricModelingPlugin::refreshShellPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive || m_pickedFaceIndices.empty())
		return;

	PluginSketchShellParams params;
	params.thicknessMm = page->shellThicknessMm();
	params.faceIndices = m_pickedFaceIndices;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	QString previewErr;
	if (!geo->previewShellFaces(doc, params, &previewErr))
		page->setShellStatus(previewErr);
	else
		page->setShellStatus(QString());
}

void GeometricModelingPlugin::onConfirmShell()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || m_pickedFaceIndices.empty())
	{
		hostLogWarn(i18n(QStringLiteral("Select at least one face."), QStringLiteral("\u8bf7\u81f3\u5c11\u9009\u62e9\u4e00\u4e2a\u9762\u3002")));
		return;
	}

	refreshShellPreview();
	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchShellParams params;
	params.thicknessMm = page->shellThicknessMm();
	params.faceIndices = m_pickedFaceIndices;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	params.resultNameUtf8 = "ParametricBody";
	clearSolidFeaturePreviewUi();

	geo->shellFacesToBrep(
		doc, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Shell feature created."), QStringLiteral("\u62bd\u58f3\u7279\u5f81\u5df2\u521b\u5efa\u3002")));
		});
}

void GeometricModelingPlugin::onCancelShell()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Shell cancelled."), QStringLiteral("\u62bd\u58f3\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::onFillet() { beginFilletPanel(); }
void GeometricModelingPlugin::onChamfer() { beginChamferPanel(); }
void GeometricModelingPlugin::onRevolve() { beginRevolvePanel(false); }
void GeometricModelingPlugin::onRevolveCut() { beginRevolvePanel(true); }
void GeometricModelingPlugin::onLinearPattern() { beginPatternPanel(); }
void GeometricModelingPlugin::onCircularPattern() { beginCircularPatternPanel(); }
void GeometricModelingPlugin::onMirror3d() { beginMirror3dPanel(); }
void GeometricModelingPlugin::onLoft() { beginLoftPanel(false); }
void GeometricModelingPlugin::onLoftCut() { beginLoftPanel(true); }
void GeometricModelingPlugin::onShell() { beginShellPanel(); }

void GeometricModelingPlugin::beginDraftPanel()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!page)
		return;
	if (page->activeBodyId().isEmpty())
	{
		hostLogWarn(i18n(QStringLiteral("Draft requires an active Parametric Body."),
						 QStringLiteral("\u62d4\u6a21\u9700\u8981\u6d3b\u52a8 Parametric Body\u3002")));
		return;
	}
	clearExtrudePreviewUi();
	clearSweepPreviewUi();
	clearSolidFeaturePreviewUi();
	m_solidPreviewActive = true;
	m_solidPanel = SideToolPanel::Draft;
	m_pickedFaceIndices.clear();
	page->clearDraftNeutralPlane();
	page->setDraftUi(true);
	page->setDraftFaceCount(0);
	hostLogInfo(i18n(QStringLiteral("Pick faces for draft."), QStringLiteral("\u8bf7\u70b9\u9009\u62d4\u6a21\u9762\u3002")));
}

void GeometricModelingPlugin::onPickDraftFace()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_solidPreviewActive || m_solidPanel != SideToolPanel::Draft)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	req.backendIdUtf8 = page->activeBodyId().toStdString();
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			if (appendUniqueFace(m_pickedFaceIndices, ref.faceIndex))
			{
				page->setDraftFaceCount(static_cast<int>(m_pickedFaceIndices.size()));
				refreshDraftPreview();
			}
		});
}

void GeometricModelingPlugin::refreshDraftPreview()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || !m_solidPreviewActive || m_pickedFaceIndices.empty())
		return;

	PluginSketchDraftParams params;
	params.angleDeg = page->draftAngleDeg();
	params.faceIndices = m_pickedFaceIndices;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	if (page->hasDraftNeutralPlane())
	{
		params.neutralPlane = page->draftNeutralPlane();
	}
	else
	{
		params.neutralPlane.isPlanar = true;
		params.neutralPlane.origin = {0, 0, 0};
		params.neutralPlane.normal = {0, 0, 1};
	}
	QString previewErr;
	if (!geo->previewDraftFaces(doc, params, &previewErr))
		page->setDraftStatus(previewErr);
	else
		page->setDraftStatus(QString());
}

void GeometricModelingPlugin::onPickDraftNeutral()
{
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	GeometricModelingPage* page = ensurePageForActiveDocument();
	if (!doc || !geo || !page || !m_solidPreviewActive || m_solidPanel != SideToolPanel::Draft)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	hostLogInfo(i18n(QStringLiteral("Pick a planar face as neutral plane."),
					 QStringLiteral("\u8bf7\u70b9\u9009\u5e73\u9762\u9762\u4f5c\u4e2d\u6027\u9762\u3002")));
	geo->pickStepElementFromViewport(
		doc, req,
		[this, page, geo, doc](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			PluginSketchPlane plane;
			QString planeErr;
			if (!geo->queryFaceSketchPlane(doc, ref, plane, &planeErr) || !plane.isPlanar)
			{
				hostLogWarn(planeErr.isEmpty()
								? i18n(QStringLiteral("Face is not planar."), QStringLiteral("\u9762\u4e0d\u662f\u5e73\u9762\u3002"))
								: planeErr);
				return;
			}
			page->setDraftNeutralPlane(plane);
			refreshDraftPreview();
		});
}

void GeometricModelingPlugin::onConfirmDraft()
{
	GeometricModelingPage* page = ensurePageForActiveDocument();
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!page || !doc || !geo || m_pickedFaceIndices.empty())
	{
		hostLogWarn(i18n(QStringLiteral("Select at least one face."), QStringLiteral("\u8bf7\u81f3\u5c11\u9009\u62e9\u4e00\u4e2a\u9762\u3002")));
		return;
	}

	refreshDraftPreview();
	QByteArray beforeHist;
	QString qerr;
	(void)geo->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), beforeHist, &qerr);

	PluginSketchDraftParams params;
	params.angleDeg = page->draftAngleDeg();
	params.faceIndices = m_pickedFaceIndices;
	params.targetParametricBackendIdUtf8 = page->activeBodyId().toStdString();
	if (page->hasDraftNeutralPlane())
	{
		params.neutralPlane = page->draftNeutralPlane();
	}
	else
	{
		params.neutralPlane.isPlanar = true;
		params.neutralPlane.origin = {0, 0, 0};
		params.neutralPlane.normal = {0, 0, 1};
	}
	params.resultNameUtf8 = "ParametricBody";
	clearSolidFeaturePreviewUi();

	geo->draftFacesToBrep(
		doc, params,
		[this, page, doc, beforeHist](bool ok, const QString& err, const PluginGeometryJobResult& result)
		{
			if (!ok)
			{
				hostLogError(err);
				return;
			}
			page->setActiveBodyId(QString::fromStdString(result.newBackendId));
			refreshBodyList(page);
			syncFeaturesFromBody(page);
			QByteArray afterHist;
			QString qe;
			if (IPluginGeometryHost* g = m_host ? m_host->geometryHost() : nullptr)
				(void)g->queryParametricBodyHistoryJson(doc, page->activeBodyId().toStdString(), afterHist, &qe);
			const QByteArray beforeSnap =
				beforeHist.isEmpty() ? QByteArrayLiteral("{\"features\":[],\"seq\":1}") : beforeHist;
			page->commands().execute(std::make_unique<BodyHistoryCmd>(
				m_host, doc, page->activeBodyId(), beforeSnap, afterHist,
				[this, page]() { syncFeaturesFromBody(page); }, true));
			hostLogInfo(i18n(QStringLiteral("Draft feature created."), QStringLiteral("\u62d4\u6a21\u7279\u5f81\u5df2\u521b\u5efa\u3002")));
		});
}

void GeometricModelingPlugin::onCancelDraft()
{
	clearSolidFeaturePreviewUi();
	hostLogInfo(i18n(QStringLiteral("Draft cancelled."), QStringLiteral("\u62d4\u6a21\u5df2\u53d6\u6d88\u3002")));
}

void GeometricModelingPlugin::onDraft() { beginDraftPanel(); }

void GeometricModelingPlugin::onGeomTangent() { setActiveTool(SketchToolKind::GeomTangent); }
void GeometricModelingPlugin::onGeomSymmetric() { setActiveTool(SketchToolKind::GeomSymmetric); }
void GeometricModelingPlugin::onGeomMidpoint() { setActiveTool(SketchToolKind::GeomMidpoint); }

void GeometricModelingPlugin::onProjectEdges()
{
	if (!m_sketch.active())
	{
		hostLogWarn(i18n(QStringLiteral("Open a sketch first."), QStringLiteral("\u8bf7\u5148\u8fdb\u5165\u8349\u56fe\u7f16\u8f91\u3002")));
		return;
	}
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Edge;
	hostLogInfo(i18n(QStringLiteral("Pick a model edge to project."), QStringLiteral("\u8bf7\u70b9\u9009\u8981\u6295\u5f71\u7684\u6a21\u578b\u8fb9\u3002")));
	geo->pickStepElementFromViewport(
		doc, req,
		[this, geo, doc](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			const std::string path = ref.stepPathUtf8.empty() ? ref.backendIdUtf8 : ref.stepPathUtf8;
			PluginMeshDiscretizeParams meshParams;
			geo->discretizeBackendEdgesToPolylines(
				doc, path, meshParams,
				[this, ref](bool ok2, const QString& err2, const PluginGeometryJobResult& result)
				{
					if (!ok2 || result.polylines.empty())
					{
						hostLogWarn(err2.isEmpty() ? i18n(QStringLiteral("Edge discretize failed."),
														  QStringLiteral("\u8fb9\u79bb\u6563\u5316\u5931\u8d25\u3002"))
												   : err2);
						return;
					}
					if (ref.edgeIndex < 0 || static_cast<std::size_t>(ref.edgeIndex) >= result.polylines.size())
					{
						hostLogWarn(i18n(QStringLiteral("Edge index out of range."), QStringLiteral("\u8fb9\u7d22\u5f15\u8d85\u51fa\u8303\u56f4\u3002")));
						return;
					}
					const std::vector<float>& pl = result.polylines[static_cast<std::size_t>(ref.edgeIndex)];
					if (pl.size() < 6)
					{
						hostLogWarn(i18n(QStringLiteral("Edge too short."), QStringLiteral("\u8fb9\u8fc7\u77ed\u3002")));
						return;
					}
					SketchDocument2d& skDoc = m_sketch.document();
					const PluginSketchPlane plane = m_sketch.plane();
					const int added = projectPolylineToSketch(skDoc, plane, pl);
					(void)m_sketch.solveNow();
					m_sketch.refreshOverlay();
					persistActiveSketchDocument(ensurePageForActiveDocument());
					hostLogInfo(i18n(QStringLiteral("Projected %1 edge segment(s)."),
									 QStringLiteral("\u5df2\u6295\u5f71 %1 \u6761\u8fb9\u6bb5\u3002"))
									.arg(added));
				});
		});
}

void GeometricModelingPlugin::onConvertEntities()
{
	if (!m_sketch.active())
	{
		hostLogWarn(i18n(QStringLiteral("Open a sketch first."), QStringLiteral("\u8bf7\u5148\u8fdb\u5165\u8349\u56fe\u7f16\u8f91\u3002")));
		return;
	}
	IPluginDocument* doc = m_host ? m_host->activeDocument() : nullptr;
	IPluginGeometryHost* geo = m_host ? m_host->geometryHost() : nullptr;
	if (!doc || !geo)
		return;

	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	hostLogInfo(i18n(QStringLiteral("Pick a face to convert boundary edges."),
					 QStringLiteral("\u8bf7\u70b9\u9009\u8981\u8f6c\u6362\u7684\u9762\u3002")));
	geo->pickStepElementFromViewport(
		doc, req,
		[this, geo, doc](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				if (!err.isEmpty())
					hostLogInfo(err);
				return;
			}
			if (ref.faceIndex < 0 || ref.backendIdUtf8.empty())
			{
				hostLogWarn(i18n(QStringLiteral("Invalid face pick."), QStringLiteral("\u9762\u62fe\u53d6\u65e0\u6548\u3002")));
				return;
			}
			PluginMeshDiscretizeParams meshParams;
			geo->discretizeBackendFaceEdgesToPolylines(
				doc, ref, meshParams,
				[this](bool ok2, const QString& err2, const PluginGeometryJobResult& result)
				{
					if (!ok2 || (result.faceBoundarySegs.empty() && result.polylines.empty()))
					{
						hostLogWarn(err2.isEmpty() ? i18n(QStringLiteral("Face boundary discretize failed."),
														  QStringLiteral("\u9762\u8fb9\u754c\u79bb\u6563\u5316\u5931\u8d25\u3002"))
												   : err2);
						return;
					}
					SketchDocument2d& skDoc = m_sketch.document();
					const PluginSketchPlane plane = m_sketch.plane();
					int totalAdded = 0;
					int edgeCount = 0;
					int circleN = 0, arcN = 0, lineN = 0, polyN = 0;
					if (!result.faceBoundarySegs.empty())
					{
						for (const PluginFaceBoundarySeg& seg : result.faceBoundarySegs)
						{
							if (seg.kind == PluginFaceBoundarySegKind::Circle)
								++circleN;
							else if (seg.kind == PluginFaceBoundarySegKind::Arc)
								++arcN;
							else if (seg.kind == PluginFaceBoundarySegKind::Line)
								++lineN;
							else
								++polyN;
							const int n = projectBoundarySegToSketch(skDoc, plane, seg);
							if (n > 0)
							{
								totalAdded += n;
								++edgeCount;
							}
						}
					}
					else
					{
						for (const std::vector<float>& pl : result.polylines)
						{
							++polyN;
							const int n = projectPolylineToSketch(skDoc, plane, pl);
							if (n > 0)
							{
								totalAdded += n;
								++edgeCount;
							}
						}
					}
					if (totalAdded == 0)
					{
						hostLogWarn(i18n(QStringLiteral("No boundary edges projected onto sketch."),
										  QStringLiteral("\u672a\u80fd\u5c06\u9762\u8fb9\u754c\u6295\u5f71\u5230\u8349\u56fe\u3002")));
						return;
					}
					(void)m_sketch.solveNow();
					m_sketch.refreshOverlay();
					persistActiveSketchDocument(ensurePageForActiveDocument());
					hostLogInfo(i18n(QStringLiteral("Converted %1 edge(s): circle=%2 arc=%3 line=%4 polyline=%5, segments=%6."),
									 QStringLiteral("\u5df2\u8f6c\u6362 %1 \u6761\u8fb9\uff1a\u5706=%2 \u5f27=%3 \u7ebf=%4 \u6298\u7ebf=%5\uff0c\u7ebf\u6bb5=%6\u3002"))
									.arg(edgeCount)
									.arg(circleN)
									.arg(arcN)
									.arg(lineN)
									.arg(polyN)
									.arg(totalAdded));
					if (polyN > 0)
						hostLogWarn(i18n(QStringLiteral("%1 edge(s) fell back to polyline (not circle/arc)."),
										 QStringLiteral("%1 \u6761\u8fb9\u56de\u9000\u4e3a\u6298\u7ebf\uff08\u975e\u5706/\u5f27\uff09\u3002"))
										.arg(polyN));
				});
		});
}

void GeometricModelingPlugin::onOffset()
{
	if (!m_sketch.active())
	{
		hostLogWarn(i18n(QStringLiteral("Open a sketch first."), QStringLiteral("\u8bf7\u5148\u8fdb\u5165\u8349\u56fe\u7f16\u8f91\u3002")));
		return;
	}
	SketchDocument2d& skDoc = m_sketch.document();
	std::vector<std::vector<SkVec2>> loops;
	std::string err;
	if (!skDoc.exportClosedProfilesUv(loops, &err) || loops.empty())
	{
		hostLogWarn(i18n(QStringLiteral("No closed profile to offset."),
						 QStringLiteral("\u65e0\u53ef\u7b49\u8ddd\u7684\u95ed\u5408\u8f6e\u5ed3\u3002")));
		return;
	}

	bool ok = false;
	const double dist =
		QInputDialog::getDouble(nullptr,
								i18n(QStringLiteral("Offset"), QStringLiteral("\u7b49\u8ddd")),
								i18n(QStringLiteral("Offset distance (mm, + outward for outer / shrink hole):"),
									 QStringLiteral("\u7b49\u8ddd\u8ddd\u79bb\uff08mm\uff0c\u6b63\u5411\uff1a\u5916\u73af\u5411\u5916\uff0f\u5b54\u73af\u7f29\u5c0f\uff09\uff1a")),
								1.0, -1e6, 1e6, 2, &ok);
	if (!ok)
		return;

	int loopOk = 0;
	for (std::size_t li = 0; li < loops.size(); ++li)
	{
		// 孔环取反号：正距表示外扩外环、缩小孔
		const double loopDist = (li == 0) ? dist : -dist;
		std::vector<SkVec2> offset;
		std::string localErr;
		if (!offsetClosedUv(loops[li], loopDist, offset, &localErr))
		{
			hostLogWarn(QString::fromStdString(localErr.empty() ? "offset failed" : localErr));
			continue;
		}
		if (offset.size() < 3)
		{
			hostLogWarn(i18n(QStringLiteral("Offset result too small."), QStringLiteral("\u7b49\u8ddd\u7ed3\u679c\u8fc7\u5c0f\u3002")));
			continue;
		}
		if (closedPolylineSelfIntersectsUv(offset))
		{
			hostLogWarn(i18n(QStringLiteral("Offset self-intersects; rejected (loop %1)."),
							 QStringLiteral("\u7b49\u8ddd\u81ea\u4ea4\uff0c\u5df2\u62d2\u7edd\uff08\u73af %1\uff09\u3002"))
							.arg(static_cast<int>(li) + 1));
			continue;
		}

		std::vector<int> ptIds;
		ptIds.reserve(offset.size());
		for (const SkVec2& uv : offset)
			ptIds.push_back(skDoc.addPoint(uv.u, uv.v));
		for (std::size_t i = 0; i < ptIds.size(); ++i)
		{
			const std::size_t j = (i + 1) % ptIds.size();
			skDoc.addLine(ptIds[i], ptIds[static_cast<std::size_t>(j)], false);
		}
		++loopOk;
	}

	if (loopOk == 0)
	{
		hostLogWarn(i18n(QStringLiteral("No offset loop created."), QStringLiteral("\u672a\u751f\u6210\u4efb\u4f55\u7b49\u8ddd\u8f6e\u5ed3\u3002")));
		return;
	}

	(void)m_sketch.solveNow();
	m_sketch.refreshOverlay();
	persistActiveSketchDocument(ensurePageForActiveDocument());
	hostLogInfo(i18n(QStringLiteral("Offset %1 loop(s) created (%2 mm)."),
					 QStringLiteral("\u5df2\u751f\u6210 %1 \u4e2a\u7b49\u8ddd\u8f6e\u5ed3\uff08%2 mm\uff09\u3002"))
					.arg(loopOk)
					.arg(dist));
}
