/// @file PluginGeometryHostImpl.cpp
/// @brief PluginGeometryHostImpl 实现

#include "PluginGeometryHostImpl.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendTypeIds.h"
#include "BrepBackendData.h"
#include "DocumentGeometryOps.h"
#include "DocumentHost.h"
#include "GeometryBackendOps.h"
#include "GeometryRef.h"
#include "OsgWidget.h"
#include "ParametricBrepBackendData.h"
#include "PickTypes.h"
#include "PluginDocumentAdapter.h"
#include "PluginHostContext.h"
#include "ShapeHandle.h"
#include "ShapeQuery.h"
#include "SketchExtrude.h"
#include "SketchFillet.h"
#include "SketchLoft.h"
#include "SketchPattern.h"
#include "SketchRevolve.h"
#include "SketchDraft.h"
#include "SketchShell.h"
#include "SketchSweep.h"
#include "SketchCurveWire.h"
#include "MeshDiscretize.h"
#include "HlrProject.h"
#include "SketchPlane.h"
#include "WidgetDocumentAccess.h"

#include <QByteArray>
#include <QEvent>
#include <QHash>
#include <QKeyEvent>
#include <QLatin1String>
#include <QMetaObject>
#include <QMouseEvent>
#include <QSet>
#include <QString>
#include <QTimer>
#include <QVector>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include <FeatureSpec.h>
#include <RobotOsgUiTypes.h>
#include <json.hpp>

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

bool isStepPath(const QString& path)
{
	const QString p = path.trimmed().toLower();
	return p.endsWith(QStringLiteral(".step")) || p.endsWith(QStringLiteral(".stp"));
}

bool isTopLevelWorkpieceBackend(const BackendDataManager& mgr, const std::string& backendId)
{
	return mgr.parentsOf(backendId).empty();
}

geoalgo::SketchCurveSegment toCurveSeg(const PluginSketchSweepPathSegment& p)
{
	geoalgo::SketchCurveSegment g;
	switch (p.kind)
	{
	case PluginSketchSweepPathSegKind::Arc:
		g.kind = geoalgo::SketchCurveSegKind::Arc;
		break;
	case PluginSketchSweepPathSegKind::SplineThrough:
		g.kind = geoalgo::SketchCurveSegKind::SplineThrough;
		break;
	case PluginSketchSweepPathSegKind::Circle:
		g.kind = geoalgo::SketchCurveSegKind::Circle;
		break;
	case PluginSketchSweepPathSegKind::Ellipse:
		g.kind = geoalgo::SketchCurveSegKind::Ellipse;
		break;
	default:
		g.kind = geoalgo::SketchCurveSegKind::Line;
		break;
	}
	g.ax = p.ax;
	g.ay = p.ay;
	g.az = p.az;
	g.bx = p.bx;
	g.by = p.by;
	g.bz = p.bz;
	g.mx = p.mx;
	g.my = p.my;
	g.mz = p.mz;
	return g;
}

void appendProfileSegments(geoalgo::SketchExtrudeParams& ep, const std::vector<PluginSketchSweepPathSegment>& segs)
{
	ep.profileSegments.clear();
	ep.profileSegments.reserve(segs.size());
	for (const auto& p : segs)
		ep.profileSegments.push_back(toCurveSeg(p));
}

void appendProfileSegments(geoalgo::SketchSweepParams& sp, const std::vector<PluginSketchSweepPathSegment>& segs)
{
	sp.profileSegments.clear();
	sp.profileSegments.reserve(segs.size());
	for (const auto& p : segs)
		sp.profileSegments.push_back(toCurveSeg(p));
}

void storeProfileSegments(ParametricFeature& feat, const std::vector<PluginSketchSweepPathSegment>& segs)
{
	feat.profileSegments.clear();
	feat.profileSegments.reserve(segs.size());
	for (const auto& p : segs)
	{
		ParametricFeature::PathSegment s;
		s.kind = static_cast<int>(p.kind);
		s.ax = p.ax;
		s.ay = p.ay;
		s.az = p.az;
		s.bx = p.bx;
		s.by = p.by;
		s.bz = p.bz;
		s.mx = p.mx;
		s.my = p.my;
		s.mz = p.mz;
		feat.profileSegments.push_back(s);
	}
}

struct ComputableBackendCandidate
{
	PluginGeometryBackendEntry entry;
	QString dedupeKey;
	bool isBrepModel = false;
};

QString stepPathForBackend(cloudsim::host::DocumentHost* page, const std::string& backendIdUtf8)
{
	if (!page || backendIdUtf8.empty())
	{
		return QString();
	}
	return page->backendSourcePath().value(QString::fromStdString(backendIdUtf8));
}

bool worldPointToStepModelMm(OsgWidget* osg, const std::string& backendIdUtf8, const osg::Vec3f& worldMm,
							 geoalgo::Point3d& outModel, std::string* errMsg = nullptr)
{
	if (!osg)
	{
		if (errMsg)
		{
			*errMsg = "no osg host";
		}
		return false;
	}
	osg::Matrixd worldMat;
	if (!osg->getBackendRootWorldMatrix(backendIdUtf8, worldMat))
	{
		if (errMsg)
		{
			*errMsg = "backend world matrix unavailable";
		}
		return false;
	}
	osg::Matrixd invMat;
	if (!invMat.invert(worldMat))
	{
		if (errMsg)
		{
			*errMsg = "failed to invert backend matrix";
		}
		return false;
	}
	const osg::Vec3d pw(static_cast<double>(worldMm.x()), static_cast<double>(worldMm.y()),
						static_cast<double>(worldMm.z()));
	const osg::Vec3d pOuter = pw * invMat;
	double cx = 0.0;
	double cy = 0.0;
	double cz = 0.0;
	(void)osg->tryGetBackendModelCenterMm(backendIdUtf8, cx, cy, cz);
	outModel.x = pOuter.x() + cx;
	outModel.y = pOuter.y() + cy;
	outModel.z = pOuter.z() + cz;
	return true;
}

struct MeshWorkResult
{
	std::vector<float> soup;
	geoalgo::MeshDiscretizeReport report;
	std::string error;
	bool ok = false;
};

struct IntersectWorkResult
{
	geoalgo::IntersectionResult intersection;
	std::vector<geoalgo::Polyline3d> polylines;
	std::vector<geoalgo::FaceBoundarySeg> faceBoundarySegs;
	std::string error;
	bool ok = false;
};

void runMeshJob(PluginHostContext* host, IPluginDocument* doc, const QString& jobTitle,
				std::function<bool(MeshWorkResult&)> work, const PluginMeshCreateOptions& options,
				PluginGeometryFinishedFn onFinished)
{
	if (!host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page)
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	auto result = std::make_shared<MeshWorkResult>();
	host->enqueueJob(
		jobTitle,
		[result, work = std::move(work)](const PluginJobProgressFn& report)
		{
			report(0.2, QStringLiteral("Running..."));
			result->ok = work(*result);
			report(1.0, QStringLiteral("Done"));
		},
		[host, page, result, options, onFinished = std::move(onFinished)](const bool threw, const QString& throwMessage)
		{
			PluginGeometryJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			std::string regErr;
			const std::string backendId = document_geometry_ops::registerMeshSoup(
				page, host->mainWindowHost(), std::move(result->soup), options, &regErr);
			if (backendId.empty())
			{
				onFinished(false, QString::fromStdString(regErr), jobResult);
				return;
			}
			jobResult = document_geometry_ops::toPluginGeometryResult(result->report, backendId);
			onFinished(true, QString(), jobResult);
		});
}

void runIntersectJob(PluginHostContext* host, IPluginDocument* doc, const QString& jobTitle,
					 std::function<bool(IntersectWorkResult&)> work, PluginGeometryFinishedFn onFinished)
{
	if (!host || !onFinished)
	{
		return;
	}
	if (!pageFromDoc(doc))
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	auto result = std::make_shared<IntersectWorkResult>();
	host->enqueueJob(
		jobTitle,
		[result, work = std::move(work)](const PluginJobProgressFn& report)
		{
			report(0.2, QStringLiteral("Running..."));
			result->ok = work(*result);
			report(1.0, QStringLiteral("Done"));
		},
		[result, onFinished = std::move(onFinished)](const bool threw, const QString& throwMessage)
		{
			PluginGeometryJobResult jobResult;
			if (threw)
			{
				onFinished(false, throwMessage, jobResult);
				return;
			}
			if (!result->ok)
			{
				onFinished(false, QString::fromStdString(result->error), jobResult);
				return;
			}
			jobResult = document_geometry_ops::toPluginGeometryResult(result->intersection);
			for (const geoalgo::Polyline3d& poly : result->polylines)
			{
				jobResult.polylines.push_back(poly.xyz);
			}
			for (const geoalgo::FaceBoundarySeg& seg : result->faceBoundarySegs)
			{
				PluginFaceBoundarySeg p;
				p.kind = static_cast<PluginFaceBoundarySegKind>(static_cast<int>(seg.kind));
				p.xyz = seg.xyz;
				p.radiusMm = seg.radiusMm;
				jobResult.faceBoundarySegs.push_back(std::move(p));
			}
			onFinished(true, QString(), jobResult);
		});
}

} // namespace

PluginGeometryHostImpl::PluginGeometryHostImpl(PluginHostContext* hostContext) : m_host(hostContext)
{
	std::string err;
	if (!ParametricBrepBackendData::runParametricHistorySelfTest(&err))
	{
		if (m_host)
			m_host->logError(QStringLiteral("ParametricBrep self-test failed: %1").arg(QString::fromStdString(err)));
	}
	else if (m_host)
	{
		m_host->logInfo(QStringLiteral("ParametricBrep Pad/Pocket/MidPlane+draft/Sweep/history self-test OK."));
	}
}

void PluginGeometryHostImpl::discretizeStepToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
												  const PluginMeshDiscretizeParams& params,
												  const PluginMeshCreateOptions& options,
												  PluginGeometryFinishedFn onFinished)
{
	const geoalgo::MeshDiscretizeParams geoParams = document_geometry_ops::toGeoMeshParams(params);
	runMeshJob(
		m_host, doc, QStringLiteral("STEP discretize"),
		[stepPathUtf8, geoParams](MeshWorkResult& out) {
			return geometry_backend_ops::discretizeStepToMesh(stepPathUtf8, geoParams, out.soup, out.report,
															  &out.error);
		},
		options, std::move(onFinished));
}

void PluginGeometryHostImpl::discretizeBackendToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
													 const PluginMeshDiscretizeParams& params,
													 const PluginMeshCreateOptions& options,
													 PluginGeometryFinishedFn onFinished)
{
	discretizeStepToMesh(doc, stepPathUtf8, params, options, std::move(onFinished));
}

void PluginGeometryHostImpl::discretizeBackendFaceToMesh(IPluginDocument* doc, const PluginGeometryStepRef& faceRef,
														 const PluginMeshDiscretizeParams& params,
														 const PluginMeshCreateOptions& options,
														 PluginGeometryFinishedFn onFinished)
{
	const geoalgo::MeshDiscretizeParams geoParams = document_geometry_ops::toGeoMeshParams(params);
	runMeshJob(
		m_host, doc, QStringLiteral("Face discretize"),
		[faceRef, geoParams](MeshWorkResult& out)
		{
			return geometry_backend_ops::discretizeStepFaceToMesh(faceRef.stepPathUtf8, faceRef.faceIndex, geoParams,
																  out.soup, out.report, &out.error);
		},
		options, std::move(onFinished));
}

void PluginGeometryHostImpl::discretizeWireToTubeMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
													  const PluginMeshDiscretizeParams& params,
													  const PluginMeshCreateOptions& options,
													  PluginGeometryFinishedFn onFinished)
{
	PluginMeshDiscretizeParams tubeParams = params;
	tubeParams.mode = PluginMeshDiscretizeMode::WireTubeMesh;
	discretizePolylineToMesh(doc, polylineXyz, tubeParams, options, std::move(onFinished));
}

void PluginGeometryHostImpl::discretizeWireToRibbonMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
														const PluginMeshDiscretizeParams& params,
														const PluginMeshCreateOptions& options,
														PluginGeometryFinishedFn onFinished)
{
	PluginMeshDiscretizeParams ribbonParams = params;
	ribbonParams.mode = PluginMeshDiscretizeMode::WireRibbonMesh;
	discretizePolylineToMesh(doc, polylineXyz, ribbonParams, options, std::move(onFinished));
}

void PluginGeometryHostImpl::discretizePolylineToMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
													  const PluginMeshDiscretizeParams& params,
													  const PluginMeshCreateOptions& options,
													  PluginGeometryFinishedFn onFinished)
{
	const geoalgo::MeshDiscretizeParams geoParams = document_geometry_ops::toGeoMeshParams(params);
	runMeshJob(
		m_host, doc, QStringLiteral("Polyline mesh"),
		[polylineXyz, geoParams](MeshWorkResult& out)
		{
			const bool ok =
				geometry_backend_ops::discretizePolylineToMesh(polylineXyz, geoParams, out.soup, &out.error);
			if (ok)
			{
				geometry_backend_ops::fillMeshReport(out.soup, out.report);
			}
			return ok;
		},
		options, std::move(onFinished));
}

void PluginGeometryHostImpl::discretizeBackendEdgesToPolylines(IPluginDocument* doc, const std::string& stepPathUtf8,
															   const PluginMeshDiscretizeParams& params,
															   PluginGeometryFinishedFn onFinished)
{
	const geoalgo::MeshDiscretizeParams geoParams = document_geometry_ops::toGeoMeshParams(params);
	runIntersectJob(
		m_host, doc, QStringLiteral("Edge polylines"),
		[stepPathUtf8, geoParams](IntersectWorkResult& out)
		{
			return geometry_backend_ops::discretizeStepEdgesToPolylines(stepPathUtf8, geoParams.tessellate,
																		out.polylines, &out.error);
		},
		std::move(onFinished));
}

void PluginGeometryHostImpl::discretizeBackendFaceEdgesToPolylines(IPluginDocument* doc,
																  const PluginGeometryStepRef& faceRef,
																  const PluginMeshDiscretizeParams& params,
																  PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || faceRef.faceIndex < 0 || faceRef.backendIdUtf8.empty())
	{
		onFinished(false, QStringLiteral("Invalid face ref for boundary edges"), {});
		return;
	}
	auto brep = std::dynamic_pointer_cast<BrepBackendData>(page->backend().getData(faceRef.backendIdUtf8));
	if (!brep || brep->worldShape().isNull())
	{
		onFinished(false, QStringLiteral("B-rep backend unavailable"), {});
		return;
	}
	const geoalgo::ShapeHandle shape = brep->worldShape();
	const int faceIndex = faceRef.faceIndex;
	const geoalgo::TessellateParams tess = document_geometry_ops::toGeoMeshParams(params).tessellate;
	runIntersectJob(
		m_host, doc, QStringLiteral("Face boundary edges"),
		[shape, faceIndex, tess](IntersectWorkResult& out)
		{
			if (!geoalgo::extractShapeFaceBoundarySegments(shape, faceIndex, tess, out.faceBoundarySegs, &out.error))
				return false;
			std::string polyErr;
			(void)geoalgo::discretizeShapeFaceEdgesToPolylines(shape, faceIndex, tess, out.polylines, &polyErr);
			return true;
		},
		std::move(onFinished));
}

void PluginGeometryHostImpl::intersectEdges(IPluginDocument* doc, const PluginGeometryStepRef& edge1,
											const PluginGeometryStepRef& edge2,
											const PluginGeometryIntersectionParams& params,
											PluginGeometryFinishedFn onFinished)
{
	const geoalgo::IntersectionParams geoParams = document_geometry_ops::toGeoIntersectionParams(params);
	const std::string path = edge1.stepPathUtf8.empty() ? edge2.stepPathUtf8 : edge1.stepPathUtf8;
	runIntersectJob(
		m_host, doc, QStringLiteral("Edge-edge intersect"),
		[path, edge1, edge2, geoParams](IntersectWorkResult& out)
		{
			return geometry_backend_ops::intersectStepEdges(path, edge1.edgeIndex, edge2.edgeIndex, geoParams,
															out.intersection, &out.error);
		},
		std::move(onFinished));
}

void PluginGeometryHostImpl::intersectEdgeFace(IPluginDocument* doc, const PluginGeometryStepRef& edgeRef,
											   const PluginGeometryStepRef& faceRef,
											   const PluginGeometryIntersectionParams& params,
											   PluginGeometryFinishedFn onFinished)
{
	const geoalgo::IntersectionParams geoParams = document_geometry_ops::toGeoIntersectionParams(params);
	const std::string path = edgeRef.stepPathUtf8.empty() ? faceRef.stepPathUtf8 : edgeRef.stepPathUtf8;
	runIntersectJob(
		m_host, doc, QStringLiteral("Edge-face intersect"),
		[path, edgeRef, faceRef, geoParams](IntersectWorkResult& out)
		{
			return geometry_backend_ops::intersectStepEdgeFace(path, edgeRef.edgeIndex, faceRef.faceIndex, geoParams,
															   out.intersection, &out.error);
		},
		std::move(onFinished));
}

void PluginGeometryHostImpl::intersectFaces(IPluginDocument* doc, const PluginGeometryStepRef& face1,
											const PluginGeometryStepRef& face2,
											const PluginGeometryIntersectionParams& params,
											PluginGeometryFinishedFn onFinished)
{
	const geoalgo::IntersectionParams geoParams = document_geometry_ops::toGeoIntersectionParams(params);
	const std::string path = face1.stepPathUtf8.empty() ? face2.stepPathUtf8 : face1.stepPathUtf8;
	runIntersectJob(
		m_host, doc, QStringLiteral("Face-face intersect"),
		[path, face1, face2, geoParams](IntersectWorkResult& out)
		{
			return geometry_backend_ops::intersectStepFaces(path, face1.faceIndex, face2.faceIndex, geoParams,
															out.intersection, &out.error);
		},
		std::move(onFinished));
}

void PluginGeometryHostImpl::intersectBackends(IPluginDocument* doc, const std::string& targetStepPathUtf8,
											   const std::string& toolStepPathUtf8,
											   const PluginGeometryIntersectionParams& params,
											   PluginGeometryFinishedFn onFinished)
{
	const geoalgo::IntersectionParams geoParams = document_geometry_ops::toGeoIntersectionParams(params);
	runIntersectJob(
		m_host, doc, QStringLiteral("Shape intersect"),
		[targetStepPathUtf8, toolStepPathUtf8, geoParams](IntersectWorkResult& out)
		{
			return geometry_backend_ops::intersectStepFiles(targetStepPathUtf8, toolStepPathUtf8, geoParams,
															out.intersection, &out.error);
		},
		std::move(onFinished));
}

void PluginGeometryHostImpl::brepBooleanToMesh(IPluginDocument* doc, const std::string& targetStepPathUtf8,
											   const std::string& toolStepPathUtf8,
											   const PluginGeometryBrepBooleanParams& params,
											   PluginGeometryFinishedFn onFinished)
{
	const geoalgo::MeshDiscretizeParams geoMesh = document_geometry_ops::toGeoMeshParams(params.meshParams);
	const geoalgo::BrepBooleanOp geoOp = document_geometry_ops::toGeoBrepBooleanOp(params.op);
	runMeshJob(
		m_host, doc, QStringLiteral("B-rep boolean"),
		[targetStepPathUtf8, toolStepPathUtf8, geoOp, geoMesh](MeshWorkResult& out)
		{
			const bool ok = geometry_backend_ops::brepBooleanStepFilesToMesh(targetStepPathUtf8, toolStepPathUtf8,
																			 geoOp, geoMesh, out.soup, &out.error);
			if (ok)
			{
				geometry_backend_ops::fillMeshReport(out.soup, out.report);
			}
			return ok;
		},
		params.resultOptions, std::move(onFinished));
}

void PluginGeometryHostImpl::fuseWiresToPolyline(IPluginDocument* doc, const std::string& stepPathUtf8,
												 const std::vector<int>& edgeIndices,
												 const PluginGeometryIntersectionParams& params,
												 PluginGeometryFinishedFn onFinished)
{
	geoalgo::TessellateParams disc;
	disc.linearDeflectionMm = params.curveLinearDeflectionMm;
	runIntersectJob(
		m_host, doc, QStringLiteral("Fuse wires"),
		[stepPathUtf8, edgeIndices, disc](IntersectWorkResult& out)
		{
			geoalgo::Polyline3d poly;
			if (!geometry_backend_ops::fuseStepEdgesToPolyline(stepPathUtf8, edgeIndices, disc, poly, &out.error))
			{
				return false;
			}
			out.polylines.push_back(std::move(poly));
			return true;
		},
		std::move(onFinished));
}

void PluginGeometryHostImpl::sewFacesToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
											const std::vector<int>& faceIndices,
											const PluginMeshDiscretizeParams& meshParams,
											const PluginMeshCreateOptions& options, PluginGeometryFinishedFn onFinished)
{
	const geoalgo::MeshDiscretizeParams geoParams = document_geometry_ops::toGeoMeshParams(meshParams);
	runMeshJob(
		m_host, doc, QStringLiteral("Sew faces"),
		[stepPathUtf8, faceIndices, geoParams](MeshWorkResult& out)
		{
			const bool ok = geometry_backend_ops::sewStepFacesToMesh(stepPathUtf8, faceIndices, 1e-3, geoParams,
																	 out.soup, &out.error);
			if (ok)
			{
				geometry_backend_ops::fillMeshReport(out.soup, out.report);
			}
			return ok;
		},
		options, std::move(onFinished));
}

bool PluginGeometryHostImpl::listComputableBackends(IPluginDocument* doc,
													std::vector<PluginGeometryBackendEntry>& outBackends,
													QString* outError)
{
	outBackends.clear();
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page)
	{
		if (outError)
		{
			*outError = QStringLiteral("No active document");
		}
		return false;
	}

	const std::vector<std::shared_ptr<BackendDataBase>> all = page->backend().listData();
	BackendDataManager& mgr = page->backend();
	QVector<ComputableBackendCandidate> candidates;
	candidates.reserve(static_cast<int>(all.size()));
	for (const std::shared_ptr<BackendDataBase>& data : all)
	{
		if (!data)
		{
			continue;
		}
		const std::string backendId = data->id();
		if (!isTopLevelWorkpieceBackend(mgr, backendId))
		{
			continue;
		}
		const QString stepPath = stepPathForBackend(page, backendId);
		const bool isBrepModel = isBrepWorkpieceClassName(data->className());
		if (isBrepModel)
		{
			if (!data->hasGeometry())
			{
				continue;
			}
		}
		else
		{
			if (!data->hasGeometry() || !isStepPath(stepPath))
			{
				continue;
			}
		}

		ComputableBackendCandidate candidate;
		candidate.entry.backendId = backendId;
		candidate.entry.displayName = data->name();
		candidate.entry.className = data->className();
		candidate.entry.stepPathUtf8 = stepPath.toStdString();
		candidate.entry.pickable = true;
		candidate.dedupeKey = stepPath.isEmpty() ? QString::fromStdString(backendId) : stepPath.toLower();
		candidate.isBrepModel = isBrepModel;
		candidates.append(candidate);
	}

	QHash<QString, int> bestIndexByKey;
	for (int i = 0; i < candidates.size(); ++i)
	{
		const ComputableBackendCandidate& candidate = candidates[i];
		if (!bestIndexByKey.contains(candidate.dedupeKey))
		{
			bestIndexByKey.insert(candidate.dedupeKey, i);
			continue;
		}
		const int prev = bestIndexByKey.value(candidate.dedupeKey);
		if (!candidates[prev].isBrepModel && candidate.isBrepModel)
		{
			bestIndexByKey[candidate.dedupeKey] = i;
		}
	}

	QSet<int> keepIndices;
	for (int idx : bestIndexByKey)
	{
		keepIndices.insert(idx);
	}

	outBackends.reserve(static_cast<std::size_t>(keepIndices.size()));
	for (int i = 0; i < candidates.size(); ++i)
	{
		if (!keepIndices.contains(i))
		{
			continue;
		}
		outBackends.push_back(std::move(candidates[i].entry));
	}

	if (outBackends.empty() && outError)
	{
		*outError = QStringLiteral("No STEP/BRep backends available");
	}
	return !outBackends.empty();
}

void PluginGeometryHostImpl::pickStepElementFromViewport(IPluginDocument* doc,
														 const PluginGeometryElementPickRequest& request,
														 PluginGeometryElementPickedFn onFinished)
{
	if (!onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	OsgWidget* osg = widgetOsgFromPage(page);
	if (!osg)
	{
		onFinished(false, QStringLiteral("3D viewport unavailable"), {});
		return;
	}

	const bool pickFace = request.kind == PluginGeometryElementKind::Face;
	const bool pickVertex = request.kind == PluginGeometryElementKind::Vertex;
	osg->setSelectionActive(true);
	if (!request.backendIdUtf8.empty())
	{
		osg->syncSelectionForBackendId(request.backendIdUtf8);
	}
	osg->setMeshLinePickMode(!pickFace);
	osg->setMeshFacePickMode(pickFace);

	struct PickState
	{
		bool done = false;
		QMetaObject::Connection conn;
	};
	const auto state = std::make_shared<PickState>();
	const auto complete = [state, osg, onFinished](bool ok, const QString& err, const PluginGeometryStepRef& ref)
	{
		if (state->done)
		{
			return;
		}
		state->done = true;
		QObject::disconnect(state->conn);
		if (osg)
		{
			osg->setMeshLinePickMode(false);
			osg->setMeshFacePickMode(false);
		}
		onFinished(ok, err, ref);
	};

	state->conn = QObject::connect(
		osg, &OsgWidget::meshPickCommitted, m_host,
		[=](PickResult pick, int pickKindInt)
		{
			const PickKind kind = static_cast<PickKind>(pickKindInt);
			if (!pick.hit)
			{
				return;
			}
			if (pickFace && kind != PickKind::MeshFace)
			{
				return;
			}
			if (!pickFace && kind != PickKind::MeshEdge)
			{
				return;
			}
			if (!request.backendIdUtf8.empty() && pick.backendId != request.backendIdUtf8)
			{
				return;
			}

			const std::string backendId = request.backendIdUtf8.empty() ? pick.backendId : request.backendIdUtf8;
			QString stepPath = QString::fromStdString(request.stepPathUtf8);
			if (stepPath.isEmpty())
			{
				stepPath = stepPathForBackend(page, backendId);
			}
			// 导入 STEP 或内存 B-rep（ParametricBrep / geomodel://）均可
			const bool stepFile = isStepPath(stepPath);
			std::shared_ptr<BrepBackendData> inMemoryBrep;
			if (!stepFile)
			{
				inMemoryBrep = std::dynamic_pointer_cast<BrepBackendData>(page->backend().getData(backendId));
				if (!inMemoryBrep || !inMemoryBrep->hasGeometry())
				{
					complete(false, QStringLiteral("STEP source path unavailable"), {});
					return;
				}
			}

			// 原生 B-rep 面拾取：直接返回 faceIndex（拉伸体无 STEP 时走此路径）
			if (pickFace && pick.brepNativePick && pick.brepFaceIndex >= 0)
			{
				PluginGeometryStepRef outRef;
				outRef.backendIdUtf8 = backendId;
				outRef.stepPathUtf8 = stepPath.toStdString();
				outRef.faceIndex = pick.brepFaceIndex;
				outRef.hasHitPoint = true;
				outRef.hitWorldMm = {pick.worldPoint.x(), pick.worldPoint.y(), pick.worldPoint.z()};
				complete(true, QString(), outRef);
				return;
			}

			geoalgo::Point3d modelA{};
			geoalgo::Point3d modelB{};
			std::string err;
			if (!worldPointToStepModelMm(osg, backendId, pick.worldPoint, modelA, &err))
			{
				complete(false, QString::fromStdString(err), {});
				return;
			}
			if (pickFace)
			{
				modelB = modelA;
			}
			else
			{
				if (!worldPointToStepModelMm(osg, backendId, pick.meshEdgeA, modelA, &err) ||
					!worldPointToStepModelMm(osg, backendId, pick.meshEdgeB, modelB, &err))
				{
					complete(false, QString::fromStdString(err), {});
					return;
				}
			}

			geometry_backend_ops::GeometryRef ref;
			ref.backendIdUtf8 = backendId;
			ref.stepPathUtf8 = stepPath.toStdString();
			geoalgo::WorkpieceRef wp;
			if (!geometry_backend_ops::resolveGeometryRef(ref, wp, &err))
			{
				complete(false, QString::fromStdString(err), {});
				return;
			}

			geoalgo::ShapeHandle shape;
			geoalgo::WorkpieceRef shapeRef;
			if (geometry_backend_ops::resolveWorkpieceShape(backendId, page->backend(), stepPath.toStdString(), shape,
															shapeRef, &err) ==
				geometry_backend_ops::WorkpieceShapeSource::Unavailable)
			{
				complete(false, QString::fromStdString(err), {});
				return;
			}

			const std::string strategyId = pickFace ? "FaceBoundary" : "EdgeChain";
			geoalgo::FeatureEntry entry;
			const int knownFaceIndex = pick.brepNativePick && pickFace ? pick.brepFaceIndex : -1;
			const int knownEdgeIndex = pick.brepNativePick && !pickFace ? pick.brepEdgeIndex : -1;
			if (!geometry_backend_ops::buildFeatureEntryFromModelPick(wp, shape, strategyId, pickFace, modelA, modelB,
																	  entry, &err, knownFaceIndex, knownEdgeIndex))
			{
				complete(false, QString::fromStdString(err), {});
				return;
			}

			PluginGeometryStepRef outRef;
			outRef.backendIdUtf8 = backendId;
			outRef.stepPathUtf8 = ref.stepPathUtf8;
			outRef.hasHitPoint = true;
			outRef.hitWorldMm = {pick.worldPoint.x(), pick.worldPoint.y(), pick.worldPoint.z()};
			if (pickFace)
			{
				if (entry.geometry.faceIndices.empty())
				{
					complete(false, QStringLiteral("Failed to resolve face index"), {});
					return;
				}
				outRef.faceIndex = entry.geometry.faceIndices.front();
			}
			else
			{
				if (entry.geometry.edgeIndices.empty())
				{
					complete(false, QStringLiteral("Failed to resolve edge index"), {});
					return;
				}
				outRef.edgeIndex = entry.geometry.edgeIndices.front();
				{
					geoalgo::Point3d ea, eb;
					if (geoalgo::shapeHandleEdgeEndpoints(shape, outRef.edgeIndex, ea, eb, nullptr))
					{
						outRef.edgeEndAMm = {static_cast<float>(ea.x), static_cast<float>(ea.y),
											 static_cast<float>(ea.z)};
						outRef.edgeEndBMm = {static_cast<float>(eb.x), static_cast<float>(eb.y),
											 static_cast<float>(eb.z)};
						outRef.hasEdgeEnds = true;
					}
				}
				// Vertex：优先 TopExp 顶点；失败再吸附 mesh 边端点
				if (pickVertex)
				{
					geoalgo::Point3d q{pick.worldPoint.x(), pick.worldPoint.y(), pick.worldPoint.z()};
					geoalgo::Point3d vMm;
					int vIdx = -1;
					if (geoalgo::pickShapeVertexByModelPoint(shape, q, 2.0, vIdx, vMm, nullptr))
					{
						outRef.vertexIndex = vIdx;
						outRef.hitWorldMm = {vMm.x, vMm.y, vMm.z};
						outRef.hasHitPoint = true;
					}
					else
					{
						const osg::Vec3f& a = pick.meshEdgeA;
						const osg::Vec3f& b = pick.meshEdgeB;
						const osg::Vec3f& p = pick.worldPoint;
						const float dA = (a - p).length2();
						const float dB = (b - p).length2();
						const osg::Vec3f& ep = (dA <= dB) ? a : b;
						outRef.hitWorldMm = {ep.x(), ep.y(), ep.z()};
					}
				}
			}
			complete(true, QString(), outRef);
		});

	QTimer::singleShot(30000, m_host, [complete]() { complete(false, QStringLiteral("Pick timeout"), {}); });
}

namespace
{
ParametricSketchPlane toParametricPlane(const PluginSketchPlane& plane)
{
	ParametricSketchPlane p;
	p.originX = plane.origin.x;
	p.originY = plane.origin.y;
	p.originZ = plane.origin.z;
	p.axisXX = plane.axisX.x;
	p.axisXY = plane.axisX.y;
	p.axisXZ = plane.axisX.z;
	p.axisYX = plane.axisY.x;
	p.axisYY = plane.axisY.y;
	p.axisYZ = plane.axisY.z;
	p.normalX = plane.normal.x;
	p.normalY = plane.normal.y;
	p.normalZ = plane.normal.z;
	p.isPlanar = plane.isPlanar;
	return p;
}

bool refreshParametricBodyScene(cloudsim::host::DocumentHost* page, ParametricBrepBackendData& body, QString* outError)
{
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return true;
	// 仅剩草图时 tip 为空：卸掉场景节点，否则删除特征后旧网格残留
	if (!body.hasGeometry() || body.worldShape().isNull())
	{
		osg->removeBackendObjectVisual(body.id());
		osg->requestRedraw();
		return true;
	}
	QString sceneErr;
	if (!osg->loadBackendFromBackendData(body, &sceneErr, false, false, true))
	{
		if (outError)
			*outError = sceneErr.isEmpty() ? QStringLiteral("OSG refresh failed") : sceneErr;
		return false;
	}
	return true;
}

bool previewShapeStaging(OsgWidget* osg, const geoalgo::ShapeHandle& shape, const osg::Vec4& rgba, QString* errOut)
{
	std::string err;
	geoalgo::MeshDiscretizeParams meshParams;
	meshParams.quality = geoalgo::MeshQualityPreset::Medium;
	std::vector<float> soup;
	geoalgo::MeshDiscretizeReport report;
	if (!geoalgo::discretizeShapeHandleToMesh(shape, meshParams, soup, report, &err) || soup.size() < 9)
	{
		if (errOut)
			*errOut = QString::fromStdString(err.empty() ? "mesh discretize failed" : err);
		return false;
	}
	osg->setStagingMeshPreview(soup, rgba);
	return true;
}

std::shared_ptr<ParametricBrepBackendData> parametricBodyWithTip(cloudsim::host::DocumentHost* page,
																 const std::string& backendId,
																 QString* errOut = nullptr)
{
	if (!page)
	{
		if (errOut)
			*errOut = QStringLiteral("No active document");
		return nullptr;
	}
	if (backendId.empty())
	{
		if (errOut)
			*errOut = QStringLiteral("Empty targetParametricBackendId");
		return nullptr;
	}
	auto body = std::dynamic_pointer_cast<ParametricBrepBackendData>(page->backend().getData(backendId));
	if (!body)
	{
		if (errOut)
			*errOut = QStringLiteral("Parametric Body not found");
		return nullptr;
	}
	if (body->worldShape().isNull())
	{
		if (errOut)
			*errOut = QStringLiteral("Parametric Body has no solid tip");
		return nullptr;
	}
	return body;
}

/// 有 sourceFeatureId 时：seed=源特征贡献体(after CUT before)，fuseOnto=当前 tip
bool resolvePatternSeed(const ParametricBrepBackendData& body, const std::string& sourceFeatureId,
						geoalgo::ShapeHandle& seedOut, geoalgo::ShapeHandle& tipCopyOut,
						const geoalgo::ShapeHandle*& fuseOntoOut, QString* errOut)
{
	seedOut = body.worldShape();
	fuseOntoOut = nullptr;
	if (sourceFeatureId.empty())
		return true;
	const geoalgo::ShapeHandle after = body.tipAfterFeature(sourceFeatureId);
	if (after.isNull())
	{
		if (errOut)
			*errOut = QStringLiteral("Source feature tip unavailable (rebuild body first)");
		return false;
	}
	std::string seedErr;
	if (!geoalgo::featureContributionSeed(after, body.tipBeforeFeature(sourceFeatureId), seedOut, &seedErr) ||
		seedOut.isNull())
	{
		if (errOut)
			*errOut = QString::fromStdString(seedErr.empty() ? "contribution seed failed" : seedErr);
		return false;
	}
	tipCopyOut = body.worldShape();
	fuseOntoOut = &tipCopyOut;
	return true;
}

bool clearStagingAndWarn(PluginHostContext* host, IPluginDocument* doc, const QString& tag, const QString& msg,
						 QString* errOut)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (osg)
		osg->clearStagingGeometry();
	if (errOut)
		*errOut = msg;
	if (host && !msg.isEmpty())
		host->logWarn(QStringLiteral("[%1] %2").arg(tag, msg));
	return false;
}

void finishParametricBodyJob(PluginHostContext* host, cloudsim::host::DocumentHost* page,
							 const std::shared_ptr<ParametricBrepBackendData>& body, bool createNew,
							 PluginGeometryFinishedFn onFinished)
{
	if (createNew)
	{
		QString regErr;
		if (!cloudsim::host::registerAdoptedBrepAndLoadScene(*page, body, QStringLiteral("geomodel://parametric"),
															QLatin1String(backend_type::kCatalogParametricBrep), QString(),
															true, &regErr))
		{
			onFinished(false, regErr.isEmpty() ? QStringLiteral("register Parametric Body failed") : regErr, {});
			return;
		}
	}
	else
	{
		QString sceneErr;
		if (!refreshParametricBodyScene(page, *body, &sceneErr))
		{
			onFinished(false, sceneErr, {});
			return;
		}
	}
	PluginGeometryJobResult job;
	job.newBackendId = body->id();
	onFinished(true, QString(), job);
	if (host && page)
		host->invokeParametricBodyHistoryChanged(page->documentId(), QString::fromStdString(body->id()));
}
} // namespace

bool PluginGeometryHostImpl::queryFaceSketchPlane(IPluginDocument* doc, const PluginGeometryStepRef& faceRef,
												  PluginSketchPlane& outPlane, QString* outError)
{
	outPlane = {};
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page)
	{
		if (outError)
			*outError = QStringLiteral("No document");
		return false;
	}
	if (faceRef.faceIndex < 0)
	{
		if (outError)
			*outError = QStringLiteral("Invalid face index");
		return false;
	}

	auto tryBrep = [&](const std::shared_ptr<BrepBackendData>& brep) -> bool
	{
		if (!brep || brep->worldShape().isNull())
			return false;
		geoalgo::SketchPlaneMm plane;
		std::string err;
		if (!geoalgo::queryPlanarFaceSketchPlane(brep->worldShape(), faceRef.faceIndex, plane, &err))
		{
			if (outError)
				*outError = QString::fromStdString(err);
			return false;
		}
		outPlane.origin = {plane.ox, plane.oy, plane.oz};
		outPlane.axisX = {plane.xx, plane.xy, plane.xz};
		outPlane.axisY = {plane.yx, plane.yy, plane.yz};
		outPlane.normal = {plane.nx, plane.ny, plane.nz};
		outPlane.isPlanar = plane.planar;
		return true;
	};

	if (!faceRef.backendIdUtf8.empty())
	{
		auto brep = std::dynamic_pointer_cast<BrepBackendData>(page->backend().getData(faceRef.backendIdUtf8));
		if (tryBrep(brep))
			return true;
		if (outError && outError->isEmpty())
			*outError = QStringLiteral("B-rep face not found on backend");
		return false;
	}

	BackendDataManager& mgr = page->backend();
	for (const auto& id : mgr.topoOrder())
	{
		auto d = mgr.getData(id);
		if (!d)
			continue;
		auto brep = std::dynamic_pointer_cast<BrepBackendData>(d);
		if (!brep || brep->shapeRef().isNull())
			continue;
		const QString src = page->backendSourcePath().value(QString::fromStdString(id));
		if (!faceRef.stepPathUtf8.empty())
		{
			const QString want = QString::fromStdString(faceRef.stepPathUtf8);
			// STEP 文件：路径包含匹配；geomodel:// 等：精确匹配 source
			const bool pathOk = isStepPath(want) ? src.contains(want, Qt::CaseInsensitive)
												: (src.compare(want, Qt::CaseInsensitive) == 0);
			if (!pathOk)
				continue;
		}
		if (tryBrep(brep))
			return true;
		// 面索引对该实体无效则继续试下一个
		if (outError)
			outError->clear();
	}
	if (outError)
		*outError = QStringLiteral("B-rep face not found");
	return false;
}

void PluginGeometryHostImpl::setSketchOverlay(IPluginDocument* doc,
											  const std::vector<PluginSketchOverlaySegment>& segments)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return;
	std::vector<RobotOsgUi::RawTrajectoryOverlayVertex> pts;
	std::vector<std::size_t> ends;
	std::vector<osg::Vec4> colors;
	std::vector<float> widths;
	for (const auto& seg : segments)
	{
		for (std::size_t i = 0; i + 2 < seg.xyzMm.size(); i += 3)
		{
			RobotOsgUi::RawTrajectoryOverlayVertex v;
			v.positionMm.x = seg.xyzMm[i];
			v.positionMm.y = seg.xyzMm[i + 1];
			v.positionMm.z = seg.xyzMm[i + 2];
			pts.push_back(v);
		}
		ends.push_back(pts.size());
		colors.emplace_back(seg.rgba[0], seg.rgba[1], seg.rgba[2], seg.rgba[3]);
		widths.push_back(seg.lineWidthPx > 0.1f ? seg.lineWidthPx : (seg.construction ? 2.0f : 2.8f));
	}
	osg->setSketchLineOverlay(pts, ends, colors, widths);
}

void PluginGeometryHostImpl::clearSketchOverlay(IPluginDocument* doc)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (osg)
		osg->clearSketchLineOverlay();
}

bool PluginGeometryHostImpl::mapScreenToSketchPlane(IPluginDocument* doc, int screenX, int screenY,
												   const PluginSketchPlane& plane, PluginPoint3d& outWorldMm,
												   QString* outError)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
	{
		if (outError)
			*outError = QStringLiteral("3D viewport unavailable");
		return false;
	}
	if (!plane.isPlanar)
	{
		if (outError)
			*outError = QStringLiteral("Non-planar sketch plane");
		return false;
	}
	osg::Vec3d hit;
	QString err;
	if (!osg->intersectScreenWithPlaneMm(screenX, screenY, osg::Vec3d(plane.origin.x, plane.origin.y, plane.origin.z),
										 osg::Vec3d(plane.normal.x, plane.normal.y, plane.normal.z), hit, &err))
	{
		if (outError)
			*outError = err;
		return false;
	}
	outWorldMm = {hit.x(), hit.y(), hit.z()};
	return true;
}

bool PluginGeometryHostImpl::beginSketchInput(IPluginDocument* doc, const PluginSketchPlane& plane,
											  PluginSketchInputFn onInput, QString* outError)
{
	endSketchInput(doc);
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg || !onInput)
	{
		if (outError)
			*outError = QStringLiteral("Sketch input unavailable");
		return false;
	}
	if (!plane.isPlanar)
	{
		if (outError)
			*outError = QStringLiteral("Non-planar sketch plane");
		return false;
	}
	m_sketchInputDoc = doc;
	m_sketchInputPlane = plane;
	// 进入草图编辑：注视平面原点并正视（法向朝相机，axisY 为上）
	osg->orientViewToPlane(osg::Vec3d(plane.origin.x, plane.origin.y, plane.origin.z),
						   osg::Vec3d(plane.normal.x, plane.normal.y, plane.normal.z),
						   osg::Vec3d(plane.axisY.x, plane.axisY.y, plane.axisY.z));
	osg->setSketchPlaneInputHandler(
		[this, onInput](QObject*, QEvent* event) -> bool
		{
			PluginSketchInputEvent ev;
			const auto fillHit = [&](int sx, int sy)
			{
				ev.screenX = sx;
				ev.screenY = sy;
				PluginPoint3d hit{};
				QString err;
				ev.hasWorldHit = mapScreenToSketchPlane(m_sketchInputDoc, sx, sy, m_sketchInputPlane, hit, &err);
				if (ev.hasWorldHit)
					ev.worldMm = hit;
			};
			const auto type = event->type();
			if (type == QEvent::MouseMove)
			{
				const auto* me = static_cast<QMouseEvent*>(event);
				ev.kind = PluginSketchInputKind::MouseMove;
				ev.modifiers = static_cast<int>(me->modifiers());
				fillHit(me->x(), me->y());
				return onInput(ev);
			}
			if (type == QEvent::MouseButtonPress || type == QEvent::MouseButtonRelease)
			{
				const auto* me = static_cast<QMouseEvent*>(event);
				ev.kind = (type == QEvent::MouseButtonPress) ? PluginSketchInputKind::MousePress
															 : PluginSketchInputKind::MouseRelease;
				ev.buttonOrKey = static_cast<int>(me->button());
				ev.modifiers = static_cast<int>(me->modifiers());
				fillHit(me->x(), me->y());
				// 左键/右键点击一律消费，避免转轨道
				const bool consumeBtn =
					me->button() == Qt::LeftButton || me->button() == Qt::RightButton || me->button() == Qt::MiddleButton;
				const bool handled = onInput(ev);
				return handled || (type == QEvent::MouseButtonPress && consumeBtn);
			}
			if (type == QEvent::KeyPress)
			{
				const auto* ke = static_cast<QKeyEvent*>(event);
				ev.kind = PluginSketchInputKind::KeyPress;
				ev.buttonOrKey = static_cast<int>(ke->key());
				ev.modifiers = static_cast<int>(ke->modifiers());
				return onInput(ev);
			}
			return false;
		});
	return true;
}

void PluginGeometryHostImpl::endSketchInput(IPluginDocument* doc)
{
	(void)doc;
	cloudsim::host::DocumentHost* page = m_sketchInputDoc ? pageFromDoc(m_sketchInputDoc) : pageFromDoc(doc);
	if (!page && m_host)
	{
		// 兜底：清掉任意活动视口 handler
	}
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (osg)
		osg->clearSketchPlaneInputHandler();
	m_sketchInputDoc = nullptr;
	m_sketchInputPlane = {};
}

namespace
{
PluginSketchPlane makeOriginPluginPlane(int index)
{
	PluginSketchPlane p;
	p.isPlanar = true;
	p.origin = {0, 0, 0};
	switch (index)
	{
	case 1: // XZ
		p.axisX = {1, 0, 0};
		p.axisY = {0, 0, 1};
		p.normal = {0, 1, 0};
		break;
	case 2: // YZ
		p.axisX = {0, 1, 0};
		p.axisY = {0, 0, 1};
		p.normal = {1, 0, 0};
		break;
	default: // XY
		p.axisX = {1, 0, 0};
		p.axisY = {0, 1, 0};
		p.normal = {0, 0, 1};
		break;
	}
	return p;
}
} // namespace

void PluginGeometryHostImpl::clearSketchSupportPlanePick()
{
	if (m_supportPlaneFaceConn)
	{
		QObject::disconnect(m_supportPlaneFaceConn);
		m_supportPlaneFaceConn = {};
	}
	m_supportPlanePickDone.reset();
}

void PluginGeometryHostImpl::pickOriginSketchPlane(IPluginDocument* doc, PluginOriginPlanePickedFn onFinished)
{
	if (!onFinished)
		return;
	pickSketchSupportPlane(doc, {},
						   [onFinished](bool ok, const QString& err, PluginOriginPlaneKind kind,
										const PluginSketchPlane& plane, const QString& /*tag*/)
						   { onFinished(ok, err, kind, plane); });
}

void PluginGeometryHostImpl::pickSketchSupportPlane(IPluginDocument* doc,
													const std::vector<PluginSupportPlaneCandidate>& extras,
													PluginSupportPlanePickedFn onFinished)
{
	if (!onFinished)
		return;
	endSketchInput(doc);
	clearSketchSupportPlanePick();
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
	{
		onFinished(false, QStringLiteral("3D viewport unavailable"), PluginOriginPlaneKind::XY, {}, QString());
		return;
	}

	const auto extrasPtr = std::make_shared<std::vector<PluginSupportPlaneCandidate>>(extras);
	{
		std::vector<OsgWidget::SketchSupportExtraPlane> osgExtras;
		osgExtras.reserve(extrasPtr->size());
		for (const PluginSupportPlaneCandidate& c : *extrasPtr)
		{
			OsgWidget::SketchSupportExtraPlane e;
			e.origin = osg::Vec3d(c.plane.origin.x, c.plane.origin.y, c.plane.origin.z);
			e.axisX = osg::Vec3d(c.plane.axisX.x, c.plane.axisX.y, c.plane.axisX.z);
			e.axisY = osg::Vec3d(c.plane.axisY.x, c.plane.axisY.y, c.plane.axisY.z);
			e.normal = osg::Vec3d(c.plane.normal.x, c.plane.normal.y, c.plane.normal.z);
			e.halfMm = c.halfExtentMm > 1.f ? c.halfExtentMm : 40.f;
			osgExtras.push_back(e);
		}
		osg->setSketchSupportExtraPlanes(std::move(osgExtras));
	}

	const auto sessionDone = std::make_shared<bool>(false);
	m_supportPlanePickDone = sessionDone;

	const auto finishPlane = [sessionDone, this, osg, onFinished, extrasPtr](bool ok, int planeIndex)
	{
		if (*sessionDone)
			return;
		*sessionDone = true;
		if (m_supportPlaneFaceConn)
		{
			QObject::disconnect(m_supportPlaneFaceConn);
			m_supportPlaneFaceConn = {};
		}
		if (osg)
		{
			osg->setMeshLinePickMode(false);
			osg->setMeshFacePickMode(false);
			osg->clearSketchSupportExtraPlanes();
		}
		m_supportPlanePickDone.reset();
		if (!ok || planeIndex < 0)
		{
			onFinished(false, QStringLiteral("已取消草图平面选择"), PluginOriginPlaneKind::XY, {}, QString());
			return;
		}
		if (planeIndex >= 100)
		{
			const int ei = planeIndex - 100;
			if (ei < 0 || ei >= static_cast<int>(extrasPtr->size()))
			{
				onFinished(false, QStringLiteral("已取消草图平面选择"), PluginOriginPlaneKind::XY, {}, QString());
				return;
			}
			const PluginSupportPlaneCandidate& c = (*extrasPtr)[static_cast<std::size_t>(ei)];
			onFinished(true, QString(), PluginOriginPlaneKind::XY, c.plane, QString::fromStdString(c.tagUtf8));
			return;
		}
		if (planeIndex > 2)
		{
			onFinished(false, QStringLiteral("已取消草图平面选择"), PluginOriginPlaneKind::XY, {}, QString());
			return;
		}
		const auto kind = static_cast<PluginOriginPlaneKind>(planeIndex);
		onFinished(true, QString(), kind, makeOriginPluginPlane(planeIndex),
				   QStringLiteral("origin:%1").arg(planeIndex));
	};

	osg->setSelectionActive(true);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(true);
	m_supportPlaneFaceConn = QObject::connect(
		osg, &OsgWidget::meshPickCommitted, m_host,
		[sessionDone, this, osg, page, doc, onFinished, extrasPtr](PickResult pick, int pickKindInt)
		{
			if (*sessionDone || !pick.hit)
				return;
			if (static_cast<PickKind>(pickKindInt) != PickKind::MeshFace)
				return;

			const QPoint mp = osg->lastMousePos();
			const int resolved = osg->resolveSketchSupportOriginIndex(mp.x(), mp.y());
			if (resolved >= 100)
			{
				const int extraIdx = resolved - 100;
				if (extraIdx < 0 || extraIdx >= static_cast<int>(extrasPtr->size()))
					return;
				*sessionDone = true;
				if (m_supportPlaneFaceConn)
				{
					QObject::disconnect(m_supportPlaneFaceConn);
					m_supportPlaneFaceConn = {};
				}
				m_supportPlanePickDone.reset();
				osg->setMeshLinePickMode(false);
				osg->setMeshFacePickMode(false);
				osg->cancelOriginPlaneSelection();
				const PluginSupportPlaneCandidate& c = (*extrasPtr)[static_cast<std::size_t>(extraIdx)];
				osg->setCameraViewDirection(osg::Vec3d(c.plane.normal.x, c.plane.normal.y, c.plane.normal.z),
											osg::Vec3d(c.plane.axisY.x, c.plane.axisY.y, c.plane.axisY.z));
				onFinished(true, QString(), PluginOriginPlaneKind::XY, c.plane, QString::fromStdString(c.tagUtf8));
				return;
			}

			const std::string backendId = pick.backendId;
			if (backendId.empty())
				return;

			QString stepPath = stepPathForBackend(page, backendId);
			PluginGeometryStepRef outRef;
			outRef.backendIdUtf8 = backendId;
			outRef.stepPathUtf8 = stepPath.toStdString();

			if (pick.brepNativePick && pick.brepFaceIndex >= 0)
			{
				outRef.faceIndex = pick.brepFaceIndex;
			}
			else
			{
				geoalgo::Point3d modelA{};
				std::string err;
				if (!worldPointToStepModelMm(osg, backendId, pick.worldPoint, modelA, &err))
					return;
				const bool stepFile = isStepPath(stepPath);
				if (!stepFile)
				{
					auto inMemoryBrep = std::dynamic_pointer_cast<BrepBackendData>(page->backend().getData(backendId));
					if (!inMemoryBrep || !inMemoryBrep->hasGeometry())
						return;
				}
				geometry_backend_ops::GeometryRef ref;
				ref.backendIdUtf8 = backendId;
				ref.stepPathUtf8 = stepPath.toStdString();
				geoalgo::WorkpieceRef wp;
				if (!geometry_backend_ops::resolveGeometryRef(ref, wp, &err))
					return;
				geoalgo::ShapeHandle shape;
				geoalgo::WorkpieceRef shapeRef;
				if (geometry_backend_ops::resolveWorkpieceShape(backendId, page->backend(), stepPath.toStdString(), shape,
																shapeRef, &err) ==
					geometry_backend_ops::WorkpieceShapeSource::Unavailable)
					return;
				geoalgo::FeatureEntry entry;
				const int knownFaceIndex = pick.brepNativePick ? pick.brepFaceIndex : -1;
				if (!geometry_backend_ops::buildFeatureEntryFromModelPick(wp, shape, "FaceBoundary", true, modelA, modelA,
																		  entry, &err, knownFaceIndex, -1))
					return;
				if (entry.geometry.faceIndices.empty())
					return;
				outRef.faceIndex = entry.geometry.faceIndices.front();
			}

			PluginSketchPlane plane;
			QString planeErr;
			if (!queryFaceSketchPlane(doc, outRef, plane, &planeErr) || !plane.isPlanar)
			{
				if (planeErr.isEmpty())
					planeErr = QStringLiteral("所选面不是平面，请选平面面或基准面");
				return;
			}

			*sessionDone = true;
			if (m_supportPlaneFaceConn)
			{
				QObject::disconnect(m_supportPlaneFaceConn);
				m_supportPlaneFaceConn = {};
			}
			m_supportPlanePickDone.reset();
			if (osg)
			{
				osg->setMeshLinePickMode(false);
				osg->setMeshFacePickMode(false);
				osg->cancelOriginPlaneSelection();
				osg->setCameraViewDirection(osg::Vec3d(plane.normal.x, plane.normal.y, plane.normal.z),
											osg::Vec3d(plane.axisY.x, plane.axisY.y, plane.axisY.z));
			}
			// tag 编码面源，供等距基准面写关联
			const QString faceTag = QStringLiteral("face:%1:%2")
										.arg(QString::fromStdString(backendId))
										.arg(outRef.faceIndex);
			onFinished(true, QString(), PluginOriginPlaneKind::XY, plane, faceTag);
		});

	osg->beginOriginPlaneSelection(finishPlane);
}

void PluginGeometryHostImpl::cancelOriginSketchPlanePick(IPluginDocument* doc)
{
	if (m_supportPlanePickDone && !*m_supportPlanePickDone)
		*m_supportPlanePickDone = true;
	clearSketchSupportPlanePick();
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (osg)
	{
		osg->setMeshLinePickMode(false);
		osg->setMeshFacePickMode(false);
		osg->cancelOriginPlaneSelection();
	}
}

void PluginGeometryHostImpl::previewSketchExtrude(IPluginDocument* doc, const std::vector<float>& closedPolylineXyzMm,
												  const PluginSketchPlane& plane,
												  const PluginSketchExtrudeParams& params)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg || closedPolylineXyzMm.size() < 12)
	{
		if (osg)
			osg->clearStagingGeometry();
		return;
	}

	geoalgo::SketchExtrudeParams ep;
	ep.mode = (params.mode == PluginSketchExtrudeMode::Pocket) ? geoalgo::SketchExtrudeMode::Pocket
															  : geoalgo::SketchExtrudeMode::Pad;
	ep.lengthMm = params.lengthMm;
	ep.length2Mm = params.length2Mm;
	ep.startOffsetMm = params.startOffsetMm;
	ep.reversed = params.reversed;
	ep.draftAngleDeg = params.draftAngleDeg;
	if (params.endCondition == PluginSketchExtrudeEnd::UpToFace)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::UpToFace;
	else if (params.endCondition == PluginSketchExtrudeEnd::MidPlane)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::MidPlane;
	else if (params.endCondition == PluginSketchExtrudeEnd::ThroughAll)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::ThroughAll;
	else if (params.endCondition == PluginSketchExtrudeEnd::UpToVertex)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::UpToVertex;
	else if (params.endCondition == PluginSketchExtrudeEnd::OffsetFromFace)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::OffsetFromFace;
	else if (params.endCondition == PluginSketchExtrudeEnd::TwoDirections)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::TwoDirections;
	else
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::Blind;
	ep.hasUpToFace = params.hasUpToFacePlane;
	ep.upOriginX = params.upToFacePlane.origin.x;
	ep.upOriginY = params.upToFacePlane.origin.y;
	ep.upOriginZ = params.upToFacePlane.origin.z;
	ep.upNormalX = params.upToFacePlane.normal.x;
	ep.upNormalY = params.upToFacePlane.normal.y;
	ep.upNormalZ = params.upToFacePlane.normal.z;
	ep.hasUpToVertex = params.hasUpToVertex;
	ep.upToVertexX = params.upToVertex.x;
	ep.upToVertexY = params.upToVertex.y;
	ep.upToVertexZ = params.upToVertex.z;
	ep.offsetFromFaceMm = params.offsetFromFaceMm;
	ep.originX = plane.origin.x;
	ep.originY = plane.origin.y;
	ep.originZ = plane.origin.z;
	ep.normalX = plane.normal.x;
	ep.normalY = plane.normal.y;
	ep.normalZ = plane.normal.z;
	ep.holePolylinesXyzMm = params.holePolylinesXyzMm;
	appendProfileSegments(ep, params.profileSegments);

	const geoalgo::ShapeHandle* basePtr = nullptr;
	geoalgo::ShapeHandle baseOwned;
	const bool needBase = (params.mode == PluginSketchExtrudeMode::Pocket)
						  || (params.endCondition == PluginSketchExtrudeEnd::ThroughAll)
						  || (params.endCondition == PluginSketchExtrudeEnd::OffsetFromFace
							  && params.hasUpToFacePlane);
	if (needBase && !params.targetParametricBackendIdUtf8.empty())
	{
		auto body =
			std::dynamic_pointer_cast<ParametricBrepBackendData>(page->backend().getData(params.targetParametricBackendIdUtf8));
		if (body && !body->worldShape().isNull())
		{
			baseOwned = body->worldShape();
			basePtr = &baseOwned;
		}
	}

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::sketchExtrudePolylineToHandle(closedPolylineXyzMm, ep, basePtr, result, &err) || result.isNull())
	{
		osg->clearStagingGeometry();
		return;
	}

	geoalgo::MeshDiscretizeParams meshParams;
	meshParams.quality = geoalgo::MeshQualityPreset::Medium;
	std::vector<float> soup;
	geoalgo::MeshDiscretizeReport report;
	if (!geoalgo::discretizeShapeHandleToMesh(result, meshParams, soup, report, &err) || soup.size() < 9)
	{
		osg->clearStagingGeometry();
		return;
	}

	const osg::Vec4 rgba = (params.mode == PluginSketchExtrudeMode::Pocket)
							   ? osg::Vec4(0.95f, 0.45f, 0.25f, 0.35f)
							   : osg::Vec4(0.20f, 0.75f, 0.85f, 0.35f);
	osg->setStagingMeshPreview(soup, rgba);
}

void PluginGeometryHostImpl::clearSketchExtrudePreview(IPluginDocument* doc)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (osg)
		osg->clearStagingGeometry();
}

void PluginGeometryHostImpl::extrudeSketchProfileToBrep(IPluginDocument* doc,
													   const std::vector<float>& closedPolylineXyzMm,
													   const PluginSketchPlane& plane,
													   const PluginSketchExtrudeParams& params,
													   PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (closedPolylineXyzMm.size() < 12)
	{
		onFinished(false, QStringLiteral("Profile too short"), {});
		return;
	}

	const bool pocket = (params.mode == PluginSketchExtrudeMode::Pocket);
	const ParametricSketchPlane pPlane = toParametricPlane(plane);
	const QString bodyName = params.resultNameUtf8.empty() ? QStringLiteral("ParametricBody")
														   : QString::fromStdString(params.resultNameUtf8);

	std::shared_ptr<ParametricBrepBackendData> body;
	const bool createNew = params.targetParametricBackendIdUtf8.empty();
	if (createNew)
	{
		if (pocket)
		{
			onFinished(false, QStringLiteral("Pocket requires existing Parametric Body"), {});
			return;
		}
		body = std::make_shared<ParametricBrepBackendData>();
		body->setName(bodyName.toStdString());
	}
	else
	{
		auto obj = page->backend().getData(params.targetParametricBackendIdUtf8);
		body = std::dynamic_pointer_cast<ParametricBrepBackendData>(obj);
		if (!body)
		{
			onFinished(false, QStringLiteral("Parametric Body not found"), {});
			return;
		}
	}

	const std::string sketchId = body->addSketch(pPlane);
	if (!body->setProfile(sketchId, closedPolylineXyzMm))
	{
		onFinished(false, QStringLiteral("Failed to set sketch profile"), {});
		return;
	}
	if (ParametricFeature* sk = body->findFeature(sketchId))
	{
		if (!params.holePolylinesXyzMm.empty())
			sk->profileHolesXyzMm = params.holePolylinesXyzMm;
		if (!params.sketchDocumentJsonUtf8.empty())
			sk->sketchDocumentJson = params.sketchDocumentJsonUtf8;
	}
	if (pocket)
		body->addPocket(sketchId, params.lengthMm, params.reversed);
	else
		body->addPad(sketchId, params.lengthMm, params.reversed);
	if (ParametricFeature* extrudeFeat = body->findFeature(body->features().back().id))
	{
		if (!params.holePolylinesXyzMm.empty())
			extrudeFeat->profileHolesXyzMm = params.holePolylinesXyzMm;
		if (params.endCondition == PluginSketchExtrudeEnd::UpToFace)
			extrudeFeat->endCondition = ParametricExtrudeEnd::UpToFace;
		else if (params.endCondition == PluginSketchExtrudeEnd::MidPlane)
			extrudeFeat->endCondition = ParametricExtrudeEnd::MidPlane;
		else if (params.endCondition == PluginSketchExtrudeEnd::ThroughAll)
			extrudeFeat->endCondition = ParametricExtrudeEnd::ThroughAll;
		else if (params.endCondition == PluginSketchExtrudeEnd::UpToVertex)
			extrudeFeat->endCondition = ParametricExtrudeEnd::UpToVertex;
		else if (params.endCondition == PluginSketchExtrudeEnd::OffsetFromFace)
			extrudeFeat->endCondition = ParametricExtrudeEnd::OffsetFromFace;
		else if (params.endCondition == PluginSketchExtrudeEnd::TwoDirections)
			extrudeFeat->endCondition = ParametricExtrudeEnd::TwoDirections;
		else
			extrudeFeat->endCondition = ParametricExtrudeEnd::Blind;
		if (params.hasUpToFacePlane)
		{
			extrudeFeat->hasUpToFacePlane = true;
			extrudeFeat->upToFacePlane = toParametricPlane(params.upToFacePlane);
		}
		extrudeFeat->upToFaceBackendId = params.upToFaceBackendIdUtf8;
		extrudeFeat->upToFaceIndex = params.upToFaceIndex;
		extrudeFeat->hasUpToVertex = params.hasUpToVertex;
		extrudeFeat->upToVertexX = params.upToVertex.x;
		extrudeFeat->upToVertexY = params.upToVertex.y;
		extrudeFeat->upToVertexZ = params.upToVertex.z;
		extrudeFeat->upToVertexIndex = params.upToVertexIndex;
		extrudeFeat->offsetFromFaceMm = params.offsetFromFaceMm;
		extrudeFeat->draftAngleDeg = params.draftAngleDeg;
		extrudeFeat->startOffsetMm = params.startOffsetMm;
		extrudeFeat->length2Mm = params.length2Mm;
		storeProfileSegments(*extrudeFeat, params.profileSegments);
	}

	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}

	if (createNew)
	{
		QString regErr;
		if (!cloudsim::host::registerAdoptedBrepAndLoadScene(*page, body, QStringLiteral("geomodel://parametric"),
															QLatin1String(backend_type::kCatalogParametricBrep), QString(), true,
															&regErr))
		{
			onFinished(false, regErr.isEmpty() ? QStringLiteral("register Parametric Body failed") : regErr, {});
			return;
		}
	}
	else
	{
		QString sceneErr;
		if (!refreshParametricBodyScene(page, *body, &sceneErr))
		{
			onFinished(false, sceneErr, {});
			return;
		}
	}

	PluginGeometryJobResult job;
	job.newBackendId = body->id();
	onFinished(true, QString(), job);
	if (m_host)
		m_host->invokeParametricBodyHistoryChanged(page->documentId(), QString::fromStdString(body->id()));
}

bool PluginGeometryHostImpl::queryParametricBodyHistoryJson(IPluginDocument* doc, const std::string& backendIdUtf8,
															QByteArray& outJsonUtf8, QString* outError)
{
	outJsonUtf8.clear();
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page)
	{
		if (outError)
			*outError = QStringLiteral("No document");
		return false;
	}
	auto body = std::dynamic_pointer_cast<ParametricBrepBackendData>(page->backend().getData(backendIdUtf8));
	if (!body)
	{
		if (outError)
			*outError = QStringLiteral("Parametric Body not found");
		return false;
	}
	const std::string dumped = body->historyToJson().dump();
	outJsonUtf8 = QByteArray(dumped.data(), static_cast<int>(dumped.size()));
	return true;
}

void PluginGeometryHostImpl::setParametricBodyHistoryJson(IPluginDocument* doc, const std::string& backendIdUtf8,
														  const QByteArray& historyJsonUtf8,
														  PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page)
	{
		onFinished(false, QStringLiteral("No document"), {});
		return;
	}
	auto body = std::dynamic_pointer_cast<ParametricBrepBackendData>(page->backend().getData(backendIdUtf8));
	if (!body)
	{
		onFinished(false, QStringLiteral("Parametric Body not found"), {});
		return;
	}
	nlohmann::json root;
	try
	{
		root = nlohmann::json::parse(historyJsonUtf8.constData(),
									 historyJsonUtf8.constData() + historyJsonUtf8.size());
	}
	catch (const std::exception& ex)
	{
		onFinished(false, QStringLiteral("Invalid history JSON: %1").arg(QString::fromUtf8(ex.what())), {});
		return;
	}
	std::string err;
	if (!body->historyFromJson(root, &err))
	{
		onFinished(false, QString::fromStdString(err.empty() ? "historyFromJson failed" : err), {});
		return;
	}
	if (!body->rebuild(&err))
	{
		onFinished(false, QString::fromStdString(err.empty() ? "rebuild failed" : err), {});
		return;
	}
	QString sceneErr;
	if (!refreshParametricBodyScene(page, *body, &sceneErr))
	{
		onFinished(false, sceneErr, {});
		return;
	}
	PluginGeometryJobResult job;
	job.newBackendId = body->id();
	onFinished(true, QString(), job);
}

bool PluginGeometryHostImpl::listParametricBodyIds(IPluginDocument* doc, std::vector<std::string>& outIds,
												   QString* outError)
{
	outIds.clear();
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page)
	{
		if (outError)
			*outError = QStringLiteral("No document");
		return false;
	}
	for (const auto& data : page->backend().listData())
	{
		if (!data)
			continue;
		if (data->className() != backend_type::kClassParametricBrep)
			continue;
		outIds.push_back(data->id());
	}
	return true;
}

void PluginGeometryHostImpl::pickParametricFeatureForEdit(IPluginDocument* doc,
														  PluginParametricFeaturePickedFn onFinished)
{
	if (!onFinished)
		return;
	PluginGeometryElementPickRequest req;
	req.kind = PluginGeometryElementKind::Face;
	pickStepElementFromViewport(
		doc, req,
		[doc, onFinished](bool ok, const QString& err, const PluginGeometryStepRef& ref)
		{
			if (!ok)
			{
				onFinished(false, err, QString(), QString());
				return;
			}
			const QString backendId = QString::fromStdString(ref.backendIdUtf8);
			if (backendId.isEmpty())
			{
				onFinished(false, QStringLiteral("Not a parametric body face"), QString(), QString());
				return;
			}
			cloudsim::host::DocumentHost* page = pageFromDoc(doc);
			auto body = page ? std::dynamic_pointer_cast<ParametricBrepBackendData>(
								   page->backend().getData(ref.backendIdUtf8))
							 : nullptr;
			if (!body)
			{
				onFinished(false, QStringLiteral("Parametric Body not found"), backendId, QString());
				return;
			}
			// 优先面归属表；未命中则空建议，由插件弹特征菜单
			QString suggested = QString::fromStdString(body->featureIdForFace(ref.faceIndex));
			onFinished(true, QString(), backendId, suggested);
		});
}

bool PluginGeometryHostImpl::previewSketchSweep(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
												const std::vector<float>& pathPolylineXyzMm,
												const PluginSketchSweepParams& params, QString* errOut)
{
	auto fail = [&](const QString& msg) -> bool
	{
		cloudsim::host::DocumentHost* page = pageFromDoc(doc);
		OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
		if (osg)
			osg->clearStagingGeometry();
		if (errOut)
			*errOut = msg;
		if (m_host && !msg.isEmpty())
			m_host->logWarn(QStringLiteral("[Sweep preview] %1").arg(msg));
		return false;
	};

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return fail(QStringLiteral("No viewport"));
	if (profilePolylineXyzMm.size() < 12)
		return fail(QStringLiteral("Sweep profile too short"));
	if (params.pathSegments.empty() && pathPolylineXyzMm.size() < 6)
		return fail(QStringLiteral("Sweep path too short"));

	geoalgo::SketchSweepParams sp;
	sp.mode = (params.mode == PluginSketchSweepMode::Cut) ? geoalgo::SketchSweepMode::Cut : geoalgo::SketchSweepMode::Boss;
	sp.twistDeg = params.twistDeg;
	appendProfileSegments(sp, params.profileSegments);

	const geoalgo::ShapeHandle* basePtr = nullptr;
	geoalgo::ShapeHandle baseOwned;
	if (!params.targetParametricBackendIdUtf8.empty())
	{
		auto body = std::dynamic_pointer_cast<ParametricBrepBackendData>(
			page->backend().getData(params.targetParametricBackendIdUtf8));
		if (body && !body->worldShape().isNull())
		{
			baseOwned = body->worldShape();
			basePtr = &baseOwned;
		}
	}
	if (params.mode == PluginSketchSweepMode::Cut && !basePtr)
		return fail(QStringLiteral("SweepCut requires existing solid"));

	geoalgo::ShapeHandle result;
	std::string err;
	bool ok = false;
	if (!params.pathSegments.empty())
	{
		std::vector<geoalgo::SketchSweepPathSegment> segs;
		segs.reserve(params.pathSegments.size());
		for (const auto& p : params.pathSegments)
		{
			geoalgo::SketchSweepPathSegment g;
			g.kind = (p.kind == PluginSketchSweepPathSegKind::Arc)	 ? geoalgo::SketchSweepPathSegKind::Arc
					 : (p.kind == PluginSketchSweepPathSegKind::SplineThrough)
						 ? geoalgo::SketchSweepPathSegKind::SplineThrough
						 : geoalgo::SketchSweepPathSegKind::Line;
			g.ax = p.ax;
			g.ay = p.ay;
			g.az = p.az;
			g.bx = p.bx;
			g.by = p.by;
			g.bz = p.bz;
			g.mx = p.mx;
			g.my = p.my;
			g.mz = p.mz;
			segs.push_back(g);
		}
		ok = geoalgo::sketchSweepSegmentsToHandle(profilePolylineXyzMm, segs, sp, basePtr, result, &err);
	}
	else
	{
		ok = geoalgo::sketchSweepPolylineToHandle(profilePolylineXyzMm, pathPolylineXyzMm, sp, basePtr, result, &err);
	}
	if (!ok || result.isNull())
		return fail(QString::fromStdString(err.empty() ? "MakePipe failed" : err));

	geoalgo::MeshDiscretizeParams meshParams;
	meshParams.quality = geoalgo::MeshQualityPreset::Medium;
	std::vector<float> soup;
	geoalgo::MeshDiscretizeReport report;
	if (!geoalgo::discretizeShapeHandleToMesh(result, meshParams, soup, report, &err) || soup.size() < 9)
		return fail(QString::fromStdString(err.empty() ? "mesh discretize failed" : err));

	const osg::Vec4 rgba = (params.mode == PluginSketchSweepMode::Cut)
							   ? osg::Vec4(0.95f, 0.45f, 0.25f, 0.35f)
							   : osg::Vec4(0.35f, 0.65f, 0.95f, 0.35f);
	osg->setStagingMeshPreview(soup, rgba);
	return true;
}

void PluginGeometryHostImpl::sweepSketchProfileToBrep(IPluginDocument* doc,
													  const std::vector<float>& profilePolylineXyzMm,
													  const std::vector<float>& pathPolylineXyzMm,
													  const PluginSketchSweepParams& params,
													  PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (profilePolylineXyzMm.size() < 12 || (params.pathSegments.empty() && pathPolylineXyzMm.size() < 6))
	{
		onFinished(false, QStringLiteral("Sweep profile/path too short"), {});
		return;
	}

	const bool cut = (params.mode == PluginSketchSweepMode::Cut);
	const QString bodyName = params.resultNameUtf8.empty() ? QStringLiteral("ParametricBody")
														   : QString::fromStdString(params.resultNameUtf8);

	std::shared_ptr<ParametricBrepBackendData> body;
	const bool createNew = params.targetParametricBackendIdUtf8.empty();
	if (createNew)
	{
		if (cut)
		{
			onFinished(false, QStringLiteral("SweepCut requires existing Parametric Body"), {});
			return;
		}
		body = std::make_shared<ParametricBrepBackendData>();
		body->setName(bodyName.toStdString());
	}
	else
	{
		body = std::dynamic_pointer_cast<ParametricBrepBackendData>(
			page->backend().getData(params.targetParametricBackendIdUtf8));
		if (!body)
		{
			onFinished(false, QStringLiteral("Parametric Body not found"), {});
			return;
		}
	}

	std::string profileSketchId = params.profileSketchIdUtf8;
	std::string pathSketchId = params.pathSketchIdUtf8;
	if (profileSketchId.empty() || !body->findFeature(profileSketchId))
	{
		profileSketchId = body->addSketch(toParametricPlane(params.profilePlane), "SweepProfile");
	}
	if (pathSketchId.empty() || !body->findFeature(pathSketchId))
	{
		pathSketchId = body->addSketch(toParametricPlane(params.pathPlane), "SweepPath");
	}
	body->setProfile(profileSketchId, profilePolylineXyzMm);
	body->setProfile(pathSketchId, pathPolylineXyzMm);
	if (ParametricFeature* sk = body->findFeature(profileSketchId))
	{
		sk->plane = toParametricPlane(params.profilePlane);
		if (!params.profileSketchDocumentJsonUtf8.empty())
			sk->sketchDocumentJson = params.profileSketchDocumentJsonUtf8;
	}
	if (ParametricFeature* sk = body->findFeature(pathSketchId))
	{
		sk->plane = toParametricPlane(params.pathPlane);
		if (!params.pathSketchDocumentJsonUtf8.empty())
			sk->sketchDocumentJson = params.pathSketchDocumentJsonUtf8;
	}

	const std::string sweepId = body->addSweep(profileSketchId, pathSketchId, cut);
	if (ParametricFeature* sw = body->findFeature(sweepId))
	{
		sw->profileXyzMm = profilePolylineXyzMm;
		sw->pathXyzMm = pathPolylineXyzMm;
		sw->pathSegments.clear();
		for (const auto& p : params.pathSegments)
		{
			ParametricFeature::PathSegment s;
			s.kind = (p.kind == PluginSketchSweepPathSegKind::Arc)			? 1
					 : (p.kind == PluginSketchSweepPathSegKind::SplineThrough) ? 2
																			  : 0;
			s.ax = p.ax;
			s.ay = p.ay;
			s.az = p.az;
			s.bx = p.bx;
			s.by = p.by;
			s.bz = p.bz;
			s.mx = p.mx;
			s.my = p.my;
			s.mz = p.mz;
			sw->pathSegments.push_back(s);
		}
		storeProfileSegments(*sw, params.profileSegments);
		sw->twistDeg = params.twistDeg;
	}

	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}

	if (createNew)
	{
		QString regErr;
		if (!cloudsim::host::registerAdoptedBrepAndLoadScene(*page, body, QStringLiteral("geomodel://parametric"),
															QLatin1String(backend_type::kCatalogParametricBrep), QString(),
															true, &regErr))
		{
			onFinished(false, regErr.isEmpty() ? QStringLiteral("register Parametric Body failed") : regErr, {});
			return;
		}
	}
	else
	{
		QString sceneErr;
		if (!refreshParametricBodyScene(page, *body, &sceneErr))
		{
			onFinished(false, sceneErr, {});
			return;
		}
	}

	PluginGeometryJobResult job;
	job.newBackendId = body->id();
	onFinished(true, QString(), job);
}

bool PluginGeometryHostImpl::previewFilletEdges(IPluginDocument* doc, const PluginSketchFilletParams& params,
												QString* errOut)
{
	auto fail = [&](const QString& msg) -> bool
	{
		cloudsim::host::DocumentHost* page = pageFromDoc(doc);
		OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
		if (osg)
			osg->clearStagingGeometry();
		if (errOut)
			*errOut = msg;
		if (m_host && !msg.isEmpty())
			m_host->logWarn(QStringLiteral("[Fillet preview] %1").arg(msg));
		return false;
	};

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return fail(QStringLiteral("No viewport"));
	if (params.edgeIndices.empty())
		return fail(QStringLiteral("No edges selected"));

	QString bodyErr;
	const auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8, &bodyErr);
	if (!body)
		return fail(bodyErr);

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::filletEdgesToHandle(body->worldShape(), params.edgeIndices, params.radiusMm, result, &err)
		|| result.isNull())
		return fail(QString::fromStdString(err.empty() ? "Fillet failed" : err));

	const osg::Vec4 rgba(0.45f, 0.85f, 0.55f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut) ? true : fail(errOut ? *errOut : QStringLiteral("preview failed"));
}

void PluginGeometryHostImpl::filletEdgesToBrep(IPluginDocument* doc, const PluginSketchFilletParams& params,
											   PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (params.edgeIndices.empty() && !params.allEdges && params.edgeSelectUtf8.empty())
	{
		onFinished(false, QStringLiteral("No edges selected"), {});
		return;
	}

	auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8);
	if (!body)
	{
		onFinished(false, QStringLiteral("Parametric Body not found or has no solid tip"), {});
		return;
	}

	std::vector<int> edges = params.edgeIndices;
	if (!params.edgeSelectUtf8.empty() && !params.allEdges)
	{
		edges.clear();
		std::string selErr;
		const bool okSel =
			(params.edgeSelectUtf8 == "top_boundary")
				? geoalgo::selectTopBoundaryEdgeIndices(body->worldShape(), edges, &selErr)
				: geoalgo::selectLongestEdgeIndices(body->worldShape(),
													params.edgeSelectCount > 0 ? params.edgeSelectCount : 4, edges,
													&selErr);
		if (!okSel || edges.empty())
		{
			onFinished(false, QString::fromStdString(selErr.empty() ? "edge select failed" : selErr), {});
			return;
		}
	}
	else if (params.allEdges)
	{
		edges.clear();
		const int n = geoalgo::shapeHandleEdgeCount(body->worldShape());
		edges.reserve(n > 0 ? n : 0);
		for (int i = 0; i < n; ++i)
			edges.push_back(i);
		if (edges.empty())
		{
			onFinished(false, QStringLiteral("Tip has no edges"), {});
			return;
		}
	}

	body->addFillet(edges, params.radiusMm);
	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, false, std::move(onFinished));
}

bool PluginGeometryHostImpl::previewChamferEdges(IPluginDocument* doc, const PluginSketchChamferParams& params,
												 QString* errOut)
{
	auto fail = [&](const QString& msg) -> bool
	{
		cloudsim::host::DocumentHost* page = pageFromDoc(doc);
		OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
		if (osg)
			osg->clearStagingGeometry();
		if (errOut)
			*errOut = msg;
		if (m_host && !msg.isEmpty())
			m_host->logWarn(QStringLiteral("[Chamfer preview] %1").arg(msg));
		return false;
	};

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return fail(QStringLiteral("No viewport"));
	if (params.edgeIndices.empty())
		return fail(QStringLiteral("No edges selected"));

	QString bodyErr;
	const auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8, &bodyErr);
	if (!body)
		return fail(bodyErr);

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::chamferEdgesToHandle(body->worldShape(), params.edgeIndices, params.distanceMm, result, &err)
		|| result.isNull())
		return fail(QString::fromStdString(err.empty() ? "Chamfer failed" : err));

	const osg::Vec4 rgba(0.85f, 0.75f, 0.35f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut) ? true : fail(errOut ? *errOut : QStringLiteral("preview failed"));
}

void PluginGeometryHostImpl::chamferEdgesToBrep(IPluginDocument* doc, const PluginSketchChamferParams& params,
												PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (params.edgeIndices.empty() && !params.allEdges && params.edgeSelectUtf8.empty())
	{
		onFinished(false, QStringLiteral("No edges selected"), {});
		return;
	}

	auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8);
	if (!body)
	{
		onFinished(false, QStringLiteral("Parametric Body not found or has no solid tip"), {});
		return;
	}

	std::vector<int> edges = params.edgeIndices;
	if (!params.edgeSelectUtf8.empty() && !params.allEdges)
	{
		edges.clear();
		std::string selErr;
		const bool okSel =
			(params.edgeSelectUtf8 == "top_boundary")
				? geoalgo::selectTopBoundaryEdgeIndices(body->worldShape(), edges, &selErr)
				: geoalgo::selectLongestEdgeIndices(body->worldShape(),
													params.edgeSelectCount > 0 ? params.edgeSelectCount : 4, edges,
													&selErr);
		if (!okSel || edges.empty())
		{
			onFinished(false, QString::fromStdString(selErr.empty() ? "edge select failed" : selErr), {});
			return;
		}
	}
	else if (params.allEdges)
	{
		edges.clear();
		const int n = geoalgo::shapeHandleEdgeCount(body->worldShape());
		edges.reserve(n > 0 ? n : 0);
		for (int i = 0; i < n; ++i)
			edges.push_back(i);
		if (edges.empty())
		{
			onFinished(false, QStringLiteral("Tip has no edges"), {});
			return;
		}
	}

	body->addChamfer(edges, params.distanceMm);
	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, false, std::move(onFinished));
}

bool PluginGeometryHostImpl::previewSketchRevolve(IPluginDocument* doc, const std::vector<float>& profilePolylineXyzMm,
												  const PluginSketchRevolveParams& params, QString* errOut)
{
	auto fail = [&](const QString& msg) -> bool
	{
		cloudsim::host::DocumentHost* page = pageFromDoc(doc);
		OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
		if (osg)
			osg->clearStagingGeometry();
		if (errOut)
			*errOut = msg;
		if (m_host && !msg.isEmpty())
			m_host->logWarn(QStringLiteral("[Revolve preview] %1").arg(msg));
		return false;
	};

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return fail(QStringLiteral("No viewport"));
	if (profilePolylineXyzMm.size() < 12)
		return fail(QStringLiteral("Revolve profile too short"));

	const bool cut = (params.mode == PluginSketchRevolveMode::Cut);
	const geoalgo::ShapeHandle* basePtr = nullptr;
	geoalgo::ShapeHandle baseOwned;
	if (!params.targetParametricBackendIdUtf8.empty())
	{
		auto body = std::dynamic_pointer_cast<ParametricBrepBackendData>(
			page->backend().getData(params.targetParametricBackendIdUtf8));
		if (body && !body->worldShape().isNull())
		{
			baseOwned = body->worldShape();
			basePtr = &baseOwned;
		}
	}
	if (cut && !basePtr)
		return fail(QStringLiteral("RevolveCut requires existing solid"));

	geoalgo::SketchRevolveParams rp;
	rp.mode = cut ? geoalgo::SketchRevolveMode::Cut : geoalgo::SketchRevolveMode::Boss;
	rp.angleDeg = params.angleDeg;
	rp.axisOx = params.axisOx;
	rp.axisOy = params.axisOy;
	rp.axisOz = params.axisOz;
	rp.axisDx = params.axisDx;
	rp.axisDy = params.axisDy;
	rp.axisDz = params.axisDz;

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::sketchRevolvePolylineToHandle(profilePolylineXyzMm, rp, basePtr, result, &err) || result.isNull())
		return fail(QString::fromStdString(err.empty() ? "Revolve failed" : err));

	const osg::Vec4 rgba = cut ? osg::Vec4(0.95f, 0.45f, 0.25f, 0.35f) : osg::Vec4(0.35f, 0.65f, 0.95f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut) ? true : fail(errOut ? *errOut : QStringLiteral("preview failed"));
}

void PluginGeometryHostImpl::revolveSketchProfileToBrep(IPluginDocument* doc,
														const std::vector<float>& profilePolylineXyzMm,
														const PluginSketchRevolveParams& params,
														PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (profilePolylineXyzMm.size() < 12)
	{
		onFinished(false, QStringLiteral("Revolve profile too short"), {});
		return;
	}

	const bool cut = (params.mode == PluginSketchRevolveMode::Cut);
	const QString bodyName = params.resultNameUtf8.empty() ? QStringLiteral("ParametricBody")
														   : QString::fromStdString(params.resultNameUtf8);

	std::shared_ptr<ParametricBrepBackendData> body;
	const bool createNew = params.targetParametricBackendIdUtf8.empty();
	if (createNew)
	{
		if (cut)
		{
			onFinished(false, QStringLiteral("RevolveCut requires existing Parametric Body"), {});
			return;
		}
		body = std::make_shared<ParametricBrepBackendData>();
		body->setName(bodyName.toStdString());
	}
	else
	{
		body = std::dynamic_pointer_cast<ParametricBrepBackendData>(
			page->backend().getData(params.targetParametricBackendIdUtf8));
		if (!body)
		{
			onFinished(false, QStringLiteral("Parametric Body not found"), {});
			return;
		}
	}

	std::string sketchId = params.sketchIdUtf8;
	if (sketchId.empty() || !body->findFeature(sketchId))
		sketchId = body->addSketch(toParametricPlane(params.plane), "RevolveProfile");
	body->setProfile(sketchId, profilePolylineXyzMm);
	if (ParametricFeature* sk = body->findFeature(sketchId))
	{
		sk->plane = toParametricPlane(params.plane);
		if (!params.sketchDocumentJsonUtf8.empty())
			sk->sketchDocumentJson = params.sketchDocumentJsonUtf8;
	}

	const std::string revolveId = body->addRevolve(sketchId, params.angleDeg, params.axisOx, params.axisOy, params.axisOz,
												   params.axisDx, params.axisDy, params.axisDz, cut);
	if (ParametricFeature* rf = body->findFeature(revolveId))
		rf->profileXyzMm = profilePolylineXyzMm;

	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, createNew, std::move(onFinished));
}

bool PluginGeometryHostImpl::previewLinearPattern(IPluginDocument* doc, const PluginSketchLinearPatternParams& params,
												  QString* errOut)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return clearStagingAndWarn(m_host, doc, QStringLiteral("LinearPattern preview"), QStringLiteral("No viewport"),
								   errOut);
	if (params.count < 2)
		return clearStagingAndWarn(m_host, doc, QStringLiteral("LinearPattern preview"),
								   QStringLiteral("Pattern count must be >= 2"), errOut);

	QString bodyErr;
	const auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8, &bodyErr);
	if (!body)
		return clearStagingAndWarn(m_host, doc, QStringLiteral("LinearPattern preview"), bodyErr, errOut);

	geoalgo::SketchLinearPatternParams pp;
	pp.count = params.count;
	pp.dxMm = params.dxMm;
	pp.dyMm = params.dyMm;
	pp.dzMm = params.dzMm;

	geoalgo::ShapeHandle seed;
	geoalgo::ShapeHandle tipCopy;
	const geoalgo::ShapeHandle* fuseOnto = nullptr;
	QString seedErr;
	if (!resolvePatternSeed(*body, params.sourceFeatureIdUtf8, seed, tipCopy, fuseOnto, &seedErr))
		return clearStagingAndWarn(m_host, doc, QStringLiteral("LinearPattern preview"), seedErr, errOut);

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::linearPatternBodyToHandle(seed, pp, result, &err, fuseOnto) || result.isNull())
		return clearStagingAndWarn(m_host, doc, QStringLiteral("LinearPattern preview"),
								   QString::fromStdString(err.empty() ? "LinearPattern failed" : err), errOut);

	const osg::Vec4 rgba(0.55f, 0.55f, 0.95f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut)
			   ? true
			   : clearStagingAndWarn(m_host, doc, QStringLiteral("LinearPattern preview"),
									 errOut ? *errOut : QStringLiteral("preview failed"), errOut);
}

void PluginGeometryHostImpl::linearPatternBodyToBrep(IPluginDocument* doc, const PluginSketchLinearPatternParams& params,
													 PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (params.count < 2)
	{
		onFinished(false, QStringLiteral("Pattern count must be >= 2"), {});
		return;
	}

	auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8);
	if (!body)
	{
		onFinished(false, QStringLiteral("Parametric Body not found or has no solid tip"), {});
		return;
	}

	body->addLinearPattern(params.count, params.dxMm, params.dyMm, params.dzMm, params.sourceFeatureIdUtf8);
	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, false, std::move(onFinished));
}

bool PluginGeometryHostImpl::previewCircularPattern(IPluginDocument* doc, const PluginSketchCircularPatternParams& params,
													QString* errOut)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return clearStagingAndWarn(m_host, doc, QStringLiteral("CircularPattern preview"), QStringLiteral("No viewport"),
								   errOut);
	if (params.count < 2)
		return clearStagingAndWarn(m_host, doc, QStringLiteral("CircularPattern preview"),
								   QStringLiteral("Pattern count must be >= 2"), errOut);

	QString bodyErr;
	const auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8, &bodyErr);
	if (!body)
		return clearStagingAndWarn(m_host, doc, QStringLiteral("CircularPattern preview"), bodyErr, errOut);

	geoalgo::ShapeHandle seed;
	geoalgo::ShapeHandle tipCopy;
	const geoalgo::ShapeHandle* fuseOnto = nullptr;
	QString seedErr;
	if (!resolvePatternSeed(*body, params.sourceFeatureIdUtf8, seed, tipCopy, fuseOnto, &seedErr))
		return clearStagingAndWarn(m_host, doc, QStringLiteral("CircularPattern preview"), seedErr, errOut);

	geoalgo::SketchCircularPatternParams pp;
	pp.count = params.count;
	pp.angleDeg = params.angleDeg;
	pp.axisOx = params.axisOx;
	pp.axisOy = params.axisOy;
	pp.axisOz = params.axisOz;
	pp.axisDx = params.axisDx;
	pp.axisDy = params.axisDy;
	pp.axisDz = params.axisDz;

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::circularPatternBodyToHandle(seed, pp, result, &err, fuseOnto) || result.isNull())
		return clearStagingAndWarn(m_host, doc, QStringLiteral("CircularPattern preview"),
								   QString::fromStdString(err.empty() ? "CircularPattern failed" : err), errOut);

	const osg::Vec4 rgba(0.55f, 0.75f, 0.95f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut)
			   ? true
			   : clearStagingAndWarn(m_host, doc, QStringLiteral("CircularPattern preview"),
									 errOut ? *errOut : QStringLiteral("preview failed"), errOut);
}

void PluginGeometryHostImpl::circularPatternBodyToBrep(IPluginDocument* doc, const PluginSketchCircularPatternParams& params,
													   PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (params.count < 2)
	{
		onFinished(false, QStringLiteral("Pattern count must be >= 2"), {});
		return;
	}

	auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8);
	if (!body)
	{
		onFinished(false, QStringLiteral("Parametric Body not found or has no solid tip"), {});
		return;
	}

	body->addCircularPattern(params.count, params.angleDeg, params.axisOx, params.axisOy, params.axisOz, params.axisDx,
							 params.axisDy, params.axisDz, params.sourceFeatureIdUtf8);
	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, false, std::move(onFinished));
}

bool PluginGeometryHostImpl::previewMirror3d(IPluginDocument* doc, const PluginSketchMirror3dParams& params,
											 QString* errOut)
{
	auto fail = [&](const QString& msg) -> bool
	{
		cloudsim::host::DocumentHost* page = pageFromDoc(doc);
		OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
		if (osg)
			osg->clearStagingGeometry();
		if (errOut)
			*errOut = msg;
		if (m_host && !msg.isEmpty())
			m_host->logWarn(QStringLiteral("[Mirror3D preview] %1").arg(msg));
		return false;
	};

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return fail(QStringLiteral("No viewport"));

	QString bodyErr;
	const auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8, &bodyErr);
	if (!body)
		return fail(bodyErr);

	geoalgo::SketchMirror3dParams mp;
	mp.ox = params.plane.origin.x;
	mp.oy = params.plane.origin.y;
	mp.oz = params.plane.origin.z;
	mp.nx = params.plane.normal.x;
	mp.ny = params.plane.normal.y;
	mp.nz = params.plane.normal.z;
	mp.keepOriginal = params.keepOriginal;

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::mirrorBodyToHandle(body->worldShape(), mp, result, &err) || result.isNull())
		return fail(QString::fromStdString(err.empty() ? "Mirror3D failed" : err));

	const osg::Vec4 rgba(0.75f, 0.45f, 0.95f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut) ? true : fail(errOut ? *errOut : QStringLiteral("preview failed"));
}

void PluginGeometryHostImpl::mirror3dBodyToBrep(IPluginDocument* doc, const PluginSketchMirror3dParams& params,
												PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}

	auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8);
	if (!body)
	{
		onFinished(false, QStringLiteral("Parametric Body not found or has no solid tip"), {});
		return;
	}

	body->addMirror3D(toParametricPlane(params.plane), params.keepOriginal);
	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, false, std::move(onFinished));
}

bool PluginGeometryHostImpl::previewSketchLoft(IPluginDocument* doc, const std::vector<float>& profilePolylineAXyzMm,
											   const std::vector<float>& profilePolylineBXyzMm,
											   const PluginSketchLoftParams& params, QString* errOut)
{
	auto fail = [&](const QString& msg) -> bool
	{
		cloudsim::host::DocumentHost* page = pageFromDoc(doc);
		OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
		if (osg)
			osg->clearStagingGeometry();
		if (errOut)
			*errOut = msg;
		if (m_host && !msg.isEmpty())
			m_host->logWarn(QStringLiteral("[Loft preview] %1").arg(msg));
		return false;
	};

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return fail(QStringLiteral("No viewport"));
	if (profilePolylineAXyzMm.size() < 12 || profilePolylineBXyzMm.size() < 12)
		return fail(QStringLiteral("Loft profiles too short"));

	const bool cut = (params.mode == PluginSketchLoftMode::Cut);
	const geoalgo::ShapeHandle* basePtr = nullptr;
	geoalgo::ShapeHandle baseOwned;
	if (!params.targetParametricBackendIdUtf8.empty())
	{
		auto body = std::dynamic_pointer_cast<ParametricBrepBackendData>(
			page->backend().getData(params.targetParametricBackendIdUtf8));
		if (body && !body->worldShape().isNull())
		{
			baseOwned = body->worldShape();
			basePtr = &baseOwned;
		}
	}
	if (cut && !basePtr)
		return fail(QStringLiteral("LoftCut requires existing solid"));

	geoalgo::SketchLoftParams lp;
	lp.mode = cut ? geoalgo::SketchLoftMode::Cut : geoalgo::SketchLoftMode::Boss;

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::sketchLoftPolylinesToHandle(profilePolylineAXyzMm, profilePolylineBXyzMm, lp, basePtr, result, &err)
		|| result.isNull())
		return fail(QString::fromStdString(err.empty() ? "Loft failed" : err));

	const osg::Vec4 rgba = cut ? osg::Vec4(0.95f, 0.45f, 0.25f, 0.35f) : osg::Vec4(0.35f, 0.65f, 0.95f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut) ? true : fail(errOut ? *errOut : QStringLiteral("preview failed"));
}

void PluginGeometryHostImpl::loftSketchProfilesToBrep(IPluginDocument* doc,
													  const std::vector<float>& profilePolylineAXyzMm,
													  const std::vector<float>& profilePolylineBXyzMm,
													  const PluginSketchLoftParams& params,
													  PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (profilePolylineAXyzMm.size() < 12 || profilePolylineBXyzMm.size() < 12)
	{
		onFinished(false, QStringLiteral("Loft profiles too short"), {});
		return;
	}

	const bool cut = (params.mode == PluginSketchLoftMode::Cut);
	const QString bodyName = params.resultNameUtf8.empty() ? QStringLiteral("ParametricBody")
														   : QString::fromStdString(params.resultNameUtf8);

	std::shared_ptr<ParametricBrepBackendData> body;
	const bool createNew = params.targetParametricBackendIdUtf8.empty();
	if (createNew)
	{
		if (cut)
		{
			onFinished(false, QStringLiteral("LoftCut requires existing Parametric Body"), {});
			return;
		}
		body = std::make_shared<ParametricBrepBackendData>();
		body->setName(bodyName.toStdString());
	}
	else
	{
		body = std::dynamic_pointer_cast<ParametricBrepBackendData>(
			page->backend().getData(params.targetParametricBackendIdUtf8));
		if (!body)
		{
			onFinished(false, QStringLiteral("Parametric Body not found"), {});
			return;
		}
	}

	std::string sketchAId = params.sketchAIdUtf8;
	std::string sketchBId = params.sketchBIdUtf8;
	if (sketchAId.empty() || !body->findFeature(sketchAId))
		sketchAId = body->addSketch(toParametricPlane(params.planeA), "LoftProfileA");
	if (sketchBId.empty() || !body->findFeature(sketchBId))
		sketchBId = body->addSketch(toParametricPlane(params.planeB), "LoftProfileB");
	body->setProfile(sketchAId, profilePolylineAXyzMm);
	body->setProfile(sketchBId, profilePolylineBXyzMm);
	if (ParametricFeature* sk = body->findFeature(sketchAId))
	{
		sk->plane = toParametricPlane(params.planeA);
		if (!params.sketchADocumentJsonUtf8.empty())
			sk->sketchDocumentJson = params.sketchADocumentJsonUtf8;
	}
	if (ParametricFeature* sk = body->findFeature(sketchBId))
	{
		sk->plane = toParametricPlane(params.planeB);
		if (!params.sketchBDocumentJsonUtf8.empty())
			sk->sketchDocumentJson = params.sketchBDocumentJsonUtf8;
	}

	const std::string loftId = body->addLoft(sketchAId, sketchBId, cut);
	if (ParametricFeature* lf = body->findFeature(loftId))
	{
		lf->profileXyzMm = profilePolylineAXyzMm;
		lf->pathXyzMm = profilePolylineBXyzMm;
	}

	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, createNew, std::move(onFinished));
}

bool PluginGeometryHostImpl::previewShellFaces(IPluginDocument* doc, const PluginSketchShellParams& params,
											   QString* errOut)
{
	auto fail = [&](const QString& msg) -> bool
	{
		cloudsim::host::DocumentHost* page = pageFromDoc(doc);
		OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
		if (osg)
			osg->clearStagingGeometry();
		if (errOut)
			*errOut = msg;
		if (m_host && !msg.isEmpty())
			m_host->logWarn(QStringLiteral("[Shell preview] %1").arg(msg));
		return false;
	};

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return fail(QStringLiteral("No viewport"));
	if (params.faceIndices.empty())
		return fail(QStringLiteral("No faces selected"));

	QString bodyErr;
	const auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8, &bodyErr);
	if (!body)
		return fail(bodyErr);

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::shellFacesToHandle(body->worldShape(), params.faceIndices, params.thicknessMm, result, &err)
		|| result.isNull())
		return fail(QString::fromStdString(err.empty() ? "Shell failed" : err));

	const osg::Vec4 rgba(0.95f, 0.55f, 0.35f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut) ? true : fail(errOut ? *errOut : QStringLiteral("preview failed"));
}

void PluginGeometryHostImpl::shellFacesToBrep(IPluginDocument* doc, const PluginSketchShellParams& params,
											  PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (params.faceIndices.empty())
	{
		onFinished(false, QStringLiteral("No faces selected"), {});
		return;
	}

	auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8);
	if (!body)
	{
		onFinished(false, QStringLiteral("Parametric Body not found or has no solid tip"), {});
		return;
	}

	body->addShell(params.faceIndices, params.thicknessMm);
	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, false, std::move(onFinished));
}

bool PluginGeometryHostImpl::previewDraftFaces(IPluginDocument* doc, const PluginSketchDraftParams& params,
											   QString* errOut)
{
	auto fail = [&](const QString& msg) -> bool
	{
		cloudsim::host::DocumentHost* page = pageFromDoc(doc);
		OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
		if (osg)
			osg->clearStagingGeometry();
		if (errOut)
			*errOut = msg;
		if (m_host && !msg.isEmpty())
			m_host->logWarn(QStringLiteral("[Draft preview] %1").arg(msg));
		return false;
	};

	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return fail(QStringLiteral("No viewport"));
	if (params.faceIndices.empty())
		return fail(QStringLiteral("No faces selected"));

	QString bodyErr;
	const auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8, &bodyErr);
	if (!body)
		return fail(bodyErr);

	PluginSketchPlane neutral = params.neutralPlane;
	if (!neutral.isPlanar)
	{
		neutral.isPlanar = true;
		neutral.origin = {0, 0, 0};
		neutral.normal = {0, 0, 1};
	}

	geoalgo::ShapeHandle result;
	std::string err;
	if (!geoalgo::draftFacesToHandle(body->worldShape(), params.faceIndices, params.angleDeg, neutral.normal.x,
									 neutral.normal.y, neutral.normal.z, neutral.origin.x, neutral.origin.y,
									 neutral.origin.z, result, &err)
		|| result.isNull())
		return fail(QString::fromStdString(err.empty() ? "Draft failed" : err));

	const osg::Vec4 rgba(0.55f, 0.75f, 0.95f, 0.35f);
	return previewShapeStaging(osg, result, rgba, errOut) ? true : fail(errOut ? *errOut : QStringLiteral("preview failed"));
}

void PluginGeometryHostImpl::draftFacesToBrep(IPluginDocument* doc, const PluginSketchDraftParams& params,
											  PluginGeometryFinishedFn onFinished)
{
	if (!onFinished)
		return;
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page || !m_host || !m_host->mainWindowHost())
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (params.faceIndices.empty())
	{
		onFinished(false, QStringLiteral("No faces selected"), {});
		return;
	}

	auto body = parametricBodyWithTip(page, params.targetParametricBackendIdUtf8);
	if (!body)
	{
		onFinished(false, QStringLiteral("Parametric Body not found or has no solid tip"), {});
		return;
	}

	PluginSketchPlane neutral = params.neutralPlane;
	if (!neutral.isPlanar)
	{
		neutral.isPlanar = true;
		neutral.origin = {0, 0, 0};
		neutral.normal = {0, 0, 1};
	}

	body->addDraft(params.faceIndices, params.angleDeg, toParametricPlane(neutral));
	std::string rebuildErr;
	if (!body->rebuild(&rebuildErr))
	{
		onFinished(false, QString::fromStdString(rebuildErr.empty() ? "rebuild failed" : rebuildErr), {});
		return;
	}
	finishParametricBodyJob(m_host, page, body, false, std::move(onFinished));
}

void PluginGeometryHostImpl::projectBrepHlrToDrawing(IPluginDocument* doc, const std::string& backendIdUtf8,
													 PluginDrawingHlrFinishedFn onFinished)
{
	PluginDrawingProjectParams params;
	params.thirdAngle = false;
	params.includeIso = false;
	params.includeSection = false;
	projectBrepToEngineeringDrawing(doc, backendIdUtf8, params, std::move(onFinished));
}

void PluginGeometryHostImpl::projectBrepToEngineeringDrawing(IPluginDocument* doc, const std::string& backendIdUtf8,
															 const PluginDrawingProjectParams& params,
															 PluginDrawingHlrFinishedFn onFinished)
{
	if (!m_host || !onFinished)
	{
		return;
	}
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	if (!page)
	{
		onFinished(false, QStringLiteral("No active document"), {});
		return;
	}
	if (backendIdUtf8.empty())
	{
		onFinished(false, QStringLiteral("Empty backendId"), {});
		return;
	}

	geoalgo::ShapeHandle shape;
	geoalgo::WorkpieceRef shapeRef;
	std::string resolveErr;
	const QString stepPath = stepPathForBackend(page, backendIdUtf8);
	if (geometry_backend_ops::resolveWorkpieceShape(backendIdUtf8, page->backend(), stepPath.toStdString(), shape,
													shapeRef, &resolveErr) ==
		geometry_backend_ops::WorkpieceShapeSource::Unavailable)
	{
		onFinished(false, QString::fromStdString(resolveErr.empty() ? "Cannot resolve B-rep shape" : resolveErr), {});
		return;
	}

	struct DrawingJobKey
	{
		int pipelineVersion = 3; // 离散优先，失效解析假圆缓存
		std::string backendId;
		bool thirdAngle = false;
		bool includeIso = false;
		bool includeSection = false;
		bool customSection = false;
		bool coarseView = false;
		int sectionPlane = 0;
		long long ox = 0, oy = 0, oz = 0, nx = 0, ny = 0, nz = 0;
		bool operator==(const DrawingJobKey& o) const
		{
			return pipelineVersion == o.pipelineVersion && backendId == o.backendId && thirdAngle == o.thirdAngle &&
				   includeIso == o.includeIso && includeSection == o.includeSection &&
				   customSection == o.customSection && coarseView == o.coarseView && sectionPlane == o.sectionPlane &&
				   ox == o.ox && oy == o.oy && oz == o.oz && nx == o.nx && ny == o.ny && nz == o.nz;
		}
	};
	auto makeKey = [&](const PluginDrawingProjectParams& p) {
		DrawingJobKey k;
		k.pipelineVersion = 3;
		k.backendId = backendIdUtf8;
		k.thirdAngle = p.thirdAngle;
		k.includeIso = p.includeIso;
		k.includeSection = p.includeSection;
		k.customSection = p.customSection;
		k.coarseView = p.coarseView;
		k.sectionPlane = p.sectionPlane;
		auto q = [](double v) { return static_cast<long long>(std::llround(v * 1000.0)); };
		k.ox = q(p.sectionOriginMm[0]);
		k.oy = q(p.sectionOriginMm[1]);
		k.oz = q(p.sectionOriginMm[2]);
		k.nx = q(p.sectionNormal[0]);
		k.ny = q(p.sectionNormal[1]);
		k.nz = q(p.sectionNormal[2]);
		return k;
	};
	const DrawingJobKey jobKey = makeKey(params);

	struct DrawingJobSlot
	{
		DrawingJobKey key;
		bool running = false;
		PluginDrawingHlrResult cached;
		bool hasCache = false;
		std::vector<PluginDrawingHlrFinishedFn> waiters;
	};
	static std::mutex s_drawingJobMutex;
	static std::vector<DrawingJobSlot> s_drawingJobs;

	{
		std::lock_guard<std::mutex> lock(s_drawingJobMutex);
		for (DrawingJobSlot& slot : s_drawingJobs)
		{
			if (!(slot.key == jobKey))
				continue;
			if (slot.hasCache)
			{
				onFinished(true, QString(), slot.cached);
				return;
			}
			if (slot.running)
			{
				// 合并同参数请求，避免重复 HLR
				slot.waiters.push_back(std::move(onFinished));
				return;
			}
		}
	}

	struct HlrWorkResult
	{
		geoalgo::HlrDrawingBundle bundle;
		std::string error;
		bool ok = false;
	};
	auto result = std::make_shared<HlrWorkResult>();
	const PluginDrawingProjectParams jobParams = params;
	const geoalgo::ShapeHandle shapeCopy = shape.clone();

	{
		std::lock_guard<std::mutex> lock(s_drawingJobMutex);
		DrawingJobSlot* slot = nullptr;
		for (DrawingJobSlot& s : s_drawingJobs)
		{
			if (s.key == jobKey)
			{
				slot = &s;
				break;
			}
		}
		if (!slot)
		{
			s_drawingJobs.push_back(DrawingJobSlot{});
			slot = &s_drawingJobs.back();
			slot->key = jobKey;
		}
		slot->running = true;
		slot->hasCache = false;
		slot->waiters.clear();
		slot->waiters.push_back(std::move(onFinished));
	}

	m_host->enqueueJob(
		QStringLiteral("Engineering drawing projection"),
		[result, shapeCopy, jobParams](const PluginJobProgressFn& report)
		{
			report(0.2, QStringLiteral("HLR..."));
			geoalgo::TessellateParams tess;
			// 工程图：尺度相关弦高；过密（0.5°/亚丝米）会让复杂模型 HLR 卡死
			double diag = 100.0;
			const geoalgo::ShapeHandle::BoundsMm bb = shapeCopy.boundingBoxMm();
			if (bb.valid)
			{
				const double dx = bb.maxX - bb.minX;
				const double dy = bb.maxY - bb.minY;
				const double dz = bb.maxZ - bb.minZ;
				diag = std::sqrt(dx * dx + dy * dy + dz * dz);
				if (!(diag > 1e-9))
					diag = 100.0;
			}
			tess.linearDeflectionMm = (std::max)(0.05, (std::min)(0.5, diag * 0.002));
			tess.linearDeflectionRelative = false;
			// 过粗转角会让圆呈折线且 HLR 短弧易丢
			tess.angularDeflectionDeg = 4.0;
			const geoalgo::HlrProjectionAngle angle =
				jobParams.thirdAngle ? geoalgo::HlrProjectionAngle::Third : geoalgo::HlrProjectionAngle::First;
			geoalgo::DrawingSectionPlane secPlane = geoalgo::DrawingSectionPlane::FrontParallel;
			if (jobParams.sectionPlane == 1)
				secPlane = geoalgo::DrawingSectionPlane::TopParallel;
			else if (jobParams.sectionPlane == 2)
				secPlane = geoalgo::DrawingSectionPlane::RightParallel;
			geoalgo::DrawingHlrRunOptions opts;
			opts.useMeshHlr = jobParams.coarseView;
			opts.nbIso = 0;
			result->ok = geoalgo::projectShapeHlrDrawingBundle(
				shapeCopy, angle, jobParams.includeIso, jobParams.includeSection, secPlane, jobParams.customSection,
				jobParams.sectionOriginMm, jobParams.sectionNormal, tess, opts, result->bundle, &result->error);
			report(1.0, QStringLiteral("Done"));
		},
		[result, jobKey](const bool threw, const QString& throwMessage)
		{
			PluginDrawingHlrResult out;
			bool ok = false;
			QString err;
			if (threw)
			{
				err = throwMessage;
			}
			else if (!result->ok)
			{
				err = QString::fromStdString(result->error);
			}
			else
			{
				auto packView = [](const char* id, const geoalgo::HlrViewPolylines& src) {
					PluginDrawingHlrViewResult v;
					v.viewId = id;
					auto toXy = [](const std::vector<geoalgo::Polyline3d>& polys) {
						std::vector<std::vector<float>> outPolys;
						outPolys.reserve(polys.size());
						for (const geoalgo::Polyline3d& p : polys)
						{
							std::vector<float> xy;
							xy.reserve(p.xyz.size() * 2 / 3);
							for (std::size_t i = 0; i + 2 < p.xyz.size(); i += 3)
							{
								xy.push_back(p.xyz[i]);
								xy.push_back(p.xyz[i + 1]);
							}
							if (xy.size() >= 4)
								outPolys.push_back(std::move(xy));
						}
						return outPolys;
					};
					v.visibleXy = toXy(src.visible);
					v.hiddenXy = toXy(src.hidden);
					return v;
				};

				out.views.push_back(packView("front", result->bundle.front));
				out.views.push_back(packView("top", result->bundle.top));
				out.views.push_back(packView("right", result->bundle.right));
				if (result->bundle.hasIso)
					out.views.push_back(packView("iso", result->bundle.iso));
				if (result->bundle.hasSection)
					out.views.push_back(packView("section", result->bundle.section));
				ok = true;
			}

			std::vector<PluginDrawingHlrFinishedFn> waiters;
			{
				std::lock_guard<std::mutex> lock(s_drawingJobMutex);
				for (DrawingJobSlot& slot : s_drawingJobs)
				{
					if (!(slot.key == jobKey))
						continue;
					slot.running = false;
					if (ok)
					{
						slot.cached = out;
						slot.hasCache = true;
					}
					waiters.swap(slot.waiters);
					break;
				}
			}
			for (PluginDrawingHlrFinishedFn& fn : waiters)
			{
				if (fn)
					fn(ok, err, out);
			}
		});
}

void PluginGeometryHostImpl::setOriginReferenceVisibility(IPluginDocument* doc,
														  const PluginOriginReferenceVisibility& visibility)
{
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
		return;
	osg->setOriginReferenceVisibility(visibility.originPoint, visibility.planeXY, visibility.planeXZ,
									  visibility.planeYZ);
}
