/// @file PluginGeometryHostImpl.cpp
/// @brief PluginGeometryHostImpl 实现

#include "PluginGeometryHostImpl.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentGeometryOps.h"
#include "DocumentHost.h"
#include "GeometryBackendOps.h"
#include "GeometryRef.h"
#include "OsgWidget.h"
#include "PickTypes.h"
#include "PluginDocumentAdapter.h"
#include "PluginHostContext.h"
#include "WidgetDocumentAccess.h"

#include <QHash>
#include <QMetaObject>
#include <QSet>
#include <QString>
#include <QTimer>
#include <QVector>
#include <functional>
#include <memory>

#include <FeatureSpec.h>

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
			onFinished(true, QString(), jobResult);
		});
}

} // namespace

PluginGeometryHostImpl::PluginGeometryHostImpl(PluginHostContext* hostContext) : m_host(hostContext) {}

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
		const bool isBrepModel = data->className() == "BrepModel";
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
			if (!isStepPath(stepPath))
			{
				complete(false, QStringLiteral("STEP source path unavailable"), {});
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
			outRef.stepPathUtf8 = ref.stepPathUtf8;
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
			}
			complete(true, QString(), outRef);
		});

	QTimer::singleShot(30000, m_host, [complete]() { complete(false, QStringLiteral("Pick timeout"), {}); });
}
