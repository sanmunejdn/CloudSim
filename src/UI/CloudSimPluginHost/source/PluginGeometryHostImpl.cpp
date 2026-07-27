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
#include "SketchExtrude.h"
#include "SketchSweep.h"
#include "MeshDiscretize.h"
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
#include <functional>
#include <memory>

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
	endSketchInput(doc);
	clearSketchSupportPlanePick();
	cloudsim::host::DocumentHost* page = pageFromDoc(doc);
	OsgWidget* osg = page ? widgetOsgFromPage(page) : nullptr;
	if (!osg)
	{
		onFinished(false, QStringLiteral("3D viewport unavailable"), PluginOriginPlaneKind::XY, {});
		return;
	}

	const auto sessionDone = std::make_shared<bool>(false);
	m_supportPlanePickDone = sessionDone;

	const auto finishOrigin = [sessionDone, this, osg, onFinished](bool ok, int planeIndex)
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
		}
		m_supportPlanePickDone.reset();
		if (!ok || planeIndex < 0 || planeIndex > 2)
		{
			onFinished(false, QStringLiteral("已取消草图平面选择"), PluginOriginPlaneKind::XY, {});
			return;
		}
		const auto kind = static_cast<PluginOriginPlaneKind>(planeIndex);
		onFinished(true, QString(), kind, makeOriginPluginPlane(planeIndex));
	};

	// 模型面与基面并行：基面未命中时 OsgWidget 放行给 MeshFace 拾取
	osg->setSelectionActive(true);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(true);
	m_supportPlaneFaceConn = QObject::connect(
		osg, &OsgWidget::meshPickCommitted, m_host,
		[sessionDone, this, osg, page, doc, onFinished](PickResult pick, int pickKindInt)
		{
			if (*sessionDone || !pick.hit)
				return;
			if (static_cast<PickKind>(pickKindInt) != PickKind::MeshFace)
				return;

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
				// 未结束会话，允许继续点选
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
			onFinished(true, QString(), PluginOriginPlaneKind::XY, plane);
		});

	osg->beginOriginPlaneSelection(finishOrigin);
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
	ep.reversed = params.reversed;
	ep.draftAngleDeg = params.draftAngleDeg;
	if (params.endCondition == PluginSketchExtrudeEnd::UpToFace)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::UpToFace;
	else if (params.endCondition == PluginSketchExtrudeEnd::MidPlane)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::MidPlane;
	else if (params.endCondition == PluginSketchExtrudeEnd::ThroughAll)
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::ThroughAll;
	else
		ep.endCondition = geoalgo::SketchExtrudeEndCondition::Blind;
	ep.hasUpToFace = params.hasUpToFacePlane;
	ep.upOriginX = params.upToFacePlane.origin.x;
	ep.upOriginY = params.upToFacePlane.origin.y;
	ep.upOriginZ = params.upToFacePlane.origin.z;
	ep.upNormalX = params.upToFacePlane.normal.x;
	ep.upNormalY = params.upToFacePlane.normal.y;
	ep.upNormalZ = params.upToFacePlane.normal.z;
	ep.originX = plane.origin.x;
	ep.originY = plane.origin.y;
	ep.originZ = plane.origin.z;
	ep.normalX = plane.normal.x;
	ep.normalY = plane.normal.y;
	ep.normalZ = plane.normal.z;

	const geoalgo::ShapeHandle* basePtr = nullptr;
	geoalgo::ShapeHandle baseOwned;
	const bool needBase = (params.mode == PluginSketchExtrudeMode::Pocket)
						  || (params.endCondition == PluginSketchExtrudeEnd::ThroughAll);
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
	if (!params.sketchDocumentJsonUtf8.empty())
	{
		if (ParametricFeature* sk = body->findFeature(sketchId))
			sk->sketchDocumentJson = params.sketchDocumentJsonUtf8;
	}
	if (pocket)
		body->addPocket(sketchId, params.lengthMm, params.reversed);
	else
		body->addPad(sketchId, params.lengthMm, params.reversed);
	if (ParametricFeature* extrudeFeat = body->findFeature(body->features().back().id))
	{
		if (params.endCondition == PluginSketchExtrudeEnd::UpToFace)
			extrudeFeat->endCondition = ParametricExtrudeEnd::UpToFace;
		else if (params.endCondition == PluginSketchExtrudeEnd::MidPlane)
			extrudeFeat->endCondition = ParametricExtrudeEnd::MidPlane;
		else if (params.endCondition == PluginSketchExtrudeEnd::ThroughAll)
			extrudeFeat->endCondition = ParametricExtrudeEnd::ThroughAll;
		else
			extrudeFeat->endCondition = ParametricExtrudeEnd::Blind;
		if (params.hasUpToFacePlane)
		{
			extrudeFeat->hasUpToFacePlane = true;
			extrudeFeat->upToFacePlane = toParametricPlane(params.upToFacePlane);
		}
		extrudeFeat->upToFaceBackendId = params.upToFaceBackendIdUtf8;
		extrudeFeat->upToFaceIndex = params.upToFaceIndex;
		extrudeFeat->draftAngleDeg = params.draftAngleDeg;
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
