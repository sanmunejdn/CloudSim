/// @file AiHostButtonApiDispatch.cpp
/// @brief Dock 按钮 Host API 分发；缺参时弹出 Agent 对话框

#include "Ai/AiAgentPickDialog.h"
#include "Ai/AiHostButtonApiDispatch.h"
#include "IPluginMainWindowHost.h"
#include "IProcessFlowAiBridge.h"
#include "PluginDocumentAdapter.h"
#include "PluginHostContext.h"

#include "IPluginDocument.h"
#include "IPluginGeometryHost.h"
#include "IPluginLabelingHost.h"
#include "IPluginPointCloudHost.h"
#include "PluginGeometryTypes.h"
#include "PluginLabelingTypes.h"
#include "PluginPointCloudTypes.h"

#include <QEventLoop>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QWidget>

#include <functional>
#include <string>

namespace AiHostButtonApiDispatch
{
namespace
{
// Agent 面板路径关闭整窗模态，缺参直接失败
thread_local bool g_allowModalDialogs = true;

struct SurfaceSessionState
{
	PluginMeshSurfaceReconstructSessionId sessionId;
	std::string meshBackendId;
};

struct TubularSessionState
{
	PluginTubularGrindingSessionId sessionId;
	std::string meshBackendId;
};

struct LabelingSessionState
{
	PluginLabelingSessionId sessionId = kInvalidLabelingSessionId;
	std::string backendId;
};

SurfaceSessionState g_surface;
TubularSessionState g_tubular;
LabelingSessionState g_labeling;
PluginGeometryStepRef g_lastEdge;
PluginGeometryStepRef g_lastFace;
PluginGeometryStepRef g_lastFaceA;
PluginGeometryStepRef g_lastFaceB;
std::vector<float> g_lastPolylineXyz;

QWidget* dialogParent(PluginHostContext& host)
{
	if (IPluginMainWindowHost* mw = host.mainWindowHost())
		return mw->mainWindowWidget();
	return nullptr;
}

std::string argString(const nlohmann::json& args, const char* key)
{
	if (!args.contains(key) || !args[key].is_string())
		return {};
	return args[key].get<std::string>();
}

void setErr(QString* outError, const QString& msg)
{
	if (outError)
		*outError = msg;
}

IPluginDocument* requireDoc(PluginHostContext& host, QString* outError)
{
	IPluginDocument* doc = host.activeDocument();
	if (!doc)
		setErr(outError, QStringLiteral("无活动文档。"));
	return doc;
}

IPluginPointCloudHost* requirePc(PluginHostContext& host, QString* outError)
{
	IPluginPointCloudHost* h = host.pointCloudHost();
	if (!h)
		setErr(outError, QStringLiteral("点云宿主不可用。"));
	return h;
}

std::string resolveBackendId(PluginHostContext& host, nlohmann::json& args, QString* outError,
							 AiAgentPickDialog::BackendKindFilter filter, const QString& pickTitle)
{
	std::string id = argString(args, "backend_id");
	if (id.empty())
		id = argString(args, "backendId");
	if (id.empty())
		id = host.selectedBackendId().toStdString();
	if (!id.empty())
		return id;

	if (!g_allowModalDialogs)
	{
		setErr(outError, QStringLiteral("缺少 backend_id（应由确认面板提供）。"));
		return {};
	}

	IPluginDocument* doc = host.activeDocument();
	auto entries = AiAgentPickDialog::listBackends(doc, filter);
	if (entries.empty())
	{
		setErr(outError, QStringLiteral("文档中无可用对象，请先导入。"));
		return {};
	}
	QString picked;
	if (!AiAgentPickDialog::pickOneBackend(dialogParent(host), entries, pickTitle, &picked))
	{
		setErr(outError, QStringLiteral("已取消选择对象。"));
		return {};
	}
	id = picked.toStdString();
	args["backend_id"] = id;
	return id;
}

bool resolveSourceTarget(PluginHostContext& host, nlohmann::json& args, QString* outError,
						 AiAgentPickDialog::BackendKindFilter filter, const QString& title, std::string* sourceId,
						 std::string* targetId)
{
	*sourceId = argString(args, "backend_id");
	if (sourceId->empty())
		*sourceId = argString(args, "source_backend_id");
	if (sourceId->empty())
		*sourceId = host.selectedBackendId().toStdString();
	*targetId = argString(args, "target_backend_id");

	if (!sourceId->empty() && !targetId->empty() && *sourceId != *targetId)
		return true;

	if (!g_allowModalDialogs)
	{
		setErr(outError, QStringLiteral("缺少源/目标对象（应由确认面板提供）。"));
		return false;
	}

	IPluginDocument* doc = host.activeDocument();
	auto entries = AiAgentPickDialog::listBackends(doc, filter);
	if (entries.size() < 2)
	{
		setErr(outError, QStringLiteral("配准需要至少两个对象，请先导入源与目标。"));
		return false;
	}
	QString src;
	QString tgt;
	if (!AiAgentPickDialog::pickSourceAndTarget(dialogParent(host), entries, title, &src, &tgt))
	{
		setErr(outError, QStringLiteral("已取消选择源/目标。"));
		return false;
	}
	*sourceId = src.toStdString();
	*targetId = tgt.toStdString();
	args["backend_id"] = *sourceId;
	args["target_backend_id"] = *targetId;
	return true;
}

bool waitPcJob(const std::function<void(PluginPointCloudFinishedFn)>& start, QString* outError)
{
	QEventLoop loop;
	bool ok = false;
	QString err;
	start([&](bool success, const QString& error, const PluginPointCloudJobResult&)
		  {
			  ok = success;
			  err = error;
			  loop.quit();
		  });
	loop.exec();
	if (!ok)
		setErr(outError, err.isEmpty() ? QStringLiteral("点云操作失败。") : err);
	return ok;
}

bool waitMeshJob(const std::function<void(PluginMeshFinishedFn)>& start, QString* outError)
{
	return waitPcJob(start, outError);
}

bool waitGeomJob(const std::function<void(PluginGeometryFinishedFn)>& start, QString* outError)
{
	QEventLoop loop;
	bool ok = false;
	QString err;
	start([&](bool success, const QString& error, const PluginGeometryJobResult&)
		  {
			  ok = success;
			  err = error;
			  loop.quit();
		  });
	loop.exec();
	if (!ok)
		setErr(outError, err.isEmpty() ? QStringLiteral("几何操作失败。") : err);
	return ok;
}

PluginLabelingSessionConfig defaultLabelingConfig()
{
	PluginLabelingSessionConfig cfg;
	cfg.unlabeledClassId = 0;
	cfg.defaultBrushRadiusPx = 16.f;
	PluginLabelingClassDef c0;
	c0.classId = 0;
	c0.nameUtf8 = "unlabeled";
	PluginLabelingClassDef c1;
	c1.classId = 1;
	c1.nameUtf8 = "class1";
	c1.colorRgb[0] = 1.f;
	c1.colorRgb[1] = 0.2f;
	c1.colorRgb[2] = 0.2f;
	cfg.classes = {c0, c1};
	return cfg;
}

bool ensureLabelingSession(PluginHostContext& host, const std::string& backendId, QString* outError)
{
	IPluginLabelingHost* lh = host.labelingHost();
	IPluginDocument* doc = requireDoc(host, outError);
	if (!lh || !doc)
	{
		if (!lh)
			setErr(outError, QStringLiteral("标注宿主不可用。"));
		return false;
	}
	if (g_labeling.sessionId != kInvalidLabelingSessionId && g_labeling.backendId == backendId)
		return true;
	if (g_labeling.sessionId != kInvalidLabelingSessionId)
	{
		lh->clearLabelingSession(g_labeling.sessionId);
		g_labeling = {};
	}
	QString err;
	g_labeling.sessionId = lh->beginLabelingSession(doc, backendId, defaultLabelingConfig(), &err);
	g_labeling.backendId = backendId;
	if (g_labeling.sessionId == kInvalidLabelingSessionId)
	{
		setErr(outError, err.isEmpty() ? QStringLiteral("无法启动标注会话。") : err);
		return false;
	}
	lh->setActiveClass(g_labeling.sessionId, 1, nullptr);
	return true;
}

bool ensureSurfaceSession(IPluginPointCloudHost* pch, IPluginDocument* doc, const std::string& meshId, QString* outError)
{
	if (g_surface.sessionId.valid() && g_surface.meshBackendId == meshId)
		return true;
	if (g_surface.sessionId.valid())
		pch->clearMeshSurfaceReconstructSession(doc, g_surface.sessionId);
	g_surface.sessionId = pch->beginMeshSurfaceReconstructSession(doc, meshId);
	g_surface.meshBackendId = meshId;
	if (!g_surface.sessionId.valid())
	{
		setErr(outError, QStringLiteral("无法启动曲面重构会话。"));
		return false;
	}
	return true;
}

bool ensureTubularSession(IPluginPointCloudHost* pch, IPluginDocument* doc, const std::string& meshId, QString* outError)
{
	if (g_tubular.sessionId.valid() && g_tubular.meshBackendId == meshId)
		return true;
	if (g_tubular.sessionId.valid())
		pch->clearTubularGrindingSession(doc, g_tubular.sessionId);
	g_tubular.sessionId = pch->beginTubularGrindingSession(doc, meshId);
	g_tubular.meshBackendId = meshId;
	if (!g_tubular.sessionId.valid())
	{
		setErr(outError, QStringLiteral("无法启动特征构建会话。"));
		return false;
	}
	return true;
}

bool runSurfaceStage(PluginHostContext& host, nlohmann::json& args, PluginMeshSurfaceReconstructStage stage,
					 QString* outError)
{
	auto* pch = requirePc(host, outError);
	auto* doc = requireDoc(host, outError);
	if (!pch || !doc)
		return false;
	const std::string id =
		resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Mesh, QStringLiteral("选择网格"));
	if (id.empty())
		return false;
	if (!ensureSurfaceSession(pch, doc, id, outError))
		return false;
	PluginMeshSurfaceReconstructParams params;
	QEventLoop loop;
	bool ok = false;
	QString err;
	pch->runMeshSurfaceReconstructStage(
		doc, g_surface.sessionId, stage, params,
		[&](bool success, const QString& error, const PluginMeshSurfaceReconstructReport&)
		{
			ok = success;
			err = error;
			loop.quit();
		});
	loop.exec();
	if (!ok)
		setErr(outError, err.isEmpty() ? QStringLiteral("曲面重构阶段失败。") : err);
	return ok;
}

bool runTubularStage(PluginHostContext& host, nlohmann::json& args, PluginTubularGrindingStage stage, QString* outError)
{
	auto* pch = requirePc(host, outError);
	auto* doc = requireDoc(host, outError);
	if (!pch || !doc)
		return false;
	const std::string id =
		resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Mesh, QStringLiteral("选择网格"));
	if (id.empty())
		return false;
	if (!ensureTubularSession(pch, doc, id, outError))
		return false;
	PluginTubularGrindingParams params;
	QEventLoop loop;
	bool ok = false;
	QString err;
	pch->runTubularGrindingStage(doc, g_tubular.sessionId, stage, params,
								 [&](bool success, const QString& error, const PluginTubularGrindingReport&)
								 {
									 ok = success;
									 err = error;
									 loop.quit();
								 });
	loop.exec();
	if (!ok)
		setErr(outError, err.isEmpty() ? QStringLiteral("特征构建阶段失败。") : err);
	return ok;
}

} // namespace

bool tryExecute(PluginHostContext& host, const std::string& api, const nlohmann::json& argsIn, QString* outError,
				bool allowModalDialogs, QString* outSummary)
{
	const bool prevModal = g_allowModalDialogs;
	g_allowModalDialogs = allowModalDialogs;
	struct RestoreModal
	{
		bool prev;
		~RestoreModal() { g_allowModalDialogs = prev; }
	} restore{prevModal};

	nlohmann::json args = argsIn.is_object() ? argsIn : nlohmann::json::object();
	if (outError)
		outError->clear();
	if (outSummary)
		outSummary->clear();

	auto doneFail = [&](const QString& msg) -> bool
	{
		setErr(outError, msg);
		return true; // 已识别
	};
	auto doneOk = []() -> bool { return true; };
	// 约定：返回 true=已处理（看 outError 是否空判断成败）；false=未识别

	if (api == "importFileIntoActiveDocument")
	{
		std::string path = argString(args, "path");
		bool isPc = true;
		if (args.contains("is_point_cloud"))
			isPc = args["is_point_cloud"].get<bool>();
		else if (args.contains("isPointCloud"))
			isPc = args["isPointCloud"].get<bool>();
		if (path.empty())
		{
			if (!g_allowModalDialogs)
				return doneFail(QStringLiteral("缺少 path（应由确认面板提供）。"));
			QString picked;
			const QString filter = isPc ? QStringLiteral("Point Cloud (*.ply *.xyz *.las *.laz);;All (*.*)")
										: QStringLiteral("Mesh (*.obj *.stl *.ply *.off *.step *.stp *.dxf);;All (*.*)");
			if (!AiAgentPickDialog::pickOpenFilePath(dialogParent(host), QStringLiteral("选择导入文件"), filter, &picked))
				return doneFail(QStringLiteral("已取消选择导入文件。"));
			path = picked.toStdString();
		}
		std::string err;
		if (host.importFileIntoActiveDocument(path, isPc, &err).empty())
			return doneFail(QString::fromStdString(err.empty() ? "导入失败。" : err));
		return doneOk();
	}

	if (api == "downsamplePointCloudVoxel" || api == "downsamplePointCloudRandom" || api == "cropPointCloudByBox" ||
		api == "cropPointCloudBySphere" || api == "cropPointCloudByPolyline" || api == "removePointCloudOutliers" ||
		api == "smoothPointCloudBilateral" || api == "estimatePointCloudNormalsPca" ||
		api == "orientPointCloudNormalsMst" || api == "reconstructMeshPoissonAuto" ||
		api == "reconstructMeshScaleSpace")
	{
		auto* pch = requirePc(host, outError);
		auto* doc = requireDoc(host, outError);
		if (!pch || !doc)
			return true;
		const std::string id = resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::PointCloud,
												QStringLiteral("选择点云"));
		if (id.empty())
			return true;

		if (api == "downsamplePointCloudVoxel")
		{
			PluginPointCloudDownsampleVoxelParams p;
			if (args.contains("voxel_mm"))
				p.voxelSizeMm = args["voxel_mm"].get<double>();
			waitPcJob([&](auto cb) { pch->downsamplePointCloudVoxel(doc, id, p, cb); }, outError);
			return true;
		}
		if (api == "downsamplePointCloudRandom")
		{
			PluginPointCloudDownsampleRandomParams p;
			if (args.contains("ratio"))
				p.retainedFraction = args["ratio"].get<double>();
			waitPcJob([&](auto cb) { pch->downsamplePointCloudRandom(doc, id, p, cb); }, outError);
			return true;
		}
		if (api == "cropPointCloudByBox")
		{
			PluginPointCloudInfo info;
			if (!doc->queryPointCloudInfo(id, info) || !info.bounds.valid)
				return doneFail(QStringLiteral("无有效包围盒，无法裁剪。"));
			PluginPointCloudCropBoxParams p;
			p.box = info.bounds;
			waitPcJob([&](auto cb) { pch->cropPointCloudByBox(doc, id, p, cb); }, outError);
			return true;
		}
		if (api == "cropPointCloudBySphere")
		{
			PluginPointCloudMeasure measure;
			if (!doc->measurePointCloud(id, measure))
				return doneFail(QStringLiteral("无法度量点云质心。"));
			PluginPointCloudCropSphereParams p;
			p.centerMm = measure.centroidMm;
			p.radiusMm = args.value("radius_mm", 50.0);
			waitPcJob([&](auto cb) { pch->cropPointCloudBySphere(doc, id, p, cb); }, outError);
			return true;
		}
		if (api == "cropPointCloudByPolyline")
		{
			QEventLoop loop;
			bool pickOk = false;
			QString pickErr;
			PluginPointCloudCropPolylineParams cropParams;
			pch->pickPolylineFromViewport(
				doc, [&](bool ok, const QString& error, const PluginPointCloudPolylinePickResult& pick)
				{
					pickOk = ok;
					pickErr = error;
					if (ok)
					{
						cropParams.polylineScreenXy = pick.polylineScreenXy;
						for (int i = 0; i < 16; ++i)
							cropParams.mvpMatrix[i] = pick.mvpMatrix[i];
						cropParams.viewportWidth = pick.viewportWidth;
						cropParams.viewportHeight = pick.viewportHeight;
					}
					loop.quit();
				});
			loop.exec();
			if (!pickOk)
				return doneFail(pickErr.isEmpty() ? QStringLiteral("多边形拾取取消或失败。") : pickErr);
			waitPcJob([&](auto cb) { pch->cropPointCloudByPolyline(doc, id, cropParams, cb); }, outError);
			return true;
		}
		if (api == "removePointCloudOutliers")
		{
			PluginPointCloudOutlierParams p;
			waitPcJob([&](auto cb) { pch->removePointCloudOutliers(doc, id, p, cb); }, outError);
			return true;
		}
		if (api == "smoothPointCloudBilateral")
		{
			waitPcJob([&](auto cb) { pch->smoothPointCloudBilateral(doc, id, cb); }, outError);
			return true;
		}
		if (api == "estimatePointCloudNormalsPca")
		{
			PluginPointCloudNormalsParams p;
			waitPcJob([&](auto cb) { pch->estimatePointCloudNormalsPca(doc, id, p, cb); }, outError);
			return true;
		}
		if (api == "orientPointCloudNormalsMst")
		{
			PluginPointCloudNormalsParams p;
			waitPcJob([&](auto cb) { pch->orientPointCloudNormalsMst(doc, id, p, cb); }, outError);
			return true;
		}
		if (api == "reconstructMeshPoissonAuto")
		{
			PluginPointCloudReconstructPoissonAutoParams p;
			waitPcJob([&](auto cb) { pch->reconstructMeshPoissonAuto(doc, id, p, cb); }, outError);
			return true;
		}
		if (api == "reconstructMeshScaleSpace")
		{
			PluginPointCloudReconstructScaleSpaceParams p;
			waitPcJob([&](auto cb) { pch->reconstructMeshScaleSpace(doc, id, p, cb); }, outError);
			return true;
		}
	}

	if (api == "rigidRegisterPointCloudsIcp" || api == "nonRigidRegisterSpare")
	{
		auto* pch = requirePc(host, outError);
		auto* doc = requireDoc(host, outError);
		if (!pch || !doc)
			return true;
		std::string sourceId;
		std::string targetId;
		const QString title = api == "rigidRegisterPointCloudsIcp" ? QStringLiteral("ICP 配准：选择源与目标")
																   : QStringLiteral("SPARE 配准：选择源与目标");
		if (!resolveSourceTarget(host, args, outError, AiAgentPickDialog::BackendKindFilter::PointCloudOrMesh, title,
								 &sourceId, &targetId))
			return true;
		if (api == "rigidRegisterPointCloudsIcp")
		{
			PluginPointCloudIcpParams p;
			p.targetBackendIdUtf8 = targetId;
			waitPcJob([&](auto cb) { pch->rigidRegisterPointCloudsIcp(doc, sourceId, p, cb); }, outError);
		}
		else
		{
			PluginPointCloudSpareParams p;
			p.targetBackendIdUtf8 = targetId;
			waitPcJob([&](auto cb) { pch->nonRigidRegisterSpare(doc, sourceId, p, cb); }, outError);
		}
		return true;
	}

	if (api == "exportMeshToPly")
	{
		auto* doc = requireDoc(host, outError);
		if (!doc)
			return true;
		const std::string id =
			resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Mesh, QStringLiteral("选择网格"));
		if (id.empty())
			return true;
		std::string path = argString(args, "path");
		if (path.empty())
		{
			if (!g_allowModalDialogs)
				return doneFail(QStringLiteral("缺少 path（应由确认面板提供）。"));
			QString picked;
			if (!AiAgentPickDialog::pickSaveFilePath(dialogParent(host), QStringLiteral("导出 PLY"),
													 QStringLiteral("PLY (*.ply)"), QStringLiteral("export.ply"), &picked))
				return doneFail(QStringLiteral("已取消导出路径选择。"));
			path = picked.toStdString();
		}
		std::string err;
		if (!doc->exportMeshToPly(id, path, &err))
			return doneFail(QString::fromStdString(err.empty() ? "导出失败。" : err));
		return doneOk();
	}

	if (api == "registerScanToCadTemplateCoarse" || api == "registerScanToCadTemplateFine" ||
		api == "updateTemplateBrepFromAlignedScan")
	{
		auto* pch = requirePc(host, outError);
		auto* doc = requireDoc(host, outError);
		if (!pch || !doc)
			return true;
		std::string scanId = argString(args, "backend_id");
		if (scanId.empty())
			scanId = host.selectedBackendId().toStdString();
		std::string templ = argString(args, "template_backend_id");
		if (scanId.empty() || templ.empty())
		{
			if (!g_allowModalDialogs)
				return doneFail(QStringLiteral("缺少扫描/模板对象（应由确认面板提供）。"));
			auto scans = AiAgentPickDialog::listBackends(doc, AiAgentPickDialog::BackendKindFilter::PointCloudOrMesh);
			auto templates = AiAgentPickDialog::listBackends(doc, AiAgentPickDialog::BackendKindFilter::Brep);
			if (scans.empty() || templates.empty())
				return doneFail(QStringLiteral("需要扫描对象与 B-rep 模板，请先导入。"));
			QString s;
			QString t;
			// 复用双选：源=扫描，目标=模板
			std::vector<AiAgentPickDialog::BackendEntry> both = scans;
			both.insert(both.end(), templates.begin(), templates.end());
			if (!AiAgentPickDialog::pickSourceAndTarget(dialogParent(host), both, QStringLiteral("选择扫描与 CAD 模板"),
													   &s, &t))
				return doneFail(QStringLiteral("已取消选择扫描/模板。"));
			scanId = s.toStdString();
			templ = t.toStdString();
		}
		PluginPointCloudTemplateBrepUpdateParams p;
		p.templateBrepBackendIdUtf8 = templ;
		if (api == "updateTemplateBrepFromAlignedScan")
		{
			QEventLoop loop;
			bool ok = false;
			QString err;
			pch->updateTemplateBrepFromAlignedScan(
				doc, scanId, p, [&](bool success, const QString& error, const PluginPointCloudTemplateBrepUpdateResult&)
				{
					ok = success;
					err = error;
					loop.quit();
				});
			loop.exec();
			if (!ok)
				setErr(outError, err.isEmpty() ? QStringLiteral("面重构失败。") : err);
			return true;
		}
		p.registrationStage = api == "registerScanToCadTemplateCoarse"
								  ? PluginPointCloudTemplateBrepRegistrationStage::CoarseOnly
								  : PluginPointCloudTemplateBrepRegistrationStage::FineOnly;
		QEventLoop loop;
		bool ok = false;
		QString err;
		pch->registerScanToCadTemplate(
			doc, scanId, p, [&](bool success, const QString& error, const PluginPointCloudTemplateBrepRegisterResult&)
			{
				ok = success;
				err = error;
				loop.quit();
			});
		loop.exec();
		if (!ok)
			setErr(outError, err.isEmpty() ? QStringLiteral("模板匹配失败。") : err);
		return true;
	}

	if (api == "simplifyMesh" || api == "smoothMeshLaplacian" || api == "smoothMeshTaubin" || api == "repairMesh" ||
		api == "remeshMeshIsotropic")
	{
		auto* pch = requirePc(host, outError);
		auto* doc = requireDoc(host, outError);
		if (!pch || !doc)
			return true;
		const std::string id =
			resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Mesh, QStringLiteral("选择网格"));
		if (id.empty())
			return true;
		if (api == "simplifyMesh")
		{
			PluginMeshSimplifyParams p;
			waitMeshJob([&](auto cb) { pch->simplifyMesh(doc, id, p, cb); }, outError);
		}
		else if (api == "repairMesh")
		{
			PluginMeshRepairParams p;
			waitMeshJob([&](auto cb) { pch->repairMesh(doc, id, p, cb); }, outError);
		}
		else if (api == "remeshMeshIsotropic")
		{
			PluginMeshRemeshParams p;
			waitMeshJob([&](auto cb) { pch->remeshMeshIsotropic(doc, id, p, cb); }, outError);
		}
		else
		{
			PluginMeshSmoothParams p;
			p.useTaubinSmooth = (api == "smoothMeshTaubin");
			waitMeshJob([&](auto cb) { pch->smoothMesh(doc, id, p, cb); }, outError);
		}
		return true;
	}

	if (api == "runMeshSurfaceReconstructPreprocess")
	{
		runSurfaceStage(host, args, PluginMeshSurfaceReconstructStage::Preprocess, outError);
		return true;
	}
	if (api == "runMeshSurfaceReconstructPartition")
	{
		runSurfaceStage(host, args, PluginMeshSurfaceReconstructStage::Partition, outError);
		return true;
	}
	if (api == "runMeshSurfaceReconstructSample")
	{
		runSurfaceStage(host, args, PluginMeshSurfaceReconstructStage::Sample, outError);
		return true;
	}
	if (api == "runMeshSurfaceReconstructFit")
	{
		runSurfaceStage(host, args, PluginMeshSurfaceReconstructStage::Fit, outError);
		return true;
	}
	if (api == "runMeshSurfaceReconstructBoundaryBlend")
	{
		runSurfaceStage(host, args, PluginMeshSurfaceReconstructStage::BoundaryBlend, outError);
		return true;
	}
	if (api == "runMeshSurfaceReconstructJunctionBlend")
	{
		runSurfaceStage(host, args, PluginMeshSurfaceReconstructStage::JunctionBlend, outError);
		return true;
	}
	if (api == "runMeshSurfaceReconstructFair")
	{
		runSurfaceStage(host, args, PluginMeshSurfaceReconstructStage::Fair, outError);
		return true;
	}
	if (api == "runMeshSurfaceReconstructAssemble")
	{
		runSurfaceStage(host, args, PluginMeshSurfaceReconstructStage::Assemble, outError);
		return true;
	}
	if (api == "reconstructSurfaceFromMesh")
	{
		auto* pch = requirePc(host, outError);
		auto* doc = requireDoc(host, outError);
		if (!pch || !doc)
			return true;
		const std::string id =
			resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Mesh, QStringLiteral("选择网格"));
		if (id.empty())
			return true;
		PluginMeshSurfaceReconstructParams p;
		QEventLoop loop;
		bool ok = false;
		QString err;
		pch->reconstructSurfaceFromMesh(doc, id, p,
										[&](bool success, const QString& error, const PluginMeshSurfaceReconstructReport&)
										{
											ok = success;
											err = error;
											loop.quit();
										});
		loop.exec();
		if (!ok)
			setErr(outError, err.isEmpty() ? QStringLiteral("全流程曲面重构失败。") : err);
		return true;
	}
	if (api == "clearMeshSurfaceReconstructSession")
	{
		auto* pch = requirePc(host, outError);
		auto* doc = requireDoc(host, outError);
		if (!pch || !doc)
			return true;
		if (g_surface.sessionId.valid())
			pch->clearMeshSurfaceReconstructSession(doc, g_surface.sessionId);
		g_surface = {};
		return true;
	}

	if (api == "runTubularGrindingCenterline")
	{
		runTubularStage(host, args, PluginTubularGrindingStage::Centerline, outError);
		return true;
	}
	if (api == "runTubularGrindingTemplatePoints")
	{
		runTubularStage(host, args, PluginTubularGrindingStage::TemplatePoints, outError);
		return true;
	}
	if (api == "runTubularGrindingProject")
	{
		runTubularStage(host, args, PluginTubularGrindingStage::Project, outError);
		return true;
	}
	if (api == "runTubularGrindingFpfhRegionPartition")
	{
		runTubularStage(host, args, PluginTubularGrindingStage::FpfhRegionPartition, outError);
		return true;
	}
	if (api == "clearTubularGrindingSession")
	{
		auto* pch = requirePc(host, outError);
		auto* doc = requireDoc(host, outError);
		if (!pch || !doc)
			return true;
		if (g_tubular.sessionId.valid())
			pch->clearTubularGrindingSession(doc, g_tubular.sessionId);
		g_tubular = {};
		return true;
	}

	if (api == "discretizeBackendToMesh")
	{
		IPluginGeometryHost* geo = host.geometryHost();
		auto* doc = requireDoc(host, outError);
		if (!geo || !doc)
		{
			if (!geo)
				setErr(outError, QStringLiteral("几何宿主不可用。"));
			return true;
		}
		std::string path = argString(args, "step_path");
		if (path.empty())
		{
			if (!g_allowModalDialogs)
			{
				const std::string id = resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Brep,
														QStringLiteral("选择 B-rep"));
				if (id.empty())
					return true;
				path = id;
			}
			else
			{
				QString picked;
				if (!AiAgentPickDialog::pickOpenFilePath(dialogParent(host), QStringLiteral("选择 STEP"),
														 QStringLiteral("STEP (*.step *.stp);;All (*.*)"), &picked))
				{
					const std::string id = resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Brep,
															QStringLiteral("选择 B-rep"));
					if (id.empty())
						return true;
					path = id; // backend path resolved inside host by id-as-path convention used by dock
				}
				else
					path = picked.toStdString();
			}
		}
		PluginMeshDiscretizeParams p;
		PluginMeshCreateOptions opt;
		opt.displayName = QStringLiteral("AiDiscretizedMesh");
		opt.selectInTree = true;
		waitGeomJob([&](PluginGeometryFinishedFn cb) { geo->discretizeBackendToMesh(doc, path, p, opt, cb); }, outError);
		return true;
	}
	if (api == "pickStepElementEdge" || api == "pickStepElementFace")
	{
		IPluginGeometryHost* geo = host.geometryHost();
		auto* doc = requireDoc(host, outError);
		if (!geo || !doc)
		{
			if (!geo)
				setErr(outError, QStringLiteral("几何宿主不可用。"));
			return true;
		}
		PluginGeometryElementPickRequest req;
		req.backendIdUtf8 = host.selectedBackendId().toStdString();
		req.kind = api == "pickStepElementEdge" ? PluginGeometryElementKind::Edge : PluginGeometryElementKind::Face;
		QEventLoop loop;
		bool ok = false;
		QString err;
		PluginGeometryStepRef ref;
		geo->pickStepElementFromViewport(doc, req,
										 [&](bool success, const QString& error, const PluginGeometryStepRef& r)
										 {
											 ok = success;
											 err = error;
											 ref = r;
											 loop.quit();
										 });
		loop.exec();
		if (!ok)
			return doneFail(err.isEmpty() ? QStringLiteral("拾取取消或失败。") : err);
		if (api == "pickStepElementEdge")
			g_lastEdge = ref;
		else
		{
			g_lastFace = ref;
			if (g_lastFaceA.faceIndex >= 0)
				g_lastFaceB = ref;
			else
				g_lastFaceA = ref;
		}
		return true;
	}
	if (api == "intersectEdgeFace" || api == "intersectFaces")
	{
		IPluginGeometryHost* geo = host.geometryHost();
		auto* doc = requireDoc(host, outError);
		if (!geo || !doc)
		{
			if (!geo)
				setErr(outError, QStringLiteral("几何宿主不可用。"));
			return true;
		}
		PluginGeometryIntersectionParams p;
		if (api == "intersectEdgeFace")
		{
			if (g_lastEdge.edgeIndex < 0 || g_lastFace.faceIndex < 0)
				return doneFail(QStringLiteral("请先点选边与点选面。"));
		waitGeomJob(
			[&](PluginGeometryFinishedFn cb)
			{
				geo->intersectEdgeFace(
					doc, g_lastEdge, g_lastFace, p,
					[&](bool success, const QString& error, const PluginGeometryJobResult& result)
					{
						if (success && !result.polylines.empty())
							g_lastPolylineXyz = result.polylines.front();
						cb(success, error, result);
					});
			},
			outError);
		}
		else
		{
			if (g_lastFaceA.faceIndex < 0 || g_lastFaceB.faceIndex < 0)
				return doneFail(QStringLiteral("请先点选 F1 与 F2（点选面两次）。"));
			waitGeomJob(
				[&](PluginGeometryFinishedFn cb)
				{
					geo->intersectFaces(
						doc, g_lastFaceA, g_lastFaceB, p,
						[&](bool success, const QString& error, const PluginGeometryJobResult& result)
						{
							if (success && !result.polylines.empty())
								g_lastPolylineXyz = result.polylines.front();
							cb(success, error, result);
						});
				},
				outError);
		}
		return true;
	}
	if (api == "discretizeWireToTubeMesh" || api == "discretizeWireToRibbonMesh")
	{
		IPluginGeometryHost* geo = host.geometryHost();
		auto* doc = requireDoc(host, outError);
		if (!geo || !doc)
		{
			if (!geo)
				setErr(outError, QStringLiteral("几何宿主不可用。"));
			return true;
		}
		if (g_lastPolylineXyz.size() < 6)
			return doneFail(QStringLiteral("请先执行线面/面面求交以获得折线，再生成管状/带状网格。"));
		PluginMeshDiscretizeParams p;
		PluginMeshCreateOptions opt;
		opt.selectInTree = true;
		waitGeomJob(
			[&](PluginGeometryFinishedFn cb)
			{
				if (api == "discretizeWireToTubeMesh")
					geo->discretizeWireToTubeMesh(doc, g_lastPolylineXyz, p, opt, cb);
				else
					geo->discretizeWireToRibbonMesh(doc, g_lastPolylineXyz, p, opt, cb);
			},
			outError);
		return true;
	}

	if (api == "labelingPickClick" || api == "labelingPickBrush" || api == "labelingPickLasso" || api == "labelingErase")
	{
		IPluginLabelingHost* lh = host.labelingHost();
		if (!lh)
			return doneFail(QStringLiteral("标注宿主不可用。"));
		const std::string id = resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::PointCloudOrMesh,
												QStringLiteral("选择标注对象"));
		if (id.empty())
			return true;
		if (!ensureLabelingSession(host, id, outError))
			return true;
		const bool erase = (api == "labelingErase");
		QEventLoop loop;
		bool ok = false;
		QString err;
		PluginLabelingSelectionResult sel;
		auto onPick = [&](bool success, const QString& error, const PluginLabelingSelectionResult& r)
		{
			ok = success;
			err = error;
			sel = r;
			loop.quit();
		};
		PluginLabelingSessionSummary summary;
		lh->getSessionSummary(g_labeling.sessionId, summary, nullptr);
		const bool mesh = summary.geometryKind == PluginLabelingGeometryKind::TriangleMesh;
		if (api == "labelingPickBrush")
		{
			if (mesh)
				lh->brushMeshFaces(g_labeling.sessionId, 16.f, [](const PluginLabelingSelectionResult&) {}, onPick);
			else
				lh->brushStroke(g_labeling.sessionId, 16.f, [](const PluginLabelingSelectionResult&) {}, onPick);
		}
		else if (api == "labelingPickLasso")
			lh->pickPolylineRegion(g_labeling.sessionId, onPick);
		else if (mesh)
			lh->pickMeshFaceOnce(g_labeling.sessionId, onPick);
		else
			lh->pickPointsOnce(g_labeling.sessionId, onPick);
		loop.exec();
		if (!ok)
			return doneFail(err.isEmpty() ? QStringLiteral("标注拾取取消或失败。") : err);
		QString applyErr;
		if (!lh->applyLabels(g_labeling.sessionId, sel, erase ? 0 : 1, erase, &applyErr))
			return doneFail(applyErr);
		lh->syncLabelVisualization(g_labeling.sessionId, nullptr);
		return true;
	}
	if (api == "cancelActiveLabelingPick")
	{
		if (auto* lh = host.labelingHost())
			lh->cancelActiveLabelingPick();
		return true;
	}
	if (api == "labelingUndo" || api == "labelingRedo")
	{
		IPluginLabelingHost* lh = host.labelingHost();
		if (!lh || g_labeling.sessionId == kInvalidLabelingSessionId)
			return doneFail(QStringLiteral("无活动标注会话。"));
		QString err;
		const bool ok =
			api == "labelingUndo" ? lh->undo(g_labeling.sessionId, &err) : lh->redo(g_labeling.sessionId, &err);
		if (!ok)
			return doneFail(err);
		lh->syncLabelVisualization(g_labeling.sessionId, nullptr);
		return true;
	}
	if (api == "pointNetPrelabel")
		return doneFail(QStringLiteral("PointNet 预标注请在标注面板执行（需 PointNet 插件与模型配置）。"));
	if (api == "exportPointNetDataset")
	{
		IPluginLabelingHost* lh = host.labelingHost();
		if (!lh || g_labeling.sessionId == kInvalidLabelingSessionId)
			return doneFail(QStringLiteral("无活动标注会话。"));
		std::string dir = argString(args, "path");
		if (dir.empty())
		{
			if (!g_allowModalDialogs)
				return doneFail(QStringLiteral("缺少导出目录 path（应由确认面板提供）。"));
			QString picked;
			if (!AiAgentPickDialog::pickExistingDirectory(dialogParent(host), QStringLiteral("选择导出目录"), &picked))
				return doneFail(QStringLiteral("已取消导出目录选择。"));
			dir = picked.toStdString();
		}
		PluginLabelingDatasetExportOptions opt;
		opt.sampleNameUtf8 = "sample_ai";
		opt.numClasses = 2;
		PluginLabelingDatasetExportResult result;
		QString err;
		if (!lh->exportPointNetDataset(g_labeling.sessionId, dir, opt, result, &err))
			return doneFail(err);
		return true;
	}

	if (api == "removeSceneObject")
	{
		IPluginDocument* doc = requireDoc(host, outError);
		if (!doc)
			return true;
		const std::string id =
			resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Any, QStringLiteral("选择对象"));
		if (id.empty())
			return true;
		std::string err;
		if (!doc->removeBackendObject(id, &err))
			return doneFail(QString::fromStdString(err));
		return doneOk();
	}
	if (api == "removeAllSceneObjects")
	{
		IPluginDocument* doc = requireDoc(host, outError);
		if (!doc)
			return true;
		const auto ids = doc->backendIds();
		for (const auto& id : ids)
		{
			std::string err;
			if (!doc->removeBackendObject(id, &err))
				return doneFail(QString::fromStdString(err.empty() ? "删除失败" : err));
		}
		return doneOk();
	}
	if (api == "translateSceneObject" || api == "rotateSceneObject")
	{
		IPluginDocument* doc = requireDoc(host, outError);
		if (!doc)
			return true;
		auto* adapter = dynamic_cast<PluginDocumentAdapter*>(doc);
		if (!adapter)
			return doneFail(QStringLiteral("文档适配器不可用。"));
		const std::string id =
			resolveBackendId(host, args, outError, AiAgentPickDialog::BackendKindFilter::Any, QStringLiteral("选择对象"));
		if (id.empty())
			return true;
		PluginDocumentAdapter::WorldPoseMm pose;
		if (!adapter->getWorldPoseMm(id, &pose))
			return doneFail(QStringLiteral("读取位姿失败。"));
		auto num = [&](const char* key) -> double
		{
			if (!args.contains(key))
				return 0.0;
			const auto& v = args[key];
			if (v.is_number())
				return v.get<double>();
			if (v.is_string())
			{
				bool ok = false;
				const double d = QString::fromStdString(v.get<std::string>()).toDouble(&ok);
				return ok ? d : 0.0;
			}
			return 0.0;
		};
		if (api == "translateSceneObject")
		{
			pose.xMm += num("dx_mm");
			pose.yMm += num("dy_mm");
			pose.zMm += num("dz_mm");
		}
		else
		{
			pose.rxDeg += num("rx_deg");
			pose.ryDeg += num("ry_deg");
			pose.rzDeg += num("rz_deg");
		}
		std::string err;
		if (!adapter->applyWorldPoseMm(id, pose, &err))
			return doneFail(QString::fromStdString(err));
		return doneOk();
	}

	if (api == "applyProcessFlowGraph" || api == "patchProcessFlowGraph" || api == "runProcessFlowSimulation" ||
		api == "compareProcessFlowPolicies")
	{
		IProcessFlowAiBridge* bridge = host.processFlowAiBridge();
		if (!bridge)
			return doneFail(QStringLiteral("工艺流程插件未加载或未注册 AI 桥接。"));

		if (api == "applyProcessFlowGraph")
		{
			QJsonObject flow;
			if (args.contains("flow") && args["flow"].is_object())
			{
				flow = QJsonDocument::fromJson(QByteArray::fromStdString(args["flow"].dump())).object();
			}
			else
			{
				std::string raw = argString(args, "flow_json");
				if (raw.empty())
					return doneFail(QStringLiteral("缺少 flow_json。"));
				// 规则/LLM 可能再包一层引号
				if (!raw.empty() && raw.front() == '"' && raw.back() == '"')
					raw = nlohmann::json::parse(raw).get<std::string>();
				QJsonParseError pe;
				const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(raw), &pe);
				if (pe.error != QJsonParseError::NoError || !doc.isObject())
					return doneFail(QStringLiteral("flow_json 不是合法 JSON 对象。"));
				flow = doc.object();
			}
			bool autoLayout = true;
			if (args.contains("auto_layout"))
			{
				if (args["auto_layout"].is_boolean())
					autoLayout = args["auto_layout"].get<bool>();
				else if (args["auto_layout"].is_string())
					autoLayout = QString::fromStdString(args["auto_layout"].get<std::string>()) != QStringLiteral("false");
			}
			QString err;
			if (!bridge->applyFlowJson(flow, autoLayout, &err))
				return doneFail(err.isEmpty() ? QStringLiteral("写入流程图失败。") : err);
			if (outSummary)
				*outSummary = QStringLiteral("已应用工艺流程图。");
			return doneOk();
		}

		if (api == "patchProcessFlowGraph")
		{
			QJsonArray ops;
			if (args.contains("ops") && args["ops"].is_array())
			{
				ops = QJsonDocument::fromJson(QByteArray::fromStdString(args["ops"].dump())).array();
			}
			else
			{
				std::string raw = argString(args, "ops_json");
				if (raw.empty())
					raw = argString(args, "ops");
				if (raw.empty())
					return doneFail(QStringLiteral("缺少 ops / ops_json。"));
				if (!raw.empty() && raw.front() == '"' && raw.back() == '"')
					raw = nlohmann::json::parse(raw).get<std::string>();
				QJsonParseError pe;
				const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(raw), &pe);
				if (pe.error != QJsonParseError::NoError || !doc.isArray())
					return doneFail(QStringLiteral("ops_json 不是合法 JSON 数组。"));
				ops = doc.array();
			}
			QString err;
			if (!bridge->applyFlowPatch(ops, &err))
				return doneFail(err.isEmpty() ? QStringLiteral("增量改图失败。") : err);
			if (outSummary)
				*outSummary = QStringLiteral("已应用工艺流程补丁（%1 条）。").arg(ops.size());
			return doneOk();
		}

		QJsonObject cfg;
		if (args.contains("horizonSec"))
		{
			if (args["horizonSec"].is_number())
				cfg.insert(QStringLiteral("horizonSec"), args["horizonSec"].get<double>());
			else if (args["horizonSec"].is_string())
				cfg.insert(QStringLiteral("horizonSec"), QString::fromStdString(args["horizonSec"].get<std::string>()).toDouble());
		}
		if (args.contains("policy") && args["policy"].is_string())
			cfg.insert(QStringLiteral("policy"), QString::fromStdString(args["policy"].get<std::string>()));

		if (api == "runProcessFlowSimulation")
		{
			QJsonObject stats;
			QString err;
			if (!bridge->runSimSync(cfg, &stats, &err))
				return doneFail(err.isEmpty() ? QStringLiteral("仿真失败。") : err);
			if (outSummary)
			{
				*outSummary = QStringLiteral("仿真完成：完成%1 报废%2 Makespan=%3s 吞吐=%4/h 瓶颈=%5")
								  .arg(stats.value(QStringLiteral("completedJobs")).toInt())
								  .arg(stats.value(QStringLiteral("scrappedJobs")).toInt())
								  .arg(stats.value(QStringLiteral("makespan")).toDouble(), 0, 'f', 1)
								  .arg(stats.value(QStringLiteral("throughputPerHour")).toDouble(), 0, 'f', 2)
								  .arg(stats.value(QStringLiteral("bottleneckTitle")).toString());
			}
			return doneOk();
		}

		QJsonArray rows;
		QString err;
		if (!bridge->compareSync(cfg, &rows, &err))
			return doneFail(err.isEmpty() ? QStringLiteral("策略对比失败。") : err);
		if (outSummary)
		{
			QStringList lines;
			for (const QJsonValue& v : rows)
			{
				const QJsonObject o = v.toObject();
				lines << QStringLiteral("%1: 完成%2 Makespan=%3 吞吐=%4")
							 .arg(o.value(QStringLiteral("policy")).toString())
							 .arg(o.value(QStringLiteral("completed")).toInt())
							 .arg(o.value(QStringLiteral("makespan")).toDouble(), 0, 'f', 1)
							 .arg(o.value(QStringLiteral("throughput")).toDouble(), 0, 'f', 2);
			}
			*outSummary = QStringLiteral("策略对比：\n%1").arg(lines.join(QLatin1Char('\n')));
		}
		return doneOk();
	}

	return false;
}

AiToolResult execute(PluginHostContext& host, const std::string& api, const nlohmann::json& args, bool allowModalDialogs)
{
	QString err;
	QString summary;
	AiToolResult r;
	r.handled = tryExecute(host, api, args, &err, allowModalDialogs, &summary);
	if (!r.handled)
		return r;
	r.ok = err.isEmpty();
	r.error = err;
	if (r.ok)
		r.summary = summary.isEmpty() ? QStringLiteral("已执行") : summary;
	return r;
}
} // namespace AiHostButtonApiDispatch
