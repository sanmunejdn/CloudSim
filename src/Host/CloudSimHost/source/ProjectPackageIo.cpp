#include "ProjectPackageIo.h"

#include "AnnotationProjectIo.h"
#include "BackendFollowSolve.h"
#include "BackendProjectObjectIo.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "IRobotService.h"
#include "IRobotUrdfImportContext.h"
#include "OsgWidget.h"
#include "RobotProjectKinematicsRestore.h"

#include "RobotSceneKinematics.h"

#include "../../UI/RobotWidget/inc/IRobotDocumentHost.h"
#include "../../UI/RobotWidget/inc/RobotProjectIoAdapter.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BrepBackendData.h"
#include "PointCloudBackendData.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <QJsonArray>
#include <QJsonValue>

#include <algorithm>
#include <osg/Vec3f>

namespace cloudsim::host
{

using ::OsgWidget;

FollowSolveContext followSolveContextFromDto(const core::FollowSolveContextDto* dto)
{
	FollowSolveContext ctx;
	if (!dto)
	{
		return ctx;
	}
	const core::FollowSolveContextDto captured = *dto;
	ctx.skipAll = [captured]() { return captured.skipAll; };
	ctx.fillGizmoSelectedId = [captured](std::string& outSelectedId) -> bool {
		if (captured.gizmoSelectedBackendId.isEmpty())
		{
			return false;
		}
		outSelectedId = captured.gizmoSelectedBackendId.toStdString();
		return true;
	};
	return ctx;
}

ProjectSaveBuildResult buildProjectSaveRoot(DocumentHost& host, const QString& languageCode,
	const QString& assetOutputDir)
{
	ProjectSaveBuildResult out;
	out.root.insert(QStringLiteral("version"), 4);
	out.root.insert(QStringLiteral("componentsSchemaVersion"), 1);
	out.root.insert(QStringLiteral("language"), languageCode);

	QJsonArray objects;
	const auto dataList = host.backend().listData();
	const int pointCloudObjectCount = static_cast<int>(std::count_if(dataList.begin(), dataList.end(),
		[](const std::shared_ptr<BackendDataBase>& d) {
			return d && std::dynamic_pointer_cast<PointCloudBackendData>(d);
		}));

	OsgWidget* osg = osgWidgetFrom(host);
	for (const auto& data : dataList)
	{
		if (!data)
		{
			continue;
		}
		const std::string id = data->id();
		const QString idQs = QString::fromStdString(id);
		const QString srcPath = host.backendSourcePath().count(idQs) ? host.backendSourcePath()[idQs] : QString();
		const QString sourceType = host.backendSourceType().count(idQs) ? host.backendSourceType()[idQs] : QString();
		const std::vector<std::string> parentIds = host.backend().parentsOf(id);

		if (auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(data))
		{
			// 单点云且后端无坐标：尝试从 OSG 分支回灌再落盘
			if (pc->pointPositionsXyz().empty() && pointCloudObjectCount == 1 && osg)
			{
				QString resyncErr;
				if (!osg->captureImportedPointCloudBackend(*pc, &resyncErr))
				{
					out.warnings.append(QStringLiteral("Save: could not embed point cloud from viewer: %1").arg(resyncErr));
				}
			}
			if (pc->pointPositionsXyz().empty())
			{
				out.abortMessage = QStringLiteral(
					"Point cloud has no coordinates in the backend; cannot save. Re-import the file while it still exists on disk.");
				return out;
			}
		}

		const QString parentId = parentIds.empty() ? QString() : QString::fromStdString(parentIds.front());
		QJsonObject obj = saveProjectObject(host, idQs, srcPath, sourceType, parentId);
		if (obj.isEmpty())
		{
			continue;
		}
		if (!assetOutputDir.isEmpty())
		{
			if (const auto brep = std::dynamic_pointer_cast<BrepBackendData>(data))
			{
				if (brep->hasGeometry())
				{
					const QString rel = QStringLiteral("objects/%1.brep").arg(idQs);
					const QString abs = QDir(assetOutputDir).filePath(rel);
					QDir().mkpath(QFileInfo(abs).absolutePath());
					std::string writeErr;
					const QByteArray enc = QFile::encodeName(abs);
					const std::string nativePath(enc.constData(), static_cast<std::size_t>(enc.size()));
					if (brep->writeBrepFile(nativePath, &writeErr))
					{
						obj.insert(QStringLiteral("assetRelativePath"), rel);
						QJsonObject geom = obj.value(QStringLiteral("geometry")).toObject();
						geom.insert(QStringLiteral("kind"), QStringLiteral("brep"));
						geom.insert(QStringLiteral("brepSidecar"), rel);
						obj.insert(QStringLiteral("geometry"), geom);
					}
					else
					{
						out.warnings.append(QStringLiteral("B-rep sidecar write failed for %1: %2")
							.arg(idQs, QString::fromStdString(writeErr)));
					}
				}
			}
		}
		objects.push_back(obj);
	}
	out.root.insert(QStringLiteral("objects"), objects);

	QJsonArray edgeArray;
	for (const auto& edge : host.backend().listEdges())
	{
		QJsonObject edgeObj;
		edgeObj.insert(QStringLiteral("parentId"), QString::fromStdString(edge.first));
		edgeObj.insert(QStringLiteral("childId"), QString::fromStdString(edge.second));
		edgeArray.push_back(edgeObj);
	}
	out.root.insert(QStringLiteral("edges"), edgeArray);

	if (osg)
	{
		out.root.insert(QStringLiteral("annotations"), buildAnnotationsJsonFromOsg(*osg, out.root));
	}
	else
	{
		out.root.insert(QStringLiteral("annotations"), QJsonArray());
	}
	return out;
}

void applyProjectViewportFromJson(DocumentHost& host, const QJsonObject& root)
{
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		return;
	}
	applyAnnotationsFromProjectJson(*osg, root);
}

void mergeRobotKinematicsIntoProjectRoot(::IRobotDocumentHost* robotDoc, QJsonObject& root,
	const QVector<double>* aggregatedJointAnglesRad)
{
	if (!robotDoc)
	{
		return;
	}
	RobotProjectIo::writeRobotKinematics(root, robotDoc, aggregatedJointAnglesRad);
}

void finalizeProjectLoadFollowAndViewport(DocumentHost& host, const QJsonObject& root, const bool useEdgesRelation,
	const QVector<ProjectHierarchyEdge>& edges, const core::FollowSolveContextDto* solveCtxDto)
{
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		return;
	}
	FollowSolveContext hostCtx = followSolveContextFromDto(solveCtxDto);
	const FollowSolveContext* solveCtx = solveCtxDto ? &hostCtx : nullptr;
	syncOsgBackendParentsFromBackend(host);
	if (useEdgesRelation)
	{
		applyProjectEdgesFollowBindingAndSolve(host, edges, solveCtx);
	}
	applyProjectViewportFromJson(host, root);
	host.invalidateFollowReverseIndex();
	host.requestFollowSolveForced(); // 工程打开首帧须全图求解
	runBackendFollowSolveAndSync(host, *osg, solveCtx);
}

RobotKinematicsRestoreResult restoreRobotKinematicsFromProjectJson(IRobotUrdfImportContext& ctx,
	const QJsonObject& projectRoot)
{
	RobotKinematicsRestoreResult result;
	const QJsonObject legacyKinematics = projectRoot.value(QStringLiteral("robotKinematics")).toObject();
	const QJsonArray instances = projectRoot.value(QStringLiteral("robotKinematicsInstances")).toArray();

	for (const QJsonValue& rv : instances)
	{
		if (!rv.isObject())
		{
			continue;
		}
		QString warn;
		if (restorePerLinkRobotKinematicsFromProjectJson(ctx, rv.toObject(), result.aggregatedJointAnglesRad, &warn))
		{
			++result.restoredInstanceCount;
		}
		else if (!warn.isEmpty())
		{
			result.warnings.append(warn);
		}
	}

	// v4 以前单条 robotKinematics 兼容
	if (result.restoredInstanceCount == 0 && !legacyKinematics.isEmpty())
	{
		QString warn;
		if (restorePerLinkRobotKinematicsFromProjectJson(ctx, legacyKinematics, result.aggregatedJointAnglesRad, &warn))
		{
			result.restoredInstanceCount = 1;
		}
		else if (!warn.isEmpty())
		{
			result.warnings.append(warn);
		}
		else
		{
			result.warnings.append(
				QStringLiteral("robotKinematics: skipped restore (missing URDF, backends, or metadata)."));
		}
	}
	return result;
}

bool applyRestoredJointAnglesToScene(IRobotUrdfImportContext& ctx,
	const QVector<double>& aggregatedJointAnglesRad, QString* outError)
{
	IRobotSimulationDocument* doc = ctx.urdfImportRobotSimulationDocument();
	IRobotBackendPoseSink* poseSink = ctx.urdfImportScenePoseSink();
	if (!doc || !poseSink)
	{
		if (outError)
		{
			*outError = QStringLiteral("no robot document or pose sink");
		}
		return false;
	}
	if (!RobotSceneKinematics::applyJointAnglesFromDocument(doc, poseSink, aggregatedJointAnglesRad))
	{
		if (outError)
		{
			*outError = QStringLiteral("applyJointAnglesFromDocument failed");
		}
		return false;
	}
	return true;
}

bool loadRobotProgramsFromProjectJson(DocumentHost& host, const QJsonObject& projectRoot, QString* outError)
{
	const QJsonArray programs = projectRoot.value(QStringLiteral("robotPrograms")).toArray();
	if (programs.isEmpty())
	{
		return false;
	}
	return host.robot().setRobotProgramsJson(programs, outError);
}

void mergeRobotProgramsIntoProjectRoot(DocumentHost& host, QJsonObject& root)
{
	const QJsonArray programs = host.robot().robotProgramsJson();
	if (programs.isEmpty())
	{
		root.remove(QStringLiteral("robotPrograms"));
	}
	else
	{
		root.insert(QStringLiteral("robotPrograms"), programs);
	}
}

} // namespace cloudsim::host
