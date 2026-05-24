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

#include "../../UI/RobotWidget/inc/IRobotDocumentHost.h"
#include "../../UI/RobotWidget/inc/RobotProjectIoAdapter.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "PointCloudBackendData.h"

#include <QJsonArray>
#include <QJsonValue>

#include <algorithm>
#include <osg/Vec3f>

namespace cloudsim::host
{

using ::OsgWidget;

ProjectSaveBuildResult buildProjectSaveRoot(DocumentHost& host, const QString& languageCode)
{
	ProjectSaveBuildResult out;
	out.root.insert(QStringLiteral("version"), 4);
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

void finalizeProjectLoadFollowAndViewport(DocumentHost& host, OsgWidget& osg, const QJsonObject& root,
	const bool useEdgesRelation, const QVector<ProjectHierarchyEdge>& edges, const FollowSolveContext* solveCtx)
{
	syncOsgBackendParentsFromBackend(host);
	if (useEdgesRelation)
	{
		applyProjectEdgesFollowBindingAndSolve(host, osg, edges, solveCtx);
	}
	applyProjectViewportFromJson(host, root);
	host.invalidateFollowReverseIndex();
	host.requestFollowSolveForced(); // 工程打开首帧须全图求解
	runBackendFollowSolveAndSync(host, osg, solveCtx);
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
