/// @file ProjectPackageIo.cpp
/// @brief ProjectPackageIo 实现

#include "ProjectPackageIo.h"

#include "AnnotationProjectIo.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowSolve.h"
#include "BackendProjectObjectIo.h"
#include "BrepBackendData.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "IRobotService.h"
#include "IRobotUrdfImportContext.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"
#include "RobotProjectKinematicsRestore.h"
#include "RobotSceneKinematics.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonValue>
#include <chrono>
#include <fstream>

#include <osg/Vec3f>

namespace cloudsim::host
{
using ::OsgWidget;

namespace
{
bool isReloadablePointCloudSourcePath(const QString& path)
{
	if (path.isEmpty() || path.startsWith(QStringLiteral("plugin://")))
	{
		return false;
	}
	return QFileInfo::exists(path);
}

bool ensurePointCloudGeometryForSave(DocumentHost& host, PointCloudBackendData& pc, const std::string& backendId,
									 OsgWidget* osg, QStringList& warnings)
{
	if (!pc.pointPositionsXyz().empty())
	{
		return true;
	}
	const QString idQs = QString::fromStdString(backendId);
	if (osg)
	{
		QString sceneErr;
		if (osg->capturePointCloudBackendFromScene(backendId, pc, &sceneErr))
		{
			return true;
		}
		if (!sceneErr.isEmpty())
		{
			warnings.append(QStringLiteral("Save: scene resync for %1: %2").arg(idQs, sceneErr));
		}
		QString stagingErr;
		if (osg->captureImportedPointCloudBackend(pc, &stagingErr))
		{
			return true;
		}
		if (!stagingErr.isEmpty())
		{
			warnings.append(QStringLiteral("Save: staging resync for %1: %2").arg(idQs, stagingErr));
		}
	}
	const QString srcPath = host.backendSourcePath().value(idQs);
	if (isReloadablePointCloudSourcePath(srcPath))
	{
		const QByteArray enc = QFile::encodeName(srcPath);
		const std::string nativePath(enc.constData(), static_cast<std::size_t>(enc.size()));
		std::string loadErr;
		if (pc.loadFromFile(nativePath, &loadErr))
		{
			return true;
		}
		warnings.append(QStringLiteral("Save: reload %1 from source failed: %2")
							.arg(idQs, loadErr.empty() ? QStringLiteral("unknown") : QString::fromStdString(loadErr)));
	}
	return !pc.pointPositionsXyz().empty();
}

} // namespace

FollowSolveContext followSolveContextFromDto(const core::FollowSolveContextDto* dto)
{
	FollowSolveContext ctx;
	if (!dto)
	{
		return ctx;
	}
	const core::FollowSolveContextDto captured = *dto;
	ctx.skipAll = [captured]() { return captured.skipAll; };
	ctx.fillGizmoSelectedId = [captured](std::string& outSelectedId) -> bool
	{
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

	// 用于 BREP/STEP 源文件去重：同一个原始 STEP 文件只拷贝一次
	QMap<QString, QString> stepSourceCache; // srcPath -> relPath
	out.root.insert(QStringLiteral("componentsSchemaVersion"), 1);
	out.root.insert(QStringLiteral("language"), languageCode);

	QJsonArray objects;
	const auto dataList = host.backend().listData();

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
			if (!ensurePointCloudGeometryForSave(host, *pc, id, osg, out.warnings))
			{
				out.abortMessage =
					QStringLiteral(
						"Point cloud '%1' has no coordinates; cannot save. Re-import or ensure the object is visible.")
						.arg(idQs);
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
			if (const auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(data))
			{
				const QString rel = QStringLiteral("objects/%1.ply").arg(idQs);
				const QString abs = QDir(assetOutputDir).filePath(rel);
				QDir().mkpath(QFileInfo(abs).absolutePath());
				std::string writeErr;
				const QByteArray enc = QFile::encodeName(abs);
				const std::string nativePath(enc.constData(), static_cast<std::size_t>(enc.size()));
				if (pc->writePointCloudPlySidecar(nativePath, &writeErr))
				{
					obj.insert(QStringLiteral("assetRelativePath"), rel);
					QJsonObject geom = obj.value(QStringLiteral("geometry")).toObject();
					geom.remove(QStringLiteral("xyzBase64"));
					geom.remove(QStringLiteral("rgbaPerVertexBase64"));
					geom.insert(QStringLiteral("kind"), QStringLiteral("points"));
					geom.insert(QStringLiteral("storage"), QStringLiteral("ply_sidecar"));
					geom.insert(QStringLiteral("plySidecar"), rel);
					geom.insert(QStringLiteral("pointCount"), static_cast<qint64>(pc->geometryElementCount()));
					obj.insert(QStringLiteral("geometry"), geom);
				}
				else
				{
					out.abortMessage = QStringLiteral("Point cloud PLY write failed for %1: %2")
										   .arg(idQs, QString::fromStdString(writeErr));
					return out;
				}
			}
			else if (const auto brep = std::dynamic_pointer_cast<BrepBackendData>(data))
			{
				// 优化：同一个原始 STEP 源文件只拷贝一次，后续对象复用同一个 stepSidecar
				const QString srcPath = host.backendSourcePath().value(idQs);
				if (!srcPath.isEmpty() && QFileInfo::exists(srcPath))
				{
					QString rel;
					if (stepSourceCache.contains(srcPath))
					{
						// 复用已分配的 sidecar 路径
						rel = stepSourceCache.value(srcPath);
					}
					else
					{
						const QString originalExt = QFileInfo(srcPath).suffix();
						const QString safeExt = originalExt.isEmpty() ? QStringLiteral("stp") : originalExt;
						rel = QStringLiteral("objects/%1.%2").arg(idQs, safeExt);
						const QString abs = QDir(assetOutputDir).filePath(rel);
						QDir().mkpath(QFileInfo(abs).absolutePath());

						if (QFile::copy(srcPath, abs))
						{
							stepSourceCache.insert(srcPath, rel);
						}
						else
						{
							out.warnings.append(QStringLiteral("Failed to copy STEP source for BREP %1").arg(idQs));
							rel.clear();
						}
					}

					if (!rel.isEmpty())
					{
						obj.insert(QStringLiteral("assetRelativePath"), rel);
						QJsonObject geom = obj.value(QStringLiteral("geometry")).toObject();
						geom.insert(QStringLiteral("kind"), QStringLiteral("brep"));
						geom.insert(QStringLiteral("stepSidecar"), rel);
						obj.insert(QStringLiteral("geometry"), geom);
					}
				}
				else
				{
					out.warnings.append(
						QStringLiteral("BREP object %1 has no valid source path, skipping stepSidecar").arg(idQs));
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

void finalizeProjectLoadFollowAndViewport(DocumentHost& host, const QJsonObject& root, const bool useEdgesRelation,
										  const QVector<ProjectHierarchyEdge>& edges,
										  const core::FollowSolveContextDto* solveCtxDto)
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
	// 打开工程后自适应视口（与工具栏「聚焦」同一接口）
	osg->focusCameraOnAllVisibleBackends();
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

bool applyRestoredJointAnglesToScene(IRobotUrdfImportContext& ctx, const QVector<double>& aggregatedJointAnglesRad,
									 QString* outError)
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
