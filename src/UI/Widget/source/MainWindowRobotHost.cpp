/// @file MainWindowRobotHost.cpp
/// @brief 机器人文档宿主适配

#include "MainWindowRobotHost.h"

#include "../RobotWidget/inc/RobotSimulationController.h"
#include "../RobotWidget/inc/RobotSimulationDockWidget.h"
#include "../RobotWidget/inc/RobotAxisControlWidget.h"
#include "../RobotWidget/inc/RobotExternalAxisSettingsWidget.h"
#include "BackendFileImport.h"
#include "BackendDataManager.h"
#include "BackendSceneDocumentFacade.h"
#include "BackendTypeIds.h"
#include "CoreTypes.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceRobotMountComponent.h"
#include "BackendFollowMath.h"
#include "DocumentImportFacade.h"
#include "DocumentPage.h"
#include "FrameBackendData.h"
#include "HierarchyMeshImport.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "IRobotService.h"
#include "JobSystem.h"
#include "MainWindow.h"
#include "MainWindowImportCaptureRenderController.h"
#include "MainWindowSelectionService.h"
#include "OsgWidget.h"
#include "PickTypes.h"
#include "RobotInstructionPropertyDto.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotPlanInstruction.h"
#include "RobotCoordinateFrames.h"
#include "RobotExternalAxes.h"
#include "RobotSimulationMath.h"
#include "KinematicModelIk.h"
#include "KinematicModelRegistry.h"
#include "RobotTeachIk.h"

#include <queue>
#include <unordered_set>
#include "RunInfoPage.h"
#include "io/CustomDeviceRobotMountOps.h"
#include "io/CustomDeviceHostOps.h"

#include <cmath>
#include <limits>
#include "UrdfRobotLoader.h"
#include "WidgetOsgViewHost.h"
#include "WidgetSceneSignalWiring.h"

#include <memory>
#include <algorithm>

#include <Adapters.h>

namespace
{
QString resolveMountFlangeLinkName(DocumentPage* page, const int instIdx, const QString& flangeBackendId,
								   const QString& hint)
{
	if (page)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (page->robotPerLinkKinematicsForInstance(instIdx, pl))
		{
			for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
			{
				if (it.value() == flangeBackendId)
				{
					return it.key();
				}
			}
		}
	}
	const QString trimmed = hint.trimmed();
	if (!trimmed.isEmpty())
	{
		return trimmed;
	}
	if (!page)
	{
		return QString();
	}
	const QString urdfPath = page->robotUrdfAbsolutePathForInstance(instIdx);
	QStringList revoluteChildren;
	if (UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildren, nullptr) &&
		!revoluteChildren.isEmpty())
	{
		return revoluteChildren.back();
	}
	return QString();
}

bool computeMountTcpWorld(IRobotDocumentHost* doc, IRobotOsgViewHost* osg, const int instIdx,
						  const QString& flangeBackendId, const QString& flangeLinkHint,
						  const QVector<double>& jointQ, const RobotCoordinate::RobotCoordinateFrameSet& frames,
						  BackendMat4& outTcpWorld, QString* outResolvedLink = nullptr)
{
	if (!doc || instIdx < 0 || jointQ.isEmpty())
	{
		return false;
	}
	DocumentPage* page = dynamic_cast<DocumentPage*>(doc);
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instIdx);
	if (urdfPath.isEmpty())
	{
		return false;
	}
	const QString resolvedFlange = resolveMountFlangeLinkName(page, instIdx, flangeBackendId, flangeLinkHint);
	if (outResolvedLink)
	{
		*outResolvedLink = resolvedFlange;
	}
	if (resolvedFlange.isEmpty())
	{
		return false;
	}
	QHash<QString, osg::Matrixd> linkWorld;
	QString fkErr;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, jointQ, linkWorld, &fkErr) ||
		!linkWorld.contains(resolvedFlange))
	{
		return false;
	}
	const BackendMat4 T_tool = RobotSimulationMath::toolMat4ForFrames(frames, nullptr);
	const BackendMat4 tcpInBase =
		RobotMatrixOsg::targetInBaseFromFlangeLinkWorld(linkWorld.value(resolvedFlange), T_tool);
	osg::Matrixd robotBaseWorld;
	robotBaseWorld.makeIdentity();
	(void)RobotSimulationMath::robotBaseWorldMatrixForInstance(doc, osg, instIdx, robotBaseWorld);
	const osg::Matrixd tcpRenderWorld =
		RobotMatrixOsg::matrixFromBackendColMajor(tcpInBase) * robotBaseWorld;
	outTcpWorld = RobotMatrixOsg::backendColMajorFromMatrix(tcpRenderWorld);
	return true;
}
// #endregion
} // namespace

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileInfo>
#include <QFormLayout>
#include <QVBoxLayout>

class MainWindowRobotHost::DocumentHost : public IRobotDocumentHost
{
public:
	explicit DocumentHost(DocumentPage* page) : m_page(page) {}

	DocumentPage* page() const { return m_page; }

	QString documentId() const override { return m_page ? m_page->documentId() : QString(); }
	bool robotLocalJointAnglesForSceneRoot(const QString& sceneRootBackendId,
										   QVector<double>& outLocal) const override
	{
		return m_page && m_page->robotLocalJointAnglesForSceneRoot(sceneRootBackendId, outLocal);
	}

	RobotProgramStore& robotProgramStore() override { return m_page->robotProgramStore(); }
	const RobotProgramStore& robotProgramStore() const override { return m_page->robotProgramStore(); }
	IRobotBackendPoseSink* poseSink() override { return m_page->sceneFacade().poseSink(); }
	BackendDataManager& backend() override { return m_page->backend(); }
	cloudsim::core::IDataService& documentData() override { return m_page->documentData(); }
	const cloudsim::core::IDataService& documentData() const override { return m_page->documentData(); }
	std::shared_ptr<BackendDataBase> findObject(const std::string& id) const override { return m_page->findObject(id); }
	std::vector<std::shared_ptr<BackendDataBase>> listObjects() const override { return m_page->listObjects(); }
	BackendDataManager* robotBackendManagerForKinematics() override
	{
		return m_page->robotBackendManagerForKinematics();
	}

	bool hasRobotSimulationContext() const override { return m_page->hasRobotSimulationContext(); }
	bool hasRobotKinematicsBind() const override { return m_page->hasRobotKinematicsBind(); }
	const QString& robotUrdfAbsolutePath() const override { return m_page->robotUrdfAbsolutePath(); }
	const QStringList& robotRevoluteJointNames() const override { return m_page->robotRevoluteJointNames(); }
	const QHash<QString, QString>& robotLinkNameToBackendId() const override
	{
		return m_page->robotLinkNameToBackendId();
	}
	bool hasRobotJointLocalMatrix(const QString& jointName) const override
	{
		return m_page->hasRobotJointLocalMatrix(jointName);
	}
	bool robotJointWorldMatrix(const QString& jointName, cloudsim::core::Mat4& outWorld) const override
	{
		return m_page->robotJointWorldMatrix(jointName, outWorld);
	}
	bool applyRobotJointLocalMatrix(const QString& jointName, const cloudsim::core::Mat4& localColumnMajor) override
	{
		return m_page->applyRobotJointLocalMatrix(jointName, localColumnMajor);
	}
	void applyRobotJointLocalMatrices(const QHash<QString, cloudsim::core::Mat4>& localByPrefixedJointKey) override
	{
		m_page->applyRobotJointLocalMatrices(localByPrefixedJointKey);
	}
	QHash<QString, cloudsim::core::Mat4> robotFkMeshWorldT0() const override { return m_page->robotFkMeshWorldT0(); }
	QHash<QString, cloudsim::core::Mat4> robotOuterWorldAtBind() const override
	{
		return m_page->robotOuterWorldAtBind();
	}
	bool robotUrdfMeshVerticesInLinkFrame() const override { return m_page->robotUrdfMeshVerticesInLinkFrame(); }
	int robotKinematicInstanceCount() const override { return m_page->robotKinematicInstanceCount(); }
	QString robotUrdfAbsolutePathForInstance(int instanceIndex) const override
	{
		return m_page->robotUrdfAbsolutePathForInstance(instanceIndex);
	}
	int robotRevoluteJointCountForInstance(int instanceIndex) const override
	{
		return m_page->robotRevoluteJointCountForInstance(instanceIndex);
	}
	QString robotJointKeyPrefixForInstance(int instanceIndex) const override
	{
		return m_page->robotJointKeyPrefixForInstance(instanceIndex);
	}
	bool robotUsesPerLinkBackendsForInstance(int instanceIndex) const override
	{
		return m_page->robotUsesPerLinkBackendsForInstance(instanceIndex);
	}
	bool robotPerLinkKinematicsForInstance(int instanceIndex,
										   cloudsim::core::RobotPerLinkKinematicsSliceDto& out) const override
	{
		return m_page->robotPerLinkKinematicsForInstance(instanceIndex, out);
	}
	QString robotSceneBackendIdForInstance(int instanceIndex) const override
	{
		return m_page->robotSceneBackendIdForInstance(instanceIndex);
	}
	QString robotFrameWorldReferenceBackendId(int instanceIndex) const override
	{
		return m_page->robotFrameWorldReferenceBackendId(instanceIndex);
	}
	QString robotDisplayLabelForInstance(int instanceIndex) const override
	{
		return m_page->robotDisplayLabelForInstance(instanceIndex);
	}
	QStringList robotRevoluteJointNamesForInstance(int instanceIndex) const override
	{
		return m_page->robotRevoluteJointNamesForInstance(instanceIndex);
	}
	void robotJointLimitsForInstance(int instanceIndex, QVector<double>& lowerRad,
									 QVector<double>& upperRad) const override
	{
		m_page->robotJointLimitsForInstance(instanceIndex, lowerRad, upperRad);
	}
	int robotJointOffsetInAggregatedVector(int instanceIndex) const override
	{
		return m_page->robotJointOffsetInAggregatedVector(instanceIndex);
	}
	int robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const override
	{
		return m_page->robotInstanceIndexForSceneBackendId(sceneBackendId);
	}
	int robotInstanceIndexForPerLinkBackend(const QString& backendId, bool* outIsSceneRoot) const override
	{
		return m_page->robotInstanceIndexForPerLinkBackend(backendId, outIsSceneRoot);
	}
	RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) override
	{
		return m_page->robotCoordinateFramesForInstance(instanceIndex);
	}
	const RobotCoordinate::RobotCoordinateFrameSet& robotCoordinateFramesForInstance(int instanceIndex) const override
	{
		return m_page->robotCoordinateFramesForInstance(instanceIndex);
	}
	RobotExternal::RobotExternalAxisConfigSet& robotExternalAxesForInstance(int instanceIndex) override
	{
		return m_page->robotExternalAxesForInstance(instanceIndex);
	}
	const RobotExternal::RobotExternalAxisConfigSet& robotExternalAxesForInstance(int instanceIndex) const override
	{
		return m_page->robotExternalAxesForInstance(instanceIndex);
	}
	RobotCollision::Settings& robotCollisionSettings() override { return m_page->robotCollisionSettings(); }
	const RobotCollision::Settings& robotCollisionSettings() const override
	{
		return m_page->robotCollisionSettings();
	}
	const RobotCoordinate::RobotUserFrame* robotActiveUserFrameForInstance(int instanceIndex) const override
	{
		return m_page->robotActiveUserFrameForInstance(instanceIndex);
	}
	void setRobotBasePlacementWorldForInstance(int instanceIndex,
											   const cloudsim::core::Mat4& placementWorld) override
	{
		m_page->setRobotBasePlacementWorldForInstance(instanceIndex, placementWorld);
	}
	cloudsim::core::Mat4 robotBasePlacementWorldForInstance(int instanceIndex) const override
	{
		return m_page->robotBasePlacementWorldForInstance(instanceIndex);
	}
	void setRobotExternalAxisQMm(int instanceIndex, double qMm) override
	{
		m_page->setRobotExternalAxisQMm(instanceIndex, qMm);
	}
	double robotExternalAxisQMm(int instanceIndex) const override
	{
		return m_page->robotExternalAxisQMm(instanceIndex);
	}
	void setRobotExternalAxisQ(int instanceIndex, const std::vector<double>& qValues) override
	{
		m_page->setRobotExternalAxisQ(instanceIndex, qValues);
	}
	std::vector<double> robotExternalAxisQ(int instanceIndex) const override
	{
		return m_page->robotExternalAxisQ(instanceIndex);
	}
	cloudsim::core::Mat4 workpieceExternalBasePlacement(int instanceIndex, const QString& backendId) const override
	{
		return m_page->workpieceExternalBasePlacement(instanceIndex, backendId);
	}
	void setWorkpieceExternalBasePlacement(int instanceIndex, const QString& backendId,
										   const cloudsim::core::Mat4& w0) override
	{
		m_page->setWorkpieceExternalBasePlacement(instanceIndex, backendId, w0);
	}
	void ensureWorkpieceExternalBasePlacement(int instanceIndex, const QString& backendId,
											  const cloudsim::core::Mat4& currentWorld) override
	{
		m_page->ensureWorkpieceExternalBasePlacement(instanceIndex, backendId, currentWorld);
	}
	cloudsim::core::Mat4 workpieceWorkingFrameOffset(int instanceIndex, const QString& boundBackendId) const override
	{
		return m_page->workpieceWorkingFrameOffset(instanceIndex, boundBackendId);
	}
	void ensureWorkpieceWorkingFrameOffset(int instanceIndex, const QString& boundBackendId,
										   const QString& workingFrameId,
										   const cloudsim::core::Mat4& workingWorld) override
	{
		m_page->ensureWorkpieceWorkingFrameOffset(instanceIndex, boundBackendId, workingFrameId, workingWorld);
	}
	void updateRobotLinkOuterBindFromWorld(int instanceIndex, const QString& linkBackendId,
										   const cloudsim::core::Mat4& world) override
	{
		m_page->updateRobotLinkOuterBindFromWorld(instanceIndex, linkBackendId, world);
	}
	void reconcilePerLinkOuterBindFromScene(int instanceIndex, const QVector<double>& jointAnglesRad) override
	{
		m_page->reconcilePerLinkOuterBindFromScene(instanceIndex, jointAnglesRad);
	}
	void notifyRobotKinematicsAppliedToScene() override { m_page->notifyRobotKinematicsAppliedToScene(); }
	void requestFollowSolveForced() override { m_page->requestFollowSolveForced(); }
	void setSuppressRobotFollowDirtyNotify(const bool suppress) override
	{
		m_page->setSuppressRobotFollowDirtyNotify(suppress);
	}
	void clearFollowDirtyBackendIds() override { m_page->clearFollowDirtyBackendIds(); }

	bool applyJointAnglesRad(int instanceIndex, const QVector<double>& jointAnglesRad,
							 QVector<double>& aggregatedJointAnglesRad, QString* outError) override
	{
		if (!m_page || !m_page->hasRobotSimulationContext())
		{
			if (outError)
			{
				*outError = QStringLiteral("no robot simulation context");
			}
			return false;
		}
		IRobotBackendPoseSink* poseSink = m_page->sceneFacade().poseSink();
		if (!poseSink)
		{
			if (outError)
			{
				*outError = QStringLiteral("no pose sink");
			}
			return false;
		}
		const int nj = m_page->robotRevoluteJointCountForInstance(instanceIndex);
		if (jointAnglesRad.size() != nj)
		{
			if (outError)
			{
				*outError =
					QStringLiteral("joint count mismatch: expected %1, got %2").arg(nj).arg(jointAnglesRad.size());
			}
			return false;
		}
		const QString sceneRootId = m_page->robotSceneBackendIdForInstance(instanceIndex);
		if (sceneRootId.isEmpty())
		{
			if (outError)
			{
				*outError = QStringLiteral("no scene root for instance %1").arg(instanceIndex);
			}
			return false;
		}
		if (!m_page->robot().applyJointAnglesRad(sceneRootId, jointAnglesRad, &aggregatedJointAnglesRad, outError))
		{
			return false;
		}
		return true;
	}

	bool captureToolFrameFromTcp(int instanceIndex, const BackendMat4& T_base_tcp,
								 const QVector<double>& jointAnglesRad, const QString& flangeLinkName,
								 RobotCoordinate::RobotCoordinateFrameSet& frames, QString* outError) override
	{
		if (!m_page)
		{
			if (outError)
			{
				*outError = QStringLiteral("no document page");
			}
			return false;
		}
		const QString urdfPath = m_page->robotUrdfAbsolutePathForInstance(instanceIndex);
		std::string flangeLink = flangeLinkName.toStdString();
		if (flangeLink.empty())
		{
			if (outError)
			{
				*outError = QStringLiteral("no flange link name");
			}
			return false;
		}
		QHash<QString, osg::Matrixd> linkWorld;
		QString err;
		if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, jointAnglesRad, linkWorld, &err))
		{
			if (outError)
			{
				*outError = err;
			}
			return false;
		}
		const QString flangeQ = QString::fromStdString(flangeLink);
		if (!linkWorld.contains(flangeQ))
		{
			if (outError)
			{
				*outError = QStringLiteral("flange link not found: %1").arg(flangeQ);
			}
			return false;
		}
		const BackendMat4 T_base_flange = RobotMatrixOsg::backendColMajorFromMatrix(linkWorld.value(flangeQ));
		BackendMat4 invFlange{};
		if (!backend_mat4_invert_rigid(T_base_flange, invFlange))
		{
			if (outError)
			{
				*outError = QStringLiteral("cannot invert flange matrix");
			}
			return false;
		}
		BackendMat4 T_flange_tool{};
		backend_mat4_multiply(invFlange, T_base_tcp, T_flange_tool);
		RobotCoordinate::RobotToolFrame* target = nullptr;
		for (RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
		{
			if (tf.id == frames.activeToolFrameId)
			{
				target = &tf;
				break;
			}
		}
		if (!target && !frames.toolFrames.empty())
		{
			target = &frames.toolFrames.front();
		}
		if (!target)
		{
			if (outError)
			{
				*outError = QStringLiteral("no tool frame available");
			}
			return false;
		}
		target->T_flange_tool = RobotCoordinate::mat4ToFrame(T_flange_tool);
		return true;
	}

	bool captureUserFrameFromTcp(int /*instanceIndex*/, double posXmm, double posYmm, double posZmm, double eulerXdeg,
								 double eulerYdeg, double eulerZdeg, RobotCoordinate::RobotCoordinateFrameSet& frames,
								 QString* outError) override
	{
		RobotCoordinate::RobotUserFrame* target = nullptr;
		for (RobotCoordinate::RobotUserFrame& uf : frames.userFrames)
		{
			if (uf.id == frames.activeUserFrameId)
			{
				target = &uf;
				break;
			}
		}
		if (!target && !frames.userFrames.empty())
		{
			target = &frames.userFrames.front();
		}
		if (!target)
		{
			if (outError)
			{
				*outError = QStringLiteral("no user frame available");
			}
			return false;
		}
		target->T_base_user.positionMm[0] = posXmm;
		target->T_base_user.positionMm[1] = posYmm;
		target->T_base_user.positionMm[2] = posZmm;
		target->T_base_user.eulerDeg[0] = eulerXdeg;
		target->T_base_user.eulerDeg[1] = eulerYdeg;
		target->T_base_user.eulerDeg[2] = eulerZdeg;
		return true;
	}

	void resetToolFrame(int /*instanceIndex*/, RobotCoordinate::RobotCoordinateFrameSet& frames) override
	{
		for (RobotCoordinate::RobotToolFrame& tf : frames.toolFrames)
		{
			if (tf.id == frames.activeToolFrameId)
			{
				tf.T_flange_tool = RobotCoordinate::identityRigidFrame();
				break;
			}
		}
	}

	TcpDragIkResult solveTcpDragTeachIk(int instanceIndex, double pxMm, double pyMm, double pzMm, double exDeg,
										double eyDeg, double ezDeg, const QVector<double>& seedJointRad,
										const QString& ikLinkName, const std::vector<double>& externalAxisQSeed = {},
										bool hasExternalAxisQSeed = false) override
	{
		TcpDragIkResult result;
		if (!m_page)
		{
			result.error = QStringLiteral("no document page");
			return result;
		}
		const QString urdfPath = m_page->robotUrdfAbsolutePathForInstance(instanceIndex);
		if (urdfPath.isEmpty())
		{
			result.error = QStringLiteral("no URDF path");
			return result;
		}
		const RobotCoordinate::RobotCoordinateFrameSet& frames =
			m_page->robotCoordinateFramesForInstance(instanceIndex);
		BackendMat4 toolMat = BackendMat4::identity();
		if (const RobotCoordinate::RobotToolFrame* tool = RobotCoordinate::activeToolFrame(frames))
		{
			toolMat = RobotCoordinate::frameToMat4(tool->T_flange_tool);
		}
		RobotTeachIk::TeachIkContext ctx;
		ctx.urdfPath = urdfPath;
		ctx.ikLinkName = ikLinkName;
		ctx.T_base_target = engine::RigidTransform::fromTranslationEulerDeg(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
		ctx.seedJointRad.reserve(static_cast<size_t>(seedJointRad.size()));
		for (double v : seedJointRad)
		{
			ctx.seedJointRad.push_back(v);
		}
		ctx.useOrientation = true;
		ctx.T_flange_tool = toolMat;
		// 拖动：小步增量 + 与 FK/罗盘同源的 URDF DLS
		ctx.maxIkIterations = 32;
		ctx.options.maxIterations = 32;
		ctx.options.maxJointStepRad = 0.22;
		ctx.options.positionToleranceMm = 1.0;
		ctx.options.orientationToleranceRad = 0.35 * 3.14159265358979323846 / 180.0;
		const QString sceneRootId = m_page->robotSceneBackendIdForInstance(instanceIndex);
		if (!sceneRootId.isEmpty())
		{
			ctx.registryKey = KinematicModelRegistry::keyRobotInstance(sceneRootId.toStdString());
		}

		auto solveIk = [](const RobotTeachIk::TeachIkContext& c) -> RobotTeachIk::TeachIkResult {
			if (!c.registryKey.empty())
			{
				return KinematicModelIk::solveTeachPose(c.registryKey, c);
			}
			return RobotTeachIk::solveTeachIk(c);
		};
		auto solveIkDrag = [](const RobotTeachIk::TeachIkContext& c, const double hint,
							  const bool hasHint) -> RobotTeachIk::TeachIkResult {
			if (!c.registryKey.empty())
			{
				RobotTeachIk::TeachIkContext cc = c;
				return RobotTeachIk::solveTeachIkCoordinatedDrag(cc, hint, hasHint);
			}
			return RobotTeachIk::solveTeachIkCoordinatedDrag(c, hint, hasHint);
		};

		const RobotExternal::RobotExternalAxisConfigSet& extSet =
			m_page->robotExternalAxesForInstance(instanceIndex);
		const std::vector<double> qDoc = m_page->robotExternalAxisQ(instanceIndex);
		const int configCount = static_cast<int>(extSet.axes.size());
		ctx.externalAxisConfigCount = configCount;

		std::vector<double> qeFull(static_cast<size_t>(std::max(0, configCount)), 0.0);
		for (size_t i = 0; i < extSet.axes.size(); ++i)
		{
			double q = (i < qDoc.size()) ? qDoc[i] : extSet.axes[i].home;
			if (hasExternalAxisQSeed && i < externalAxisQSeed.size())
			{
				q = externalAxisQSeed[i];
			}
			qeFull[i] = std::clamp(q, extSet.axes[i].lower, extSet.axes[i].upper);
		}

		auto buildDof = [&](RobotExternal::RobotExternalAttachment att,
							const std::vector<double>& qe) -> RobotTeachIk::TeachIkExternalAxisDof {
			RobotTeachIk::TeachIkExternalAxisDof dof;
			dof.optimizeExternal = true;
			dof.adaptiveExternalDamping = true;
			for (size_t i = 0; i < extSet.axes.size(); ++i)
			{
				const RobotExternal::RobotExternalAxisConfig& a = extSet.axes[i];
				if (!a.enabled || a.attachment != att)
				{
					continue;
				}
				RobotTeachIk::TeachIkExternalAxisSlot slot;
				slot.configIndex = static_cast<int>(i);
				slot.isPrismatic = a.motionType == RobotExternal::RobotExternalMotionType::Translate;
				slot.axis[0] = a.axis[0];
				slot.axis[1] = a.axis[1];
				slot.axis[2] = a.axis[2];
				slot.originMm[0] = a.originMm[0];
				slot.originMm[1] = a.originMm[1];
				slot.originMm[2] = a.originMm[2];
				slot.lower = a.lower;
				slot.upper = a.upper;
				dof.axes.push_back(slot);
				dof.qExternal.push_back(qe[i]);
			}
			return dof;
		};

		auto appendSparseSamples = [](const RobotTeachIk::TeachIkExternalAxisDof& dof,
									  std::vector<std::vector<double>>& samples) {
			const int dofN = static_cast<int>(dof.axes.size());
			if (dofN <= 0)
			{
				return;
			}
			if (dofN == 1)
			{
				const auto& ax = dof.axes.front();
				const double span = std::max(0.0, ax.upper - ax.lower);
				const int gridN = span < 1e-9 ? 1 : std::clamp(static_cast<int>(span / 200.0) + 1, 3, 5);
				for (int i = 0; i < gridN; ++i)
				{
					const double t = gridN <= 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(gridN - 1);
					samples.push_back({ax.lower + t * (ax.upper - ax.lower)});
				}
				return;
			}
			std::vector<int> gridN(static_cast<size_t>(dofN), 1);
			int total = 1;
			for (int i = 0; i < dofN; ++i)
			{
				const double span =
					std::max(0.0, dof.axes[static_cast<size_t>(i)].upper - dof.axes[static_cast<size_t>(i)].lower);
				gridN[static_cast<size_t>(i)] =
					span < 1e-9 ? 1 : std::clamp(static_cast<int>(span / 250.0) + 1, 2, dofN >= 3 ? 2 : 3);
				total *= gridN[static_cast<size_t>(i)];
			}
			while (total > 32)
			{
				int maxIdx = 0;
				for (int i = 1; i < dofN; ++i)
				{
					if (gridN[static_cast<size_t>(i)] > gridN[static_cast<size_t>(maxIdx)])
					{
						maxIdx = i;
					}
				}
				if (gridN[static_cast<size_t>(maxIdx)] <= 2)
				{
					break;
				}
				total /= gridN[static_cast<size_t>(maxIdx)];
				--gridN[static_cast<size_t>(maxIdx)];
				total *= gridN[static_cast<size_t>(maxIdx)];
			}
			std::vector<int> idx(static_cast<size_t>(dofN), 0);
			for (;;)
			{
				std::vector<double> sample(static_cast<size_t>(dofN), 0.0);
				for (int i = 0; i < dofN; ++i)
				{
					const auto& ax = dof.axes[static_cast<size_t>(i)];
					const int gn = gridN[static_cast<size_t>(i)];
					const double t =
						gn <= 1 ? 0.0
								: static_cast<double>(idx[static_cast<size_t>(i)]) / static_cast<double>(gn - 1);
					sample[static_cast<size_t>(i)] = ax.lower + t * (ax.upper - ax.lower);
				}
				samples.push_back(std::move(sample));
				int carry = 0;
				++idx[0];
				while (carry < dofN && idx[static_cast<size_t>(carry)] >= gridN[static_cast<size_t>(carry)])
				{
					idx[static_cast<size_t>(carry)] = 0;
					++carry;
					if (carry < dofN)
					{
						++idx[static_cast<size_t>(carry)];
					}
				}
				if (carry >= dofN)
				{
					break;
				}
			}
		};

		auto rigidFromCol16 = [](const double m[16]) -> engine::RigidTransform {
			BackendMat4 bm{};
			for (int i = 0; i < 16; ++i)
			{
				bm.v[i] = m[i];
			}
			return RobotCoordinate::rigidTransformFromBackendMat4(bm);
		};

		auto costOf = [&](const RobotTeachIk::TeachIkResult& r, const std::vector<double>& qeTry) {
			double c = r.residualTcpMm;
			const size_t nArm = std::min(r.jointRad.size(), ctx.seedJointRad.size());
			for (size_t i = 0; i < nArm; ++i)
			{
				c += 8.0 * std::abs(r.jointRad[i] - ctx.seedJointRad[i]);
			}
			for (size_t i = 0; i < qeTry.size() && i < qeFull.size(); ++i)
			{
				c += 0.02 * std::abs(qeTry[i] - qeFull[i]);
			}
			return c;
		};

		auto fillResult = [&](const RobotTeachIk::TeachIkResult& ik, const RobotTeachIk::TeachIkExternalAxisDof& baseDof,
							  const std::vector<double>& qeBest) {
			result.ok = true;
			result.jointRad.reserve(static_cast<int>(ik.jointRad.size()));
			for (double v : ik.jointRad)
			{
				result.jointRad.push_back(v);
			}
			if (!qeBest.empty() || baseDof.active() || !ik.externalAxisQs.empty())
			{
				result.hasExternalAxisQ = true;
				result.externalAxisQs = qeBest;
				if (result.externalAxisQs.empty())
				{
					result.externalAxisQs.assign(static_cast<size_t>(configCount), 0.0);
				}
				if (static_cast<int>(ik.externalAxisQs.size()) == configCount)
				{
					result.externalAxisQs = ik.externalAxisQs;
				}
				else
				{
					for (size_t i = 0; i < baseDof.axes.size() && i < ik.externalAxisQs.size(); ++i)
					{
						const int cidx = baseDof.axes[i].configIndex;
						if (cidx >= 0 && cidx < configCount)
						{
							result.externalAxisQs[static_cast<size_t>(cidx)] = ik.externalAxisQs[i];
						}
					}
					if (ik.externalAxisQs.empty())
					{
						for (size_t i = 0; i < baseDof.axes.size() && i < baseDof.qExternal.size(); ++i)
						{
							const int cidx = baseDof.axes[i].configIndex;
							if (cidx >= 0 && cidx < configCount)
							{
								result.externalAxisQs[static_cast<size_t>(cidx)] = baseDof.qExternal[i];
							}
						}
					}
				}
				result.externalAxisQ = result.externalAxisQs.empty() ? ik.externalAxisQ : result.externalAxisQs.front();
				for (size_t i = 0; i < extSet.axes.size() && i < result.externalAxisQs.size(); ++i)
				{
					if (extSet.axes[i].enabled &&
						extSet.axes[i].attachment == RobotExternal::RobotExternalAttachment::RobotBase)
					{
						result.externalAxisQ = result.externalAxisQs[i];
						break;
					}
				}
			}
		};

		RobotTeachIk::TeachIkExternalAxisDof baseDof = buildDof(RobotExternal::RobotExternalAttachment::RobotBase, qeFull);
		const bool useWorkpieceRep = RobotExternal::hasEnabledWorkpieceExternalAxes(extSet);
		RobotTeachIk::TeachIkResult bestIk;
		bestIk.ok = false;
		double bestCost = std::numeric_limits<double>::infinity();
		std::vector<double> bestQe = qeFull;
		RobotTeachIk::TeachIkExternalAxisDof bestBaseDof = baseDof;

		if (useWorkpieceRep)
		{
			const std::string boundId = RobotExternal::primaryWorkpieceBackendId(extSet);
			const QString boundQ = QString::fromStdString(boundId);
			const QString workFrameQ = QString::fromStdString(RobotExternal::resolveWorkingFrameId(extSet));
			if (IRobotBackendPoseSink* sink = m_page->sceneFacade().poseSink())
			{
				cloudsim::core::Mat4 curBound = cloudsim::core::PlanContextDto::identityMat4();
				if (!boundId.empty() && sink->getBackendRootWorldMatrix(boundId, curBound))
				{
					cloudsim::core::Mat4 w0Cand = curBound;
					RobotExternal::unbakeWorkpiecePlacementExternalAxis(curBound.data(), extSet, boundId, qeFull,
																		w0Cand.data());
					m_page->ensureWorkpieceExternalBasePlacement(instanceIndex, boundQ, w0Cand);
				}
				if (!workFrameQ.isEmpty() && workFrameQ != boundQ)
				{
					cloudsim::core::Mat4 workWorld = cloudsim::core::PlanContextDto::identityMat4();
					if (sink->getBackendRootWorldMatrix(workFrameQ.toStdString(), workWorld))
					{
						m_page->ensureWorkpieceWorkingFrameOffset(instanceIndex, boundQ, workFrameQ, workWorld);
					}
				}
				else
				{
					m_page->ensureWorkpieceWorkingFrameOffset(instanceIndex, boundQ, boundQ,
															  cloudsim::core::PlanContextDto::identityMat4());
				}
			}

			const cloudsim::core::Mat4 p0 = m_page->robotBasePlacementWorldForInstance(instanceIndex);
			const cloudsim::core::Mat4 w0 = m_page->workpieceExternalBasePlacement(instanceIndex, boundQ);
			const cloudsim::core::Mat4 offset = m_page->workpieceWorkingFrameOffset(instanceIndex, boundQ);

			double tp0WorkCur[16];
			if (!boundId.empty() &&
				RobotExternal::composeWorkpieceWorkingFrameInRobotP0(p0.data(), w0.data(), extSet, boundId, qeFull,
																	 offset.data(), tp0WorkCur))
			{
				const engine::RigidTransform T_p0_work_cur = rigidFromCol16(tp0WorkCur);
				const engine::RigidTransform T_work = T_p0_work_cur.inverse().composeColumn(ctx.T_base_target);

				RobotTeachIk::TeachIkExternalAxisDof wpDof =
					buildDof(RobotExternal::RobotExternalAttachment::Workpiece, qeFull);
				std::vector<std::vector<double>> wpSamples;
				wpSamples.push_back(wpDof.qExternal);
				appendSparseSamples(wpDof, wpSamples);

				const std::vector<int> wpIdx = RobotExternal::enabledExternalAxisIndicesForAttachment(
					extSet, RobotExternal::RobotExternalAttachment::Workpiece);

				for (const std::vector<double>& qw : wpSamples)
				{
					std::vector<double> qeTry = qeFull;
					for (size_t i = 0; i < wpIdx.size() && i < qw.size(); ++i)
					{
						const int cidx = wpIdx[i];
						if (cidx >= 0 && cidx < configCount)
						{
							qeTry[static_cast<size_t>(cidx)] = qw[i];
						}
					}
					double tp0Work[16];
					if (!RobotExternal::composeWorkpieceWorkingFrameInRobotP0(p0.data(), w0.data(), extSet, boundId,
																			 qeTry, offset.data(), tp0Work))
					{
						continue;
					}
					ctx.T_base_target = rigidFromCol16(tp0Work).composeColumn(T_work);
					RobotTeachIk::TeachIkExternalAxisDof tryBase = buildDof(
						RobotExternal::RobotExternalAttachment::RobotBase, qeTry);
					RobotTeachIk::TeachIkResult ik;
					if (tryBase.active())
					{
						ctx.externalAxes = tryBase;
						const double hint = tryBase.qExternal.empty() ? 0.0 : tryBase.qExternal.front();
						ik = solveIkDrag(ctx, hint, true);
					}
					else
					{
						ctx.externalAxes = {};
						ik = solveIk(ctx);
					}
					if (!ik.ok)
					{
						continue;
					}
					const double c = costOf(ik, qeTry);
					if (c < bestCost)
					{
						bestCost = c;
						bestIk = ik;
						bestQe = qeTry;
						bestBaseDof = tryBase;
					}
					if (bestIk.ok && bestIk.residualTcpMm < 1.0 && c < 8.0)
					{
						break;
					}
				}
			}
		}

		if (!bestIk.ok)
		{
			ctx.T_base_target =
				engine::RigidTransform::fromTranslationEulerDeg(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
			if (baseDof.active())
			{
				ctx.externalAxes = baseDof;
				const double hint = baseDof.qExternal.empty() ? 0.0 : baseDof.qExternal.front();
				bestIk = solveIkDrag(ctx, hint, hasExternalAxisQSeed);
			}
			else
			{
				ctx.externalAxes = {};
				bestIk = solveIk(ctx);
			}
			bestQe = qeFull;
			bestBaseDof = baseDof;
		}

		if (!bestIk.ok || bestIk.residualTcpMm > 5.0)
		{
			ctx.T_base_target =
				engine::RigidTransform::fromTranslationEulerDeg(pxMm, pyMm, pzMm, exDeg, eyDeg, ezDeg);
			ctx.useOrientation = false;
			ctx.maxIkIterations = 80;
			ctx.options.maxIterations = 80;
			RobotTeachIk::TeachIkResult posOnly{};
			if (baseDof.active())
			{
				ctx.externalAxes = baseDof;
				const double hint = baseDof.qExternal.empty() ? 0.0 : baseDof.qExternal.front();
				posOnly = solveIkDrag(ctx, hint, hasExternalAxisQSeed);
			}
			else
			{
				ctx.externalAxes = {};
				posOnly = solveIk(ctx);
			}
			if (posOnly.ok && (!bestIk.ok || posOnly.residualTcpMm < bestIk.residualTcpMm))
			{
				bestIk = posOnly;
				bestQe = qeFull;
				bestBaseDof = baseDof;
			}
		}

		if (!bestIk.ok)
		{
			result.error = bestIk.error.empty() ? QStringLiteral("IK solve failed")
												: QString::fromStdString(bestIk.error);
			return result;
		}
		fillResult(bestIk, bestBaseDof, bestQe);
		return result;
	}

	bool planForExport(int instanceIndex, const std::vector<std::shared_ptr<RobotInstruction::Base>>& instructions,
					   const QVector<double>& seedJointRad, const QString& urdfPath, const QString& tcpLinkName,
					   std::vector<ExportPlanResult>& outPlans, int& outFailedCount, QString* outError) override
	{
		if (!m_page)
		{
			if (outError)
			{
				*outError = QStringLiteral("no document page");
			}
			return false;
		}
		outPlans.clear();
		outFailedCount = 0;
		QVector<double> rollingQ = seedJointRad;
		for (const auto& insPtr : instructions)
		{
			ExportPlanResult result;
			if (!insPtr)
			{
				result.ok = false;
				result.summary = "Invalid instruction";
				outPlans.push_back(result);
				++outFailedCount;
				continue;
			}
			// 规划由 Controller 的 m_instructionController 处理
			// 这里只准备上下文，实际规划需 Controller 调用
			result.ok = true;
			outPlans.push_back(result);
		}
		return true;
	}

	QString meshBackendStepSourcePath(const QString& backendId) const override
	{
		return m_page->backendSourcePath().value(backendId);
	}

private:
	DocumentPage* m_page = nullptr;
};

MainWindowRobotHost::MainWindowRobotHost(MainWindow* mw) : m_mw(mw) {}

MainWindowRobotHost::~MainWindowRobotHost() = default;

IRobotDocumentHost* MainWindowRobotHost::document()
{
	if (!m_mw)
	{
		m_docHost.reset();
		return nullptr;
	}
	if (DocumentPage* p = m_mw->currentPage())
	{
		if (!m_docHost || m_docHost->page() != p)
		{
			m_docHost = std::make_unique<DocumentHost>(p);
		}
		return m_docHost.get();
	}
	m_docHost.reset();
	return nullptr;
}

const IRobotDocumentHost* MainWindowRobotHost::document() const
{
	return const_cast<MainWindowRobotHost*>(this)->document();
}

IRobotOsgViewHost* MainWindowRobotHost::osgView()
{
	DocumentPage* page = m_mw->currentPage();
	if (!page)
	{
		m_osgHost.reset();
		m_osgHostPage = nullptr;
		return nullptr;
	}
	if (!page->osgWidget())
	{
		return nullptr;
	}
	if (!m_osgHost || m_osgHostPage != page)
	{
		m_osgHost = std::make_unique<WidgetOsgViewHost>(page);
		m_osgHostPage = page;
	}
	return m_osgHost.get();
}

void MainWindowRobotHost::endMeshSectionPlaneEditDirect()
{
	auto tryEndOnPage = [](DocumentPage* page) -> bool
	{
		if (!page)
		{
			return false;
		}
		if (OsgWidget* w = page->osgWidget())
		{
			w->endMeshSectionPlaneEdit();
			return true;
		}
		return false;
	};
	if (tryEndOnPage(m_osgHostPage))
	{
		return;
	}
	(void)tryEndOnPage(m_mw->currentPage());
}

void MainWindowRobotHost::hideMeshSectionPlaneDirect()
{
	auto tryHideOnPage = [](DocumentPage* page) -> bool
	{
		if (!page)
		{
			return false;
		}
		if (OsgWidget* w = page->osgWidget())
		{
			w->hideMeshSectionPlane();
			return true;
		}
		return false;
	};
	if (tryHideOnPage(m_osgHostPage))
	{
		return;
	}
	(void)tryHideOnPage(m_mw->currentPage());
}

bool MainWindowRobotHost::useChinese() const
{
	return m_mw->m_useChinese;
}

QString MainWindowRobotHost::i18n(const QString& en, const QString& zh) const
{
	return m_mw->i18n(en, zh);
}

RunInfoPage* MainWindowRobotHost::runInfoPage()
{
	return m_mw->m_runInfoPage;
}

void MainWindowRobotHost::appendRunInfo(const QString& message)
{
	if (m_mw->m_runInfoPage)
	{
		m_mw->m_runInfoPage->appendInfo(message);
	}
}

void MainWindowRobotHost::appendRunWarning(const QString& message)
{
	if (m_mw->m_runInfoPage)
	{
		m_mw->m_runInfoPage->appendWarning(message);
	}
}

QStatusBar* MainWindowRobotHost::statusBar()
{
	return m_mw->statusBar();
}

SimulationCommandWidget* MainWindowRobotHost::simulationCommandPage()
{
	RobotSimulationDockWidget* dock = m_mw->m_robotSimulation ? m_mw->m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->commandPage() : nullptr;
}

RobotAxisControlWidget* MainWindowRobotHost::robotAxisControlPage()
{
	RobotSimulationDockWidget* dock = m_mw->m_robotSimulation ? m_mw->m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->axisPage() : nullptr;
}

RobotFrameSettingsWidget* MainWindowRobotHost::robotFrameSettingsPage()
{
	RobotSimulationDockWidget* dock = m_mw->m_robotSimulation ? m_mw->m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->framePage() : nullptr;
}

RobotExternalAxisSettingsWidget* MainWindowRobotHost::robotExternalAxisSettingsPage()
{
	RobotSimulationDockWidget* dock = m_mw->m_robotSimulation ? m_mw->m_robotSimulation->simulationDock() : nullptr;
	return dock ? dock->externalAxisPage() : nullptr;
}

DevicePageWidget* MainWindowRobotHost::devicePage()
{
	return m_mw->m_devicePage;
}

QAction* MainWindowRobotHost::simulationStartAction()
{
	return nullptr;
}

int MainWindowRobotHost::currentSimulationRobotInstanceIndex() const
{
	return m_mw->currentSimulationRobotInstanceIndex();
}

void MainWindowRobotHost::refreshBackendTree()
{
	m_mw->refreshBackendTree();
}

void MainWindowRobotHost::runFollowSolveAndSyncForCurrentDocument()
{
	DocumentPage* page = m_mw->currentPage();
	if (!page)
	{
		return;
	}
	// TCP 拖动等高频 FK 后须 forced + flush，否则 follower 仍读陈旧父级
	page->requestFollowSolveForced();
	(void)page->flushVisualSync();
	cloudsim::core::FollowSolveContextDto ctx;
	ctx.skipAll = false;
	(void)page->data().runFollowSolveAndSync(ctx, nullptr);
	cloudsim::host::refreshCustomDevicesFollowingKinematicsTargets(*page);
}

void MainWindowRobotHost::rebakeMountedCustomDevicesFollowLocalsForCurrentDocument()
{
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host)
	{
		return;
	}
	cloudsim::host::rebakeMountedCustomDevicesFollowLocals(*host);
}

void MainWindowRobotHost::prepareCustomDeviceAxisControlTarget(const QString& deviceBackendId)
{
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host || deviceBackendId.isEmpty())
	{
		return;
	}
	cloudsim::host::finalizeCustomDeviceLinkJointGraph(*host, deviceBackendId.toStdString());
}

void MainWindowRobotHost::flushCustomDeviceLinkGeometryVisual(const QString& deviceBackendId)
{
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host || deviceBackendId.isEmpty())
	{
		return;
	}
	cloudsim::host::flushCustomDeviceLinkGeometryVisual(*host, deviceBackendId.toStdString());
}

void MainWindowRobotHost::refreshInstructionPropertyPanel(const std::shared_ptr<RobotInstruction::Base>& instruction,
														  const bool refreshFeasibleAxisOptions)
{
	m_mw->updateInstructionPropertyPanel(instruction, refreshFeasibleAxisOptions);
}

void MainWindowRobotHost::clearInstructionPropertyPanel()
{
	m_mw->updateInstructionPropertyPanel(nullptr, true);
}

void MainWindowRobotHost::invalidateInstructionPropertyCache()
{
	m_mw->invalidateFeasibleAxisConfigurationCache();
}

void MainWindowRobotHost::clearBackendObjectSelection(const bool clearTreeSelection)
{
	MainWindowSelectionService::clearBackendObjectSelection(*m_mw, clearTreeSelection);
}

QString MainWindowRobotHost::selectedBackendId() const
{
	return m_mw ? m_mw->m_selectionState.selectedBackendId() : QString();
}

std::shared_ptr<RobotInstruction::Base> MainWindowRobotHost::activeInstructionForProperty() const
{
	return m_mw->m_activeInstructionForProperty;
}

void MainWindowRobotHost::applySuggestedAxisPresetFromSeedIfNeeded(
	const std::shared_ptr<RobotInstruction::Base>& instruction, const QVector<double>& seedJointRad,
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions& feasible)
{
	m_mw->applySuggestedAxisPresetFromSeedIfNeeded(instruction, seedJointRad, feasible);
}

bool MainWindowRobotHost::registerUrdfRobot(const QString& urdfPath, const bool quietUi)
{
	MainWindowImportCaptureRenderController controller;
	return controller.registerUrdfRobot(*m_mw, urdfPath, quietUi);
}

bool MainWindowRobotHost::planRobotMotionInstruction(RobotInstruction::Base& instruction,
													 const QVector<double>& seedJointRad, const int instanceIndex,
													 const QString& urdfPath, const QString& defaultTcpLinkName,
													 const QString& sceneRootBackendId,
													 RobotInstruction::PlanResult& out, std::string* outErr)
{
	DocumentPage* page = m_mw->currentPage();
	if (!page)
	{
		if (outErr)
		{
			*outErr = "no active document";
		}
		return false;
	}
	QString hostErr;
	const bool ok =
		cloudsim::host::planRobotInstruction(*page, instruction, seedJointRad, instanceIndex, urdfPath,
											 defaultTcpLinkName.toStdString(), sceneRootBackendId, out, &hostErr);
	if (!ok && outErr)
	{
		*outErr = hostErr.toStdString();
	}
	return ok;
}

void MainWindowRobotHost::enqueueBackgroundJob(const QString& title, std::function<void()> work,
											   std::function<void(bool threw, const QString& msg)> onFinished)
{
	if (!m_mw || !m_mw->jobSystem())
	{
		if (onFinished)
		{
			onFinished(true, QStringLiteral("JobSystem unavailable"));
		}
		return;
	}
	m_mw->jobSystem()->enqueue(
		title,
		[work = std::move(work)](const JobProgressSink&)
		{
			if (work)
			{
				work();
			}
		},
		std::move(onFinished));
}

void MainWindowRobotHost::setMeshPickCommittedHandler(std::function<void(const PickResult&, PickKind)> handler)
{
	m_meshPickHandler = std::move(handler);
}

void MainWindowRobotHost::clearMeshPickCommittedHandler()
{
	m_meshPickHandler = {};
}

void MainWindowRobotHost::notifyMeshPickCommitted(const PickResult& pick, const PickKind kind)
{
	if (m_solidPickCallback && kind == PickKind::MeshFace && pick.hit && pick.brepNativePick &&
		pick.brepFaceIndex >= 0 && !pick.backendId.empty())
	{
		cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
		if (!host)
		{
			return;
		}
		QString partId;
		QString err;
		if (!cloudsim::host::extractBrepSolidByFace(*host, pick.backendId, pick.brepFaceIndex, &partId, &err) ||
			partId.isEmpty())
		{
			if (!err.isEmpty())
			{
				appendRunInfo(err);
			}
			return;
		}
		refreshBackendTree();
		m_solidPickCallback(partId);
		return;
	}
	if (m_meshPickHandler)
	{
		m_meshPickHandler(pick, kind);
	}
}

void MainWindowRobotHost::setMeshTriangleLabelingPickHandlers(
	const IRobotMainWindowHost::MeshTriangleLabelingPickHandlers handlers)
{
	m_meshTriangleLabelingHandlers = std::move(handlers);
}

void MainWindowRobotHost::clearMeshTriangleLabelingPickHandlers()
{
	m_meshTriangleLabelingHandlers = {};
}

void MainWindowRobotHost::notifyMeshTriangleLabelingClick(const PickResult& pick)
{
	if (m_meshTriangleLabelingHandlers.onClick)
	{
		m_meshTriangleLabelingHandlers.onClick(pick);
	}
}

void MainWindowRobotHost::notifyMeshTriangleLabelingBrush(const std::vector<int>& triangleIndices)
{
	if (m_meshTriangleLabelingHandlers.onBrushStroke)
	{
		m_meshTriangleLabelingHandlers.onBrushStroke(triangleIndices);
	}
}

void MainWindowRobotHost::notifyMeshTriangleLabelingPolyline(const QVector<float>& polylineScreenXy,
															 const QVector<double>& mvpMatrix, const int viewportWidth,
															 const int viewportHeight)
{
	if (m_meshTriangleLabelingHandlers.onPolylineClosed)
	{
		m_meshTriangleLabelingHandlers.onPolylineClosed(polylineScreenXy, mvpMatrix, viewportWidth, viewportHeight);
	}
}

void MainWindowRobotHost::wireDocumentPageSceneSignals(DocumentPage* page)
{
	if (!m_mw)
	{
		return;
	}
	wireMainWindowDocumentSceneSignals(*m_mw, page, this);
}

namespace
{
cloudsim::core::FeasibleMotionAxisOptionsDto
toFeasibleAxisDto(const RobotInstruction::FeasibleMotionAxisConfigurationOptions& engine)
{
	return cloudsim::host::feasibleAxisOptionsFromEngine(engine.presetTokens, engine.elbowTokens, engine.wristTokens,
														 engine.armTokens, engine.turnJ1Tokens, engine.turnJ4Tokens,
														 engine.turnJ6Tokens);
}
} // namespace

QVector<cloudsim::core::PropertyRowDto> MainWindowRobotHost::instructionPropertyRows(const QString& instructionId)
{
	if (!m_mw || !m_mw->robotSimulation())
	{
		return {};
	}
	const std::shared_ptr<RobotInstruction::Base> ins = m_mw->robotSimulation()->findInstructionById(instructionId);
	if (!ins)
	{
		return {};
	}
	return cloudsim::host::propertyRowsFromInstructionSnapshotJson(ins->snapshotPropertyRows());
}

bool MainWindowRobotHost::applyInstructionPropertyChange(const QString& instructionId, const QString& key,
														 const QString& value, QString* outError)
{
	if (!m_mw || !m_mw->robotSimulation())
	{
		if (outError)
		{
			*outError = QStringLiteral("no robot simulation");
		}
		return false;
	}
	const std::shared_ptr<RobotInstruction::Base> ins = m_mw->robotSimulation()->findInstructionById(instructionId);
	if (!ins)
	{
		if (outError)
		{
			*outError = QStringLiteral("instruction not found: %1").arg(instructionId);
		}
		return false;
	}
	std::string err;
	const bool ok = ins->applyPropertyChange(key.toStdString(), value.toStdString(), &err);
	if (!ok && outError)
	{
		*outError = QString::fromStdString(err);
	}
	return ok;
}

cloudsim::core::FeasibleMotionAxisOptionsDto
MainWindowRobotHost::queryFeasibleMotionAxisOptions(const QString& instructionId, QVector<double>* outSeedJointRad)
{
	if (!m_mw || !m_mw->robotSimulation())
	{
		return {};
	}
	const std::shared_ptr<RobotInstruction::Base> ins = m_mw->robotSimulation()->findInstructionById(instructionId);
	if (!ins)
	{
		return {};
	}
	const RobotInstruction::FeasibleMotionAxisConfigurationOptions engine =
		m_mw->robotSimulation()->feasibleMotionAxisConfigurationOptionsForInstruction(ins, outSeedJointRad);
	return toFeasibleAxisDto(engine);
}

cloudsim::core::FeasibleMotionAxisOptionsDto MainWindowRobotHost::cachedFeasibleMotionAxisOptions()
{
	if (!m_mw || !m_mw->robotSimulation())
	{
		return {};
	}
	return toFeasibleAxisDto(m_mw->robotSimulation()->cachedFeasibleAxisConfigurationOptions());
}

bool MainWindowRobotHost::registerCustomDevice(const std::shared_ptr<CustomDeviceBackendData>& device,
											   QString* outError)
{
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host || !device)
	{
		if (outError)
		{
			*outError = QStringLiteral("No active document.");
		}
		return false;
	}
	return cloudsim::host::registerAdoptedCustomDeviceAndLoadScene(
		*host, device, QLatin1String(backend_type::kCatalogCustomDevice), QString(), false, outError);
}

bool MainWindowRobotHost::attachChildToCustomDevice(const std::string& deviceId, const std::string& childId,
													QString* outError)
{
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host)
	{
		if (outError)
		{
			*outError = QStringLiteral("No active document.");
		}
		return false;
	}
	return cloudsim::host::attachBackendChildToCustomDevice(*host, deviceId, childId, outError);
}

QStringList MainWindowRobotHost::importModelsForAssembly(QWidget* parent, const QStringList& paths,
														 QStringList* outErrors)
{
	Q_UNUSED(parent);
	QStringList roots;
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host || !m_mw || paths.isEmpty())
	{
		return roots;
	}

	auto isMeshQualityExt = [](const QString& extLower) {
		return extLower == QLatin1String("obj") || extLower == QLatin1String("stl") ||
			   extLower == QLatin1String("ply") || extLower == QLatin1String("off");
	};

	QString qualityProbePath;
	for (const QString& path : paths)
	{
		if (isMeshQualityExt(QFileInfo(path).suffix().toLower()))
		{
			qualityProbePath = path;
			break;
		}
	}
	int quality = m_mw->meshImportQuality();
	if (!qualityProbePath.isEmpty())
	{
		QDialog dialog(m_mw);
		dialog.setWindowTitle(QStringLiteral("Mesh Import Quality"));
		auto* layout = new QVBoxLayout(&dialog);
		auto* form = new QFormLayout();
		auto* qualityCombo = new QComboBox(&dialog);
		qualityCombo->addItem(QStringLiteral("Coarse"), 0);
		qualityCombo->addItem(QStringLiteral("Medium"), 1);
		qualityCombo->addItem(QStringLiteral("Fine"), 2);
		const int idx = qualityCombo->findData(quality);
		qualityCombo->setCurrentIndex(idx >= 0 ? idx : 1);
		form->addRow(QStringLiteral("Triangle density:"), qualityCombo);
		layout->addLayout(form);
		auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
		layout->addWidget(buttons);
		QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
		QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
		if (dialog.exec() != QDialog::Accepted)
		{
			return roots;
		}
		quality = qualityCombo->currentData().toInt();
		m_mw->setMeshImportQuality(quality);
	}

	cloudsim::core::ImportOptionsDto opt;
	opt.resetViewToHome = false;
	opt.quietUi = true;
	opt.catalogTypeName = QLatin1String(backend_type::kCatalogModel);
	opt.meshImportQuality = quality;
	for (const QString& path : paths)
	{
		QString importErr;
		const cloudsim::host::ImportFileResult imported = cloudsim::host::importFileIntoDocument(
			*host, path, cloudsim::host::ImportFileKind::Mesh, opt, &importErr);
		if (!imported.ok || imported.rootBackendId.isEmpty())
		{
			if (outErrors)
			{
				outErrors->append(importErr.isEmpty() ? QStringLiteral("Model import failed: %1").arg(path)
													  : importErr);
			}
			continue;
		}
		roots.append(imported.rootBackendId);
		if (imported.hierarchyImport && !imported.hierarchyDetail.partBackendIds.isEmpty())
		{
			roots.removeLast();
			roots.append(imported.hierarchyDetail.partBackendIds);
		}
	}
	return roots;
}

void MainWindowRobotHost::beginPickSolidInView(std::function<void(const QString& partId)> onPartPicked)
{
	m_solidPickCallback = std::move(onPartPicked);
	clearBackendObjectSelection(true);
	DocumentPage* page = m_mw ? m_mw->currentPage() : nullptr;
	OsgWidget* osg = page ? page->osgWidget() : nullptr;
	if (!osg)
	{
		return;
	}
	osg->setObjectSelectionMode(false);
	osg->setMeshLinePickMode(false);
	osg->setMeshFacePickMode(true);
	osg->syncSelectionForBackendId(std::string());
	osg->setSelectionActive(false);
}

void MainWindowRobotHost::endPickSolidInView()
{
	m_solidPickCallback = {};
	DocumentPage* page = m_mw ? m_mw->currentPage() : nullptr;
	OsgWidget* osg = page ? page->osgWidget() : nullptr;
	if (osg)
	{
		osg->setMeshFacePickMode(false);
	}
}

bool MainWindowRobotHost::exportCustomDeviceUrdfInteractive(const QString& deviceBackendId)
{
	return m_mw ? m_mw->exportCustomDeviceUrdfInteractive(deviceBackendId) : false;
}

void MainWindowRobotHost::markFollowAttachmentDirty(const QString& deviceBackendId)
{
	if (DocumentPage* page = m_mw ? m_mw->currentPage() : nullptr)
	{
		page->markFollowAttachmentDirtyFromBackendMove(deviceBackendId);
	}
}

void MainWindowRobotHost::focusBackendInTree(const QString& backendId)
{
	if (m_mw)
	{
		m_mw->focusBackendInTreeAfterImport(backendId);
	}
}

void MainWindowRobotHost::runFollowSolveAndSync()
{
	runFollowSolveAndSyncForCurrentDocument();
}

void MainWindowRobotHost::onCustomDeviceAssemblyCommitted(const QString& deviceBackendId)
{
	if (!m_mw || !m_mw->m_robotSimulation)
	{
		return;
	}
	if (cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr)
	{
		cloudsim::host::finalizeCustomDeviceLinkJointGraph(*host, deviceBackendId.toStdString());
	}
	m_mw->m_robotSimulation->refreshAxisControlTargets();
	if (RobotAxisControlWidget* axis = robotAxisControlPage())
	{
		axis->selectControlTarget(AxisControlTargetKind::CustomDevice, deviceBackendId);
	}
}

QVector<CustomDeviceMountRobotCandidate> MainWindowRobotHost::listMountRobotCandidates()
{
	QVector<CustomDeviceMountRobotCandidate> out;
	DocumentPage* page = m_mw ? m_mw->currentPage() : nullptr;
	IRobotDocumentHost* doc = document();
	if (!page || !doc)
	{
		return out;
	}
	const int n = page->robotKinematicInstanceCount();
	out.reserve(n);
	for (int i = 0; i < n; ++i)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (!page->robotPerLinkKinematicsForInstance(i, pl))
		{
			continue;
		}
		CustomDeviceMountRobotCandidate c;
		c.sceneBackendId = pl.sceneRootBackendId;
		const QFileInfo urdfInfo(pl.urdfAbsolutePath);
		c.label = urdfInfo.completeBaseName().isEmpty() ? pl.sceneRootBackendId : urdfInfo.completeBaseName();
		const RobotCoordinate::RobotCoordinateFrameSet& frames = page->robotCoordinateFramesForInstance(i);
		c.flangeLinkName = QString::fromStdString(frames.flangeLinkName);
		if (!c.flangeLinkName.isEmpty())
		{
			c.flangeBackendId =
				RobotSimulationMath::linkMeshBackendIdForInstance(doc, i, frames.flangeLinkName);
		}
		if (c.flangeBackendId.isEmpty())
		{
			const QHash<QString, QString>& links = pl.linkNameToBackendId;
			if (!links.isEmpty())
			{
				QStringList names = links.keys();
				std::sort(names.begin(), names.end());
				c.flangeBackendId = links.value(names.last());
				if (c.flangeLinkName.isEmpty())
				{
					c.flangeLinkName = names.last();
				}
			}
		}
		out.push_back(c);
	}
	return out;
}

QVector<CustomDeviceMountFrameCandidate> MainWindowRobotHost::listMountFrameCandidates(const QString& deviceBackendId)
{
	QVector<CustomDeviceMountFrameCandidate> out;
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host)
	{
		return out;
	}
	const std::string deviceId = deviceBackendId.toStdString();
	std::unordered_set<std::string> subtreeIds;
	if (!deviceId.empty() && host->backend().contains(deviceId))
	{
		std::queue<std::string> queue;
		queue.push(deviceId);
		subtreeIds.insert(deviceId);
		while (!queue.empty())
		{
			const std::string cur = queue.front();
			queue.pop();
			for (const std::string& child : host->backend().childrenOf(cur))
			{
				if (subtreeIds.insert(child).second)
				{
					queue.push(child);
				}
			}
		}
	}
	auto appendFrame = [&](const std::shared_ptr<BackendDataBase>& obj) {
		if (!obj || obj->className() != backend_type::kClassFrame)
		{
			return;
		}
		CustomDeviceMountFrameCandidate c;
		c.backendId = QString::fromStdString(obj->id());
		c.displayName = QString::fromStdString(obj->name().empty() ? obj->id() : obj->name());
		out.push_back(c);
	};
	for (const auto& obj : host->listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassFrame)
		{
			continue;
		}
		if (!subtreeIds.empty() && subtreeIds.count(obj->id()) == 0)
		{
			continue;
		}
		appendFrame(obj);
	}
	if (!out.isEmpty())
	{
		return out;
	}
	for (const auto& obj : host->listObjects())
	{
		appendFrame(obj);
	}
	return out;
}

bool MainWindowRobotHost::mountDeviceToRobot(const QString& deviceBackendId, const QString& robotSceneBackendId,
											 const QString& flangeLinkName, const QString& flangeBackendId,
											 const QString& mountFrameBackendId, QString* outError)
{
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host)
	{
		if (outError)
		{
			*outError = QStringLiteral("No active document.");
		}
		return false;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host->findObject(deviceBackendId.toStdString()));
	if (!device)
	{
		if (outError)
		{
			*outError = QStringLiteral("device not found");
		}
		return false;
	}
	(void)host->flushVisualSync();

	DocumentPage* page = m_mw ? m_mw->currentPage() : nullptr;
	RobotSimulationController* sim = m_mw ? m_mw->robotSimulation() : nullptr;
	const QVector<double>* jointAnglesForMount = nullptr;
	QVector<double> localJointAngles;
	const BackendMat4* mountTcpWorldForAlign = nullptr;
	BackendMat4 mountTcpWorld{};
	if (page && sim && page->hasRobotSimulationContext() && !robotSceneBackendId.isEmpty())
	{
		const int instIdx = page->robotInstanceIndexForSceneBackendId(robotSceneBackendId);
		if (instIdx >= 0)
		{
			const int offset = page->robotJointOffsetInAggregatedVector(instIdx);
			const int nj = page->robotRevoluteJointCountForInstance(instIdx);
			QVector<double> agg = sim->aggregatedJointAnglesRad();
			if (nj > 0)
			{
				localJointAngles = QVector<double>(nj, 0.0);
				if (agg.size() >= offset + nj)
				{
					for (int j = 0; j < nj; ++j)
					{
						localJointAngles[j] = agg[offset + j];
					}
				}
				else if (robotAxisControlPage() && robotAxisControlPage()->jointCount() >= nj)
				{
					const QVector<double> sliderQ = robotAxisControlPage()->jointAnglesRad();
					for (int j = 0; j < nj; ++j)
					{
						localJointAngles[j] = sliderQ[j];
					}
				}
				jointAnglesForMount = &localJointAngles;
				QVector<double> aggOut = agg;
				QString fkErr;
				(void)page->robot().applyJointAnglesRad(robotSceneBackendId, localJointAngles, &aggOut, &fkErr);
				page->reconcilePerLinkOuterBindFromScene(instIdx, localJointAngles);
				page->notifyRobotKinematicsAppliedToScene();

				const RobotCoordinate::RobotCoordinateFrameSet& frames =
					page->robotCoordinateFramesForInstance(instIdx);
				QString resolvedFlange;
				const bool tcpOk = computeMountTcpWorld(document(), osgView(), instIdx, flangeBackendId, flangeLinkName,
													  localJointAngles, frames, mountTcpWorld, &resolvedFlange);
				if (tcpOk)
				{
					mountTcpWorldForAlign = &mountTcpWorld;
				}
			}
		}
	}

	const bool mounted = cloudsim::host::mountCustomDeviceToFlange(*device, *host, robotSceneBackendId, flangeLinkName,
																   flangeBackendId, mountFrameBackendId,
																   BackendMat4::identity(), jointAnglesForMount,
																   mountTcpWorldForAlign, outError);
	if (mounted)
	{
		// pre-mount 已 FK；再 applyJointAnglesRad 会内嵌 notify 并用陈旧 OSG 覆盖刚挂好的设备
		if (page)
		{
			page->notifyRobotKinematicsAppliedToScene();
		}
		else
		{
			runFollowSolveAndSyncForCurrentDocument();
		}
		if (m_mw)
		{
			m_mw->updatePropertyPanel(deviceBackendId);
			m_mw->refreshRobotCoordinateFrameOverlays();
		}
	}
	return mounted;
}

bool MainWindowRobotHost::unmountDeviceFromRobot(const QString& deviceBackendId, QString* outError)
{
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host)
	{
		if (outError)
		{
			*outError = QStringLiteral("No active document.");
		}
		return false;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host->findObject(deviceBackendId.toStdString()));
	if (!device)
	{
		if (outError)
		{
			*outError = QStringLiteral("device not found");
		}
		return false;
	}
	return cloudsim::host::unmountCustomDeviceFromRobot(*device, *host, outError);
}

bool MainWindowRobotHost::isDeviceMountedToRobot(const QString& deviceBackendId) const
{
	cloudsim::host::DocumentHost* host = m_mw ? m_mw->currentDocumentHost() : nullptr;
	if (!host)
	{
		return false;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host->findObject(deviceBackendId.toStdString()));
	if (!device)
	{
		return false;
	}
	const auto mount = CustomDeviceRobotMountComponent::mountOf(*device);
	return mount && mount->enabled();
}
