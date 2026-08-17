/// @file DocumentPage.cpp
/// @brief 文档页与仿真文档绑定

#include "DocumentPage.h"

#include "BackendDataManager.h"
#include "BackendSceneDocumentFacade.h"
#include "CoreTypes.h"
#include "EventHub.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "IRobotBackendPoseSink.h"
#include "MeshBackendData.h"
#include "OsgScene.h"
#include "OsgWidget.h"
#include "RobotPerLinkKinematicsSliceOsg.h"
#include "RobotProgramStore.h"
#include "RobotExternalAxes.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"
#include "ViewportToolBar.h"

#include <algorithm>

#include <QSet>
#include <QTabWidget>
#include <QUuid>
#include <memory>
#include <unordered_map>
#include <vector>

#include <osg/Group>
#include <osg/MatrixTransform>

namespace
{
cloudsim::core::Mat4 mat4FromOsg(const osg::Matrixd& m)
{
	cloudsim::core::Mat4 out{};
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out[static_cast<size_t>(c * 4 + r)] = m(r, c);
		}
	}
	return out;
}

osg::Matrixd osgFromMat4(const cloudsim::core::Mat4& columnMajor)
{
	osg::Matrixd out;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out(r, c) = columnMajor[static_cast<size_t>(c * 4 + r)];
		}
	}
	return out;
}

cloudsim::core::Mat4 identityMat4()
{
	cloudsim::core::Mat4 m{};
	m[0] = m[5] = m[10] = m[15] = 1.0;
	return m;
}
} // namespace

DocumentPage::DocumentPage(QTabWidget* parentTabs, cloudsim::core::EventHub& events)
	: DocumentHost(parentTabs, events, QStringLiteral("doc-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces)))
{
	setRobotUrdfImportContext(this);
	setPerLinkKinematicsHost(this);
	setPerLinkRobotStateAccessor(this);

	// 视口浮动按钮（无容器 QWidget，避免 Windows 透明层黑块）
	if (OsgWidget* ow = osgWidget())
	{
		auto* toolbar = new ViewportToolBar(ow);
		connect(toolbar, &ViewportToolBar::focusRequested, ow, &OsgWidget::onViewportFocusRequested);
		connect(toolbar, &ViewportToolBar::wireframeToggled, ow, &OsgWidget::setWireframeMode);
		connect(toolbar, &ViewportToolBar::screenshotRequested, ow, &OsgWidget::onViewportScreenshotRequested);
	}
}

void DocumentPage::setViewportToolBarDarkTheme(bool dark)
{
	if (QWidget* view = render().widget())
	{
		if (auto* toolbar = view->findChild<ViewportToolBar*>())
		{
			toolbar->setDarkTheme(dark);
		}
	}
}

void DocumentPage::setViewportToolBarUseChinese(bool useChinese)
{
	if (QWidget* view = render().widget())
	{
		if (auto* toolbar = view->findChild<ViewportToolBar*>())
		{
			toolbar->setUseChinese(useChinese);
		}
	}
}

void DocumentPage::syncViewportSidePanelToggleState(const bool leftVisible, const bool rightVisible)
{
	if (QWidget* view = render().widget())
	{
		if (auto* toolbar = view->findChild<ViewportToolBar*>())
		{
			toolbar->setSidePanelToggleState(leftVisible, rightVisible);
		}
	}
}

IRobotBackendPoseSink* DocumentPage::urdfImportScenePoseSink()
{
	return sceneFacade().poseSink();
}

cloudsim::host::PerLinkRobotStateSnapshot DocumentPage::extractPerLinkStateSnapshot(int instanceIndex) const
{
	cloudsim::host::PerLinkRobotStateSnapshot snap;
	snap.instanceIndex = instanceIndex;
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return snap;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	snap.urdfAbsolutePath = ri.urdfAbsolutePath;
	snap.linkNameToBackendId = ri.linkNameToBackendId;
	for (auto it = ri.fkMeshWorldT0.constBegin(); it != ri.fkMeshWorldT0.constEnd(); ++it)
	{
		snap.fkMeshWorldT0.insert(it.key(), it.value());
	}
	for (auto it = ri.outerWorldAtBindByBackendId.constBegin(); it != ri.outerWorldAtBindByBackendId.constEnd(); ++it)
	{
		snap.outerWorldAtBindByBackendId.insert(it.key(), it.value());
	}
	snap.basePlacementWorld = ri.basePlacementWorld;
	snap.meshVerticesInLinkFrame = ri.meshVerticesInLinkFrame;
	return snap;
}

void DocumentPage::applyPerLinkFkResult(const cloudsim::host::PerLinkRobotFkResult& result)
{
	// 简化实现：仅更新 basePlacementWorld（实际可扩展 updatedOuterWorlds 处理）
	if (!result.success)
	{
		return;
	}
	// 此处可根据需要更新 m_hierarchicalRobots 和 backend pose
	(void)result;
}

void DocumentPage::rebuildHierarchicalRobotAggregates()
{
	m_robotJointTransforms.clear();
	m_robotRevoluteJointNames.clear();
	m_robotJointLowerRad.clear();
	m_robotJointUpperRad.clear();
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		for (const QString& jn : ri.revoluteJointNamesUnprefixed)
		{
			m_robotRevoluteJointNames.append(ri.jointKeyPrefix + jn);
		}
		m_robotJointLowerRad += ri.jointLowerRad;
		m_robotJointUpperRad += ri.jointUpperRad;
		for (auto it = ri.jointTransformsByPrefixedKey.constBegin(); it != ri.jointTransformsByPrefixedKey.constEnd();
			 ++it)
		{
			m_robotJointTransforms.insert(it.key(), it.value());
		}
	}
	m_robotUrdfAbsolutePath =
		m_hierarchicalRobots.isEmpty() ? QString() : m_hierarchicalRobots.first().urdfAbsolutePath;
	m_robotSceneBackendId = m_hierarchicalRobots.isEmpty() ? QString() : m_hierarchicalRobots.first().sceneBackendId;
	rebuildPerLinkLegacyAggregates();
}

void DocumentPage::rebuildPerLinkLegacyAggregates()
{
	m_robotLinkNameToBackendId.clear();
	m_robotFkMeshWorldT0.clear();
	m_robotOuterWorldAtBind.clear();
	m_robotUrdfMeshVerticesInLinkFrame = false;
	int perLinkCount = 0;
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (!ri.perLinkBackends)
		{
			continue;
		}
		++perLinkCount;
		if (perLinkCount == 1)
		{
			m_robotLinkNameToBackendId = ri.linkNameToBackendId;
			m_robotFkMeshWorldT0 = ri.fkMeshWorldT0;
			m_robotOuterWorldAtBind = ri.outerWorldAtBindByBackendId;
			m_robotUrdfMeshVerticesInLinkFrame = ri.meshVerticesInLinkFrame;
			m_robotImportParentId = ri.perLinkImportKey;
		}
	}
}

void DocumentPage::appendHierarchicalRobotSimulationContext(
	const QString& urdfAbsolutePath, const QStringList& revoluteJointNamesUnprefixed,
	const QVector<double>& jointLowerRad, const QVector<double>& jointUpperRad,
	const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys, const QString& robotSceneBackendId,
	const QString& jointPrefixRootOverride)
{
	HierarchicalRobotInstance ri;
	ri.urdfAbsolutePath = urdfAbsolutePath;
	ri.sceneBackendId = robotSceneBackendId;
	ri.jointKeyPrefix = jointPrefixRootOverride.isEmpty() ? (robotSceneBackendId + QStringLiteral("::"))
														  : (jointPrefixRootOverride + QStringLiteral("::"));
	ri.revoluteJointNamesUnprefixed = revoluteJointNamesUnprefixed;
	ri.jointLowerRad = jointLowerRad;
	ri.jointUpperRad = jointUpperRad;
	ri.jointTransformsByPrefixedKey = jointTransformsPrefixedKeys;
	ri.basePlacementWorld = identityMat4();
	m_hierarchicalRobots.append(std::move(ri));
	rebuildHierarchicalRobotAggregates();
}

void DocumentPage::setRobotPerLinkKinematicsBinding(const QString& importKey,
													const QHash<QString, QString>& linkNameToBackendId,
													const QHash<QString, cloudsim::core::Mat4>& fkMeshWorldT0,
													const QHash<QString, cloudsim::core::Mat4>& outerWorldAtBindByBackendId,
													bool meshVerticesInLinkFrame)
{
	QString jointPrefix = importKey;
	if (jointPrefix.endsWith(QStringLiteral("_ctx")))
	{
		jointPrefix.chop(4);
	}
	jointPrefix += QStringLiteral("::");

	HierarchicalRobotInstance* target = nullptr;
	for (int i = m_hierarchicalRobots.size() - 1; i >= 0; --i)
	{
		if (m_hierarchicalRobots[i].jointKeyPrefix == jointPrefix)
		{
			target = &m_hierarchicalRobots[i];
			break;
		}
	}
	if (!target && !m_hierarchicalRobots.isEmpty())
	{
		target = &m_hierarchicalRobots.last();
	}
	if (!target)
	{
		return;
	}
	target->perLinkBackends = true;
	target->perLinkImportKey = importKey;
	target->linkNameToBackendId = linkNameToBackendId;
	target->fkMeshWorldT0 = fkMeshWorldT0;
	target->outerWorldAtBindByBackendId = outerWorldAtBindByBackendId;
	target->meshVerticesInLinkFrame = meshVerticesInLinkFrame;
	rebuildPerLinkLegacyAggregates();
}

void DocumentPage::setHierarchicalRobotSimulationContext(const QString& urdfAbsolutePath,
														 const QStringList& revoluteJointNames,
														 const QVector<double>& jointLowerRad,
														 const QVector<double>& jointUpperRad,
														 const QHash<QString, osg::MatrixTransform*>& jointTransforms,
														 const QString& robotBackendId, osg::Group* robotAssembly)
{
	(void)robotAssembly;
	clearRobotSimulationContext();
	const QString prefix = robotBackendId + QStringLiteral("::");
	QHash<QString, osg::MatrixTransform*> prefixed;
	prefixed.reserve(jointTransforms.size());
	for (auto it = jointTransforms.constBegin(); it != jointTransforms.constEnd(); ++it)
	{
		prefixed.insert(prefix + it.key(), it.value());
	}
	appendHierarchicalRobotSimulationContext(urdfAbsolutePath, revoluteJointNames, jointLowerRad, jointUpperRad,
											 prefixed, robotBackendId, QString());
}

osg::MatrixTransform* DocumentPage::robotJointMatrixTransform(const QString& jointName) const
{
	return m_robotJointTransforms.value(jointName);
}

bool DocumentPage::hasRobotJointLocalMatrix(const QString& jointName) const
{
	return m_robotJointTransforms.contains(jointName) && m_robotJointTransforms.value(jointName) != nullptr;
}

bool DocumentPage::robotJointWorldMatrix(const QString& jointName, cloudsim::core::Mat4& outWorld) const
{
	osg::MatrixTransform* jointMt = robotJointMatrixTransform(jointName);
	if (!jointMt)
	{
		return false;
	}
	osg::NodePathList paths = jointMt->getParentalNodePaths();
	if (paths.empty())
	{
		return false;
	}
	const osg::NodePath& path = paths.front();
	osg::Matrixd world = osg::computeLocalToWorld(path);
	if (path.empty() || path.back() != jointMt)
	{
		world = world * jointMt->getMatrix();
	}
	outWorld = mat4FromOsg(world);
	return true;
}

bool DocumentPage::applyRobotJointLocalMatrix(const QString& jointName, const cloudsim::core::Mat4& localColumnMajor)
{
	osg::MatrixTransform* jointMt = robotJointMatrixTransform(jointName);
	if (!jointMt)
	{
		return false;
	}
	jointMt->setMatrix(osgFromMat4(localColumnMajor));
	return true;
}

int DocumentPage::robotKinematicInstanceCount() const
{
	return m_hierarchicalRobots.size();
}

QString DocumentPage::robotSceneBackendIdForInstance(const int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
			   ? m_hierarchicalRobots[instanceIndex].sceneBackendId
			   : QString();
}

QString DocumentPage::robotFrameWorldReferenceBackendId(const int instanceIndex) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return QString();
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends)
	{
		return ri.sceneBackendId;
	}
	for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
	{
		const QString& linkBackendId = it.value();
		if (backendParentId().value(linkBackendId) == ri.sceneBackendId)
		{
			return linkBackendId;
		}
	}
	return ri.linkNameToBackendId.isEmpty() ? ri.sceneBackendId : ri.linkNameToBackendId.constBegin().value();
}

QString DocumentPage::robotDisplayLabelForInstance(const int instanceIndex) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return QString();
	}
	const QString id = m_hierarchicalRobots[instanceIndex].sceneBackendId;
	const QString name = const_cast<DocumentPage*>(this)->data().displayName(id);
	return name.isEmpty() ? id : name;
}

QStringList DocumentPage::robotRevoluteJointNamesForInstance(const int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
			   ? m_hierarchicalRobots[instanceIndex].revoluteJointNamesUnprefixed
			   : QStringList();
}

void DocumentPage::robotJointLimitsForInstance(const int instanceIndex, QVector<double>& lowerRad,
											   QVector<double>& upperRad) const
{
	lowerRad.clear();
	upperRad.clear();
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	lowerRad = ri.jointLowerRad;
	upperRad = ri.jointUpperRad;
}

int DocumentPage::robotJointOffsetInAggregatedVector(const int instanceIndex) const
{
	int offset = 0;
	for (int i = 0; i < instanceIndex && i < m_hierarchicalRobots.size(); ++i)
	{
		offset += m_hierarchicalRobots[i].revoluteJointNamesUnprefixed.size();
	}
	return offset;
}

int DocumentPage::robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const
{
	for (int i = 0; i < m_hierarchicalRobots.size(); ++i)
	{
		if (m_hierarchicalRobots[i].sceneBackendId == sceneBackendId)
		{
			return i;
		}
	}
	return -1;
}

QString DocumentPage::robotUrdfAbsolutePathForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
			   ? m_hierarchicalRobots[instanceIndex].urdfAbsolutePath
			   : QString();
}

int DocumentPage::robotRevoluteJointCountForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
			   ? m_hierarchicalRobots[instanceIndex].revoluteJointNamesUnprefixed.size()
			   : 0;
}

QString DocumentPage::robotJointKeyPrefixForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
			   ? m_hierarchicalRobots[instanceIndex].jointKeyPrefix
			   : QString();
}

bool DocumentPage::robotUsesPerLinkBackendsForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size() &&
		   m_hierarchicalRobots[instanceIndex].perLinkBackends;
}

bool DocumentPage::robotPerLinkKinematicsForInstance(int instanceIndex,
													 cloudsim::core::RobotPerLinkKinematicsSliceDto& out) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return false;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends || ri.linkNameToBackendId.isEmpty())
	{
		return false;
	}
	out.urdfAbsolutePath = ri.urdfAbsolutePath;
	out.sceneRootBackendId = ri.sceneBackendId;
	out.linkNameToBackendId = ri.linkNameToBackendId;
	out.fkMeshWorldT0 = ri.fkMeshWorldT0;
	out.outerWorldAtBindByBackendId = ri.outerWorldAtBindByBackendId;
	RobotExternal::composeBasePlacementWithExternalAxis(ri.basePlacementWorld.data(), ri.externalAxes,
														ri.externalAxisQ.empty()
															? std::vector<double>{ri.externalAxisQMm}
															: ri.externalAxisQ,
														out.robotBasePlacementWorld.data());
	out.meshVerticesInLinkFrame = ri.meshVerticesInLinkFrame;
	return true;
}

int DocumentPage::robotInstanceIndexForPerLinkBackend(const QString& backendId, bool* outIsSceneRoot) const
{
	if (outIsSceneRoot)
	{
		*outIsSceneRoot = false;
	}
	for (int i = 0; i < m_hierarchicalRobots.size(); ++i)
	{
		const HierarchicalRobotInstance& ri = m_hierarchicalRobots[i];
		if (ri.linkNameToBackendId.isEmpty())
		{
			continue;
		}
		if (ri.sceneBackendId == backendId)
		{
			if (outIsSceneRoot)
			{
				*outIsSceneRoot = true;
			}
			return i;
		}
		if (!ri.perLinkBackends)
		{
			continue;
		}
		for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
		{
			if (it.value() == backendId)
			{
				return i;
			}
		}
	}
	return -1;
}

void DocumentPage::setRobotBasePlacementWorldForInstance(const int instanceIndex,
														 const cloudsim::core::Mat4& placementWorld)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return;
	}
	m_hierarchicalRobots[instanceIndex].basePlacementWorld = placementWorld;
	rebuildPerLinkLegacyAggregates();
}

cloudsim::core::Mat4 DocumentPage::robotBasePlacementWorldForInstance(const int instanceIndex) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return cloudsim::core::PlanContextDto::identityMat4();
	}
	return m_hierarchicalRobots[instanceIndex].basePlacementWorld;
}

void DocumentPage::setRobotExternalAxisQMm(const int instanceIndex, const double qMm)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return;
	}
	HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (ri.externalAxisQ.size() != ri.externalAxes.axes.size())
	{
		ri.externalAxisQ.assign(ri.externalAxes.axes.size(), 0.0);
		for (size_t i = 0; i < ri.externalAxes.axes.size(); ++i)
		{
			ri.externalAxisQ[i] = ri.externalAxes.axes[i].home;
		}
	}
	bool wrote = false;
	for (size_t i = 0; i < ri.externalAxes.axes.size(); ++i)
	{
		const RobotExternal::RobotExternalAxisConfig& a = ri.externalAxes.axes[i];
		if (!a.enabled || a.attachment != RobotExternal::RobotExternalAttachment::RobotBase)
		{
			continue;
		}
		ri.externalAxisQ[i] = std::clamp(qMm, a.lower, a.upper);
		ri.externalAxisQMm = ri.externalAxisQ[i];
		wrote = true;
		break;
	}
	if (!wrote)
	{
		ri.externalAxisQMm = qMm;
	}
	rebuildPerLinkLegacyAggregates();
}

double DocumentPage::robotExternalAxisQMm(const int instanceIndex) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return 0.0;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	for (size_t i = 0; i < ri.externalAxes.axes.size() && i < ri.externalAxisQ.size(); ++i)
	{
		if (ri.externalAxes.axes[i].enabled &&
			ri.externalAxes.axes[i].attachment == RobotExternal::RobotExternalAttachment::RobotBase)
		{
			return ri.externalAxisQ[i];
		}
	}
	return ri.externalAxisQMm;
}

void DocumentPage::setRobotExternalAxisQ(const int instanceIndex, const std::vector<double>& qValues)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return;
	}
	HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	ri.externalAxisQ.assign(ri.externalAxes.axes.size(), 0.0);
	for (size_t i = 0; i < ri.externalAxes.axes.size(); ++i)
	{
		double q = ri.externalAxes.axes[i].home;
		if (i < qValues.size())
		{
			q = qValues[i];
		}
		ri.externalAxisQ[i] = std::clamp(q, ri.externalAxes.axes[i].lower, ri.externalAxes.axes[i].upper);
	}
	ri.externalAxisQMm = robotExternalAxisQMm(instanceIndex);
	rebuildPerLinkLegacyAggregates();
}

std::vector<double> DocumentPage::robotExternalAxisQ(const int instanceIndex) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return {};
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (ri.externalAxisQ.size() == ri.externalAxes.axes.size())
	{
		return ri.externalAxisQ;
	}
	std::vector<double> qs(ri.externalAxes.axes.size(), 0.0);
	for (size_t i = 0; i < ri.externalAxes.axes.size(); ++i)
	{
		qs[i] = ri.externalAxes.axes[i].home;
	}
	if (!ri.externalAxes.axes.empty())
	{
		for (size_t i = 0; i < qs.size(); ++i)
		{
			if (ri.externalAxes.axes[i].enabled &&
				ri.externalAxes.axes[i].attachment == RobotExternal::RobotExternalAttachment::RobotBase)
			{
				qs[i] = ri.externalAxisQMm;
				break;
			}
		}
	}
	return qs;
}

cloudsim::core::Mat4 DocumentPage::workpieceExternalBasePlacement(const int instanceIndex,
																  const QString& backendId) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || backendId.isEmpty())
	{
		return cloudsim::core::PlanContextDto::identityMat4();
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	const auto it = ri.workpieceBasePlacementWorld.constFind(backendId);
	if (it == ri.workpieceBasePlacementWorld.constEnd())
	{
		return cloudsim::core::PlanContextDto::identityMat4();
	}
	return it.value();
}

void DocumentPage::setWorkpieceExternalBasePlacement(const int instanceIndex, const QString& backendId,
													 const cloudsim::core::Mat4& w0)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || backendId.isEmpty())
	{
		return;
	}
	m_hierarchicalRobots[instanceIndex].workpieceBasePlacementWorld.insert(backendId, w0);
}

void DocumentPage::ensureWorkpieceExternalBasePlacement(const int instanceIndex, const QString& backendId,
														const cloudsim::core::Mat4& currentWorld)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || backendId.isEmpty())
	{
		return;
	}
	HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (ri.workpieceBasePlacementWorld.contains(backendId))
	{
		return;
	}
	ri.workpieceBasePlacementWorld.insert(backendId, currentWorld);
}

cloudsim::core::Mat4 DocumentPage::workpieceWorkingFrameOffset(const int instanceIndex,
															  const QString& boundBackendId) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || boundBackendId.isEmpty())
	{
		return cloudsim::core::PlanContextDto::identityMat4();
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	const auto it = ri.workpieceWorkingFrameOffsetByBackend.constFind(boundBackendId);
	if (it == ri.workpieceWorkingFrameOffsetByBackend.constEnd())
	{
		return cloudsim::core::PlanContextDto::identityMat4();
	}
	return it.value();
}

void DocumentPage::ensureWorkpieceWorkingFrameOffset(const int instanceIndex, const QString& boundBackendId,
													 const QString& workingFrameId,
													 const cloudsim::core::Mat4& workingWorld)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || boundBackendId.isEmpty())
	{
		return;
	}
	HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (ri.workpieceWorkingFrameOffsetByBackend.contains(boundBackendId))
	{
		return;
	}
	if (workingFrameId.isEmpty() || workingFrameId == boundBackendId)
	{
		ri.workpieceWorkingFrameOffsetByBackend.insert(boundBackendId, cloudsim::core::PlanContextDto::identityMat4());
		return;
	}
	const cloudsim::core::Mat4 w0 = workpieceExternalBasePlacement(instanceIndex, boundBackendId);
	double invW0[16];
	if (!RobotExternal::mat4InvertRigidColumnMajor(w0.data(), invW0))
	{
		ri.workpieceWorkingFrameOffsetByBackend.insert(boundBackendId, cloudsim::core::PlanContextDto::identityMat4());
		return;
	}
	cloudsim::core::Mat4 offset = cloudsim::core::PlanContextDto::identityMat4();
	RobotExternal::mat4MulColumnMajor16(invW0, workingWorld.data(), offset.data());
	ri.workpieceWorkingFrameOffsetByBackend.insert(boundBackendId, offset);
}

void DocumentPage::updateRobotLinkOuterBindFromWorld(const int instanceIndex, const QString& linkBackendId,
													 const cloudsim::core::Mat4& world)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || linkBackendId.isEmpty())
	{
		return;
	}
	HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends)
	{
		return;
	}
	ri.outerWorldAtBindByBackendId.insert(linkBackendId, world);
	rebuildPerLinkLegacyAggregates();
}

void DocumentPage::clearRobotSimulationContext()
{
	robotProgramStore().clear();
	// 清除传统成员
	m_robotImportParentId.clear();
	m_robotLinkNameToBackendId.clear();
	m_robotFkMeshWorldT0.clear();
	m_robotOuterWorldAtBind.clear();
	m_robotUrdfMeshVerticesInLinkFrame = false;

	m_hierarchicalRobots.clear();
	rebuildHierarchicalRobotAggregates();
}

void DocumentPage::clearContentForProjectOpen()
{
	if (OsgWidget* ow = osgWidget())
	{
		ow->clearImportedContent();
	}
	data().clear();
	clearRobotSimulationContext();
	backendSourcePath().clear();
	backendSourceType().clear();
	backendParentId().clear();
	render().clearAllAnnotations();
	invalidateFollowReverseIndex();
	clearFollowDirtyBackendIds();
	m_robotCollisionSettings = RobotCollision::Settings{};
}

void DocumentPage::clearRobotSimulationIfContains(const QString& removedBackendId)
{
	for (int i = 0; i < m_hierarchicalRobots.size(); ++i)
	{
		const HierarchicalRobotInstance& ri = m_hierarchicalRobots[i];
		if (ri.sceneBackendId == removedBackendId)
		{
			m_hierarchicalRobots.removeAt(i);
			rebuildHierarchicalRobotAggregates();
			return;
		}
		if (ri.perLinkBackends)
		{
			for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
			{
				if (it.value() == removedBackendId)
				{
					m_hierarchicalRobots.removeAt(i);
					rebuildHierarchicalRobotAggregates();
					return;
				}
			}
		}
	}

	if (!m_robotSceneBackendId.isEmpty() && removedBackendId == m_robotSceneBackendId)
	{
		clearRobotSimulationContext();
	}
}

bool DocumentPage::hasRobotSimulationContext() const
{
	return !m_hierarchicalRobots.isEmpty() || !m_robotUrdfAbsolutePath.isEmpty() ||
		   !m_robotLinkNameToBackendId.isEmpty();
}

bool DocumentPage::hasRobotKinematicsBind() const
{
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (ri.perLinkBackends && !ri.fkMeshWorldT0.isEmpty() && !ri.outerWorldAtBindByBackendId.isEmpty())
		{
			return true;
		}
	}
	return !m_robotJointTransforms.isEmpty() || (!m_robotFkMeshWorldT0.isEmpty() && !m_robotOuterWorldAtBind.isEmpty());
}

// 每连杆 URDF：link → mesh 后端 id（层级导入时为空）
const QHash<QString, QString>& DocumentPage::robotLinkNameToBackendId() const
{
	return m_robotLinkNameToBackendId;
}

QHash<QString, cloudsim::core::Mat4> DocumentPage::robotFkMeshWorldT0() const
{
	return m_robotFkMeshWorldT0;
}

QHash<QString, cloudsim::core::Mat4> DocumentPage::robotOuterWorldAtBind() const
{
	return m_robotOuterWorldAtBind;
}

bool DocumentPage::robotUrdfMeshVerticesInLinkFrame() const
{
	return m_robotUrdfMeshVerticesInLinkFrame;
}

QString DocumentPage::robotImportParentId() const
{
	return m_robotImportParentId;
}

QStringList DocumentPage::robotLinkBackendIds() const
{
	QStringList out;
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (!ri.perLinkBackends)
		{
			continue;
		}
		for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
		{
			out.append(it.value());
		}
	}
	return out;
}

QString DocumentPage::selectionRootBackendId(const QString& backendId) const
{
	if (backendId.isEmpty())
	{
		return backendId;
	}
	bool isSceneRoot = false;
	const int instanceIndex = robotInstanceIndexForPerLinkBackend(backendId, &isSceneRoot);
	if (instanceIndex < 0 || isSceneRoot)
	{
		return backendId;
	}
	return m_hierarchicalRobots[instanceIndex].sceneBackendId;
}

QString DocumentPage::robotGizmoAnchorBackendId(const QString& backendId) const
{
	if (backendId.isEmpty())
	{
		return backendId;
	}
	bool isSceneRoot = false;
	const int instanceIndex = robotInstanceIndexForPerLinkBackend(backendId, &isSceneRoot);
	if (instanceIndex < 0 || !robotUsesPerLinkBackendsForInstance(instanceIndex))
	{
		return backendId;
	}
	const QString anchor = robotFrameWorldReferenceBackendId(instanceIndex);
	return anchor.isEmpty() ? backendId : anchor;
}

bool DocumentPage::applyPerLinkRobotFkFromGizmoAnchor(const int instanceIndex, const QString& anchorLinkBackendId,
													  const QVector<double>& jointAnglesRad)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || anchorLinkBackendId.isEmpty())
	{
		return false;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends)
	{
		return false;
	}
	IRobotBackendPoseSink* osg = urdfImportScenePoseSink();
	if (!osg)
	{
		return false;
	}
	cloudsim::core::RobotPerLinkKinematicsSliceDto dto;
	if (!robotPerLinkKinematicsForInstance(instanceIndex, dto))
	{
		return false;
	}
	RobotPerLinkKinematicsSlice slice = RobotSceneKinematics::robotPerLinkSliceFromDto(dto);
	cloudsim::core::Mat4 anchorWorldMat;
	if (!osg->getBackendRootWorldMatrix(anchorLinkBackendId.toStdString(), anchorWorldMat))
	{
		return false;
	}
	osg::Matrixd anchorWorld = osgFromMat4(anchorWorldMat);
	osg::Matrixd placement;
	if (!RobotSceneKinematics::computeBasePlacementFromAnchorLinkWorld(slice, anchorLinkBackendId, jointAnglesRad,
																	   anchorWorld, placement))
	{
		return false;
	}
	// reverse 解出的是含外轴的 P_eff，存盘只留 P0
	cloudsim::core::Mat4 pEff = mat4FromOsg(placement);
	cloudsim::core::Mat4 p0{};
	const std::vector<double> qs = robotExternalAxisQ(instanceIndex);
	RobotExternal::unbakeBasePlacementExternalAxis(pEff.data(), ri.externalAxes, qs, p0.data());
	setRobotBasePlacementWorldForInstance(instanceIndex, p0);
	RobotExternal::composeBasePlacementWithExternalAxis(p0.data(), ri.externalAxes, qs, pEff.data());
	placement = osgFromMat4(pEff);
	slice.robotBasePlacementWorld = placement;
	if (!RobotSceneKinematics::applyPerLinkRobotBasePlacement(osg, backend(), slice, jointAnglesRad, placement))
	{
		return false;
	}
	const std::shared_ptr<BackendDataBase> rootData = findObject(ri.sceneBackendId.toStdString());
	if (rootData && rootData->hasPoseProperty())
	{
		osg::Vec3d trans;
		osg::Quat rot;
		osg::Vec3d scale;
		osg::Quat so;
		placement.decompose(trans, rot, scale, so);
		const osg::Vec3f euler = OsgScene::quatToEulerDeg(rot);
		rootData->setPose(BackendVec3{trans.x(), trans.y(), trans.z()});
		rootData->setRotation(BackendVec3{static_cast<double>(euler.x()), static_cast<double>(euler.y()),
										  static_cast<double>(euler.z())});
	}
	notifyRobotKinematicsAppliedToScene();
	return true;
}

void DocumentPage::reconcilePerLinkOuterBindFromScene(const int instanceIndex, const QVector<double>& jointAnglesRad)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends || ri.linkNameToBackendId.isEmpty() || ri.urdfAbsolutePath.isEmpty())
	{
		return;
	}
	IRobotBackendPoseSink* osg = urdfImportScenePoseSink();
	if (!osg)
	{
		return;
	}
	cloudsim::core::RobotPerLinkKinematicsSliceDto dto;
	if (!robotPerLinkKinematicsForInstance(instanceIndex, dto))
	{
		return;
	}
	RobotPerLinkKinematicsSlice slice = RobotSceneKinematics::robotPerLinkSliceFromDto(dto);
	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(slice.urdfAbsolutePath, jointAnglesRad, Tq, &fkErr,
												   slice.meshVerticesInLinkFrame))
	{
		return;
	}
	const osg::Matrixd Pinv = osg::Matrixd::inverse(slice.robotBasePlacementWorld);
	for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& linkBackendId = it.value();
		if (linkName.isEmpty() || linkBackendId.isEmpty())
		{
			continue;
		}
		const auto t0It = slice.fkMeshWorldT0.constFind(linkName);
		const auto tqIt = Tq.constFind(linkName);
		if (t0It == slice.fkMeshWorldT0.constEnd() || tqIt == Tq.constEnd())
		{
			continue;
		}
		osg::Matrixd world;
		cloudsim::core::Mat4 worldMat;
		if (!osg->getBackendRootWorldMatrix(linkBackendId.toStdString(), worldMat))
		{
			continue;
		}
		world = osgFromMat4(worldMat);
		const osg::Matrixd m0Bind = world * Pinv * osg::Matrixd::inverse(tqIt.value()) * t0It.value();
		updateRobotLinkOuterBindFromWorld(instanceIndex, linkBackendId, mat4FromOsg(m0Bind));
	}
}

QString DocumentPage::robotJointPrefixRoot() const
{
	if (m_hierarchicalRobots.isEmpty())
	{
		return QString();
	}
	QString p = m_hierarchicalRobots.first().jointKeyPrefix;
	if (p.endsWith(QStringLiteral("::")))
	{
		p.chop(2);
	}
	return p;
}

void DocumentPage::notifyRobotKinematicsAppliedToScene()
{
	if (suppressRobotFollowDirtyNotify())
	{
		return;
	}
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (!ri.sceneBackendId.isEmpty())
		{
			DocumentHost::markFollowAttachmentDirtyFromBackendMove(ri.sceneBackendId.toStdString());
		}
		// 跟随目标常是连杆 backend，须显式置脏（勿仅依赖 root→children 拓扑）
		for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
		{
			if (!it.value().isEmpty())
			{
				DocumentHost::markFollowAttachmentDirtyFromBackendMove(it.value().toStdString());
			}
		}
	}
	// 同步求解并清空脏集；per-frame hook 仅兜底 gizmo/属性/forced，避免 FK 后再解一遍
	cloudsim::core::FollowSolveContextDto ctx;
	ctx.skipAll = false;
	(void)data().runFollowSolveAndSync(ctx, nullptr);
}

void DocumentPage::setBackendVisible(const QString& backendId, bool visible)
{
	(void)data().setVisible(backendId, visible);
	sceneFacade().entity(backendId.toStdString()).setVisible(visible);
}

void DocumentPage::setBackendsVisible(const QStringList& backendIds, bool visible)
{
	std::vector<std::string> ids;
	ids.reserve(backendIds.size());
	for (const QString& id : backendIds)
	{
		(void)data().setVisible(id, visible);
		ids.push_back(id.toStdString());
	}
	sceneFacade().setBackendsVisible(ids, visible);
}

void DocumentPage::markFollowAttachmentDirtyFromBackendMove(const QString& seedBackendId)
{
	data().markFollowDirtyFromMove(seedBackendId);
}

const RobotCoordinate::RobotCoordinateFrameSet&
DocumentPage::robotCoordinateFramesForInstance(const int instanceIndex) const
{
	static const RobotCoordinate::RobotCoordinateFrameSet kEmpty{};
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return kEmpty;
	}
	return m_hierarchicalRobots[instanceIndex].coordinateFrames;
}

RobotCoordinate::RobotCoordinateFrameSet& DocumentPage::robotCoordinateFramesForInstance(const int instanceIndex)
{
	static RobotCoordinate::RobotCoordinateFrameSet kEmpty{};
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return kEmpty;
	}
	return m_hierarchicalRobots[instanceIndex].coordinateFrames;
}

const RobotCoordinate::RobotUserFrame* DocumentPage::robotActiveUserFrameForInstance(const int instanceIndex) const
{
	return RobotCoordinate::activeUserFrame(robotCoordinateFramesForInstance(instanceIndex));
}

const RobotExternal::RobotExternalAxisConfigSet&
DocumentPage::robotExternalAxesForInstance(const int instanceIndex) const
{
	static const RobotExternal::RobotExternalAxisConfigSet kEmpty{};
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return kEmpty;
	}
	return m_hierarchicalRobots[instanceIndex].externalAxes;
}

RobotExternal::RobotExternalAxisConfigSet& DocumentPage::robotExternalAxesForInstance(const int instanceIndex)
{
	static RobotExternal::RobotExternalAxisConfigSet kEmpty{};
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return kEmpty;
	}
	return m_hierarchicalRobots[instanceIndex].externalAxes;
}
