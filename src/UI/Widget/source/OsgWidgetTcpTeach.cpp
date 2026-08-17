/// @file OsgWidgetTcpTeach.cpp
/// @brief TCP 拖动示教 OSG 侧

#include "../../OsgWidgetCore/inc/OsgCompassGeometry.h"
#include "../../OsgWidgetCore/inc/OsgCompassRender.h"
#include "OsgWidget.h"
#include "RobotTcpDragTeachOperation.h"

#include <algorithm>
#include <cmath>

#include <Adapters.h>
#include <RigidTransform.h>
#include <osg/BlendFunc>
#include <osg/Camera>
#include <osg/Depth>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/MatrixTransform>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

namespace
{
osg::Vec3d tcpTeachNodeOriginWorld(const osg::Node* leaf)
{
	if (!leaf)
	{
		return osg::Vec3d(0.0, 0.0, 0.0);
	}
	osg::NodePath path;
	for (const osg::Node* n = leaf; n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), const_cast<osg::Node*>(n));
	}
	return osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(path);
}

bool tcpTeachMountPatWorldMatrix(const osg::MatrixTransform* mountPat, osg::Matrixd& outWorld)
{
	if (!mountPat)
	{
		return false;
	}
	const osg::NodePathList paths = mountPat->getParentalNodePaths();
	if (paths.empty())
	{
		return false;
	}
	const osg::NodePath& path = paths.front();
	outWorld = osg::computeLocalToWorld(path);
	if (path.empty() || path.back() != mountPat)
	{
		outWorld = outWorld * mountPat->getMatrix();
	}
	return true;
}

} // namespace

bool OsgWidget::tcpTeachResolveBaseWorld(osg::Matrixd& outBaseWorld) const
{
	if (m_tcpTeachMountBackendId.empty())
	{
		return false;
	}
	if (m_tcpTeachResolveRobotBaseWorld)
	{
		return m_tcpTeachResolveRobotBaseWorld(outBaseWorld);
	}
	return getBackendRootWorldMatrix(m_tcpTeachMountBackendId, outBaseWorld);
}

bool OsgWidget::tcpTeachToolWorldMatrix(osg::Matrixd& outToolWorld) const
{
	// H8: per-link 法兰挂载时以场景图真值为准，避免 baseWorld 与 URDF 基座不一致导致 World 拖动错位
	if (m_tcpTeachUseFlangeLocalPlacement && m_tcpTeachMountPat.valid() &&
		tcpTeachMountPatWorldMatrix(m_tcpTeachMountPat.get(), outToolWorld))
	{
		return true;
	}
	osg::Matrixd baseWorld;
	if (!tcpTeachResolveBaseWorld(baseWorld))
	{
		return false;
	}
	outToolWorld = engine::osgMatrixFromRigidTransform(m_tcpTeachTargetInBase) * baseWorld;
	return true;
}

void OsgWidget::tcpTeachSetTargetFromToolWorld(const osg::Matrixd& toolWorldOsg)
{
	osg::Matrixd baseWorldOsg;
	if (!tcpTeachResolveBaseWorld(baseWorldOsg))
	{
		return;
	}
	const engine::RigidTransform toolW = engine::rigidTransformFromOsg(toolWorldOsg);
	const engine::RigidTransform baseW = engine::rigidTransformFromOsg(baseWorldOsg);
	// URDF/IK 基座用 composeColumn；勿用 OSG 行链 toolWorld*inv(base) 再转 Eigen（易致 X/Z 与 Y 符号不一致）
	m_tcpTeachTargetInBase = baseW.inverse().composeColumn(toolW);
}

void OsgWidget::beginTcpDragTeach(const std::string& mountBackendId, const engine::RigidTransform& T_base_target,
								  const float modelDiagonalMm, std::function<bool(osg::Matrixd&)> resolveRobotBaseWorld,
								  const osg::Matrixd* toolLocalOnFlange)
{
	endTcpDragTeach();
	m_tcpTeachActive = true;
	m_tcpTeachMountBackendId = mountBackendId;
	m_tcpTeachResolveRobotBaseWorld = std::move(resolveRobotBaseWorld);
	m_tcpTeachUseFlangeLocalPlacement = (toolLocalOnFlange != nullptr);
	m_tcpTeachToolLocalOnFlange.makeIdentity();
	if (toolLocalOnFlange)
	{
		m_tcpTeachToolLocalOnFlange = *toolLocalOnFlange;
	}
	m_tcpTeachTargetInBase = T_base_target;
	m_tcpTeachModelDiagonal = std::max(100.0f, modelDiagonalMm);
	m_tcpTeachGizmoRefDistance = -1.0;
	m_tcpTeachGizmoRefScale = 1.0;

	auto it = m_backendObjectRoots.find(mountBackendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		m_tcpTeachActive = false;
		m_tcpTeachMountBackendId.clear();
		m_tcpTeachResolveRobotBaseWorld = nullptr;
		m_tcpTeachUseFlangeLocalPlacement = false;
		return;
	}

	m_tcpTeachMountPat = new osg::MatrixTransform;
	m_tcpTeachMountPat->setName("TcpTeachMount");
	it->second->addChild(m_tcpTeachMountPat.get());

	m_tcpTeachWorldPat = new osg::MatrixTransform;
	m_tcpTeachWorldPat->setName("TcpTeachWorld");
	osg_compass::applyUnlitHighlitStateSet(m_tcpTeachWorldPat->getOrCreateStateSet());

	m_tcpTeachOverlayGroup = new osg::Group;
	m_tcpTeachOverlayGroup->setName("TcpTeachGizmoOverlay");
	osg_compass::applyUnlitHighlitStateSet(m_tcpTeachOverlayGroup->getOrCreateStateSet());
	m_tcpTeachCompassTransform = new osg::PositionAttitudeTransform;
	osg_compass::applyUnlitHighlitStateSet(m_tcpTeachCompassTransform->getOrCreateStateSet());
	m_tcpTeachCompassScaleTransform = new osg::MatrixTransform;
	osg_compass::applyUnlitHighlitStateSet(m_tcpTeachCompassScaleTransform->getOrCreateStateSet());
	osg_compass::TransformCompassBranches branches;
	m_tcpTeachCompassNode = osg_compass::buildTransformCompassNode(&branches);
	for (int i = 0; i < 3; ++i)
	{
		m_tcpTeachAxisBranch[i] = branches.axis[i];
		m_tcpTeachRingBranch[i] = branches.ring[i];
	}
	m_tcpTeachCompassScaleTransform->addChild(m_tcpTeachCompassNode.get());
	m_tcpTeachCompassTransform->addChild(m_tcpTeachCompassScaleTransform.get());
	m_tcpTeachOverlayGroup->addChild(m_tcpTeachCompassTransform.get());
	m_tcpTeachWorldPat->addChild(m_tcpTeachOverlayGroup.get());
	if (m_tcpTeachSceneOverlayGroup.valid())
	{
		m_tcpTeachSceneOverlayGroup->addChild(m_tcpTeachWorldPat.get());
	}

	updateTcpDragTeachFromTarget(T_base_target);
	syncTcpTeachWorldPatFromTarget();
	m_tcpTeachCompassTransform->setNodeMask(0xffffffffu);
	requestRedraw();
}

void OsgWidget::endTcpDragTeach()
{
	if (m_tcpTeachMountPat.valid() && !m_tcpTeachMountBackendId.empty())
	{
		auto it = m_backendObjectRoots.find(m_tcpTeachMountBackendId);
		if (it != m_backendObjectRoots.end() && it->second.valid())
		{
			it->second->removeChild(m_tcpTeachMountPat.get());
		}
	}
	m_tcpTeachActive = false;
	m_tcpTeachResolveRobotBaseWorld = nullptr;
	m_tcpTeachUseFlangeLocalPlacement = false;
	m_tcpTeachDragging = false;
	m_tcpTeachRotating = false;
	m_tcpTeachDragAxis = DragAxis::None;
	m_tcpTeachDragAxisWorld.set(0.0, 0.0, 0.0);
	m_tcpTeachDragScreenAxisUx = 1.0;
	m_tcpTeachDragScreenAxisUy = 0.0;
	m_tcpTeachDragMmPerPixel = 1.0;
	m_tcpTeachHoverAxis = DragAxis::None;
	if (m_tcpTeachWorldPat.valid() && m_tcpTeachSceneOverlayGroup.valid())
	{
		m_tcpTeachSceneOverlayGroup->removeChild(m_tcpTeachWorldPat.get());
	}
	m_tcpTeachMountPat = nullptr;
	m_tcpTeachWorldPat = nullptr;
	m_tcpTeachOverlayGroup = nullptr;
	m_tcpTeachCompassTransform = nullptr;
	m_tcpTeachCompassScaleTransform = nullptr;
	m_tcpTeachCompassNode = nullptr;
	for (int i = 0; i < 3; ++i)
	{
		m_tcpTeachAxisBranch[i] = nullptr;
		m_tcpTeachRingBranch[i] = nullptr;
	}
	m_tcpTeachMountBackendId.clear();
	requestRedraw();
}

void OsgWidget::updateTcpDragTeachToolLocalOnFlange(const osg::Matrixd& toolLocalOnFlange)
{
	if (!m_tcpTeachActive || !m_tcpTeachUseFlangeLocalPlacement || !m_tcpTeachMountPat.valid())
	{
		return;
	}
	m_tcpTeachToolLocalOnFlange = toolLocalOnFlange;
	m_tcpTeachMountPat->setMatrix(m_tcpTeachToolLocalOnFlange);
	// mount 仍跟法兰；overlay 跟目标，避免刷新工具系时把罗盘拧向 FK
	syncTcpTeachWorldPatFromTarget();
	requestRedraw();
}

void OsgWidget::syncTcpTeachWorldPatFromMount()
{
	if (!m_tcpTeachActive || !m_tcpTeachWorldPat.valid() || !m_tcpTeachMountPat.valid())
	{
		return;
	}
	osg::NodePath path;
	for (osg::Node* n = m_tcpTeachMountPat.get(); n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), n);
	}
	m_tcpTeachWorldPat->setMatrix(osg::computeLocalToWorld(path));
}

void OsgWidget::syncTcpTeachWorldPatFromTarget()
{
	if (!m_tcpTeachActive || !m_tcpTeachWorldPat.valid())
	{
		return;
	}
	osg::Matrixd baseWorld;
	if (!tcpTeachResolveBaseWorld(baseWorld))
	{
		return;
	}
	// 与 tcpTeachToolWorldMatrix 非法兰路径一致：toolWorld = T_base * P
	m_tcpTeachWorldPat->setMatrix(engine::osgMatrixFromRigidTransform(m_tcpTeachTargetInBase) * baseWorld);
	syncTcpTeachCompassAttitude();
	updateTcpTeachCompassScale();
}

void OsgWidget::updateTcpDragTeachFromTarget(const engine::RigidTransform& T_base_target, bool syncTargetInBase)
{
	// Per-link flange mount: scene pose follows link FK; keep dragged T_base_target for IK (DEVELOPER_GUIDE §13.1).
	if (syncTargetInBase || !m_tcpTeachUseFlangeLocalPlacement)
	{
		m_tcpTeachTargetInBase = T_base_target;
	}
	if (!m_tcpTeachActive || !m_tcpTeachMountPat.valid() || m_tcpTeachMountBackendId.empty())
	{
		return;
	}
	if (m_tcpTeachUseFlangeLocalPlacement)
	{
		m_tcpTeachMountPat->setMatrix(m_tcpTeachToolLocalOnFlange);
		// 跳点后网格已在 TCP，但 robotBasePlacement 常与外绑/场景不一致；FK×P 会把罗盘甩到默认位
		if (syncTargetInBase)
		{
			osg::Matrixd toolW;
			if (tcpTeachMountPatWorldMatrix(m_tcpTeachMountPat.get(), toolW))
			{
				tcpTeachSetTargetFromToolWorld(toolW);
			}
			else
			{
				m_tcpTeachTargetInBase = T_base_target;
			}
		}
		syncTcpTeachWorldPatFromTarget();
		requestRedraw();
		return;
	}
	osg::Matrixd sceneWorld;
	sceneWorld.makeIdentity();
	if (!getBackendRootWorldMatrix(m_tcpTeachMountBackendId, sceneWorld))
	{
		return;
	}
	osg::Matrixd baseWorld;
	if (!tcpTeachResolveBaseWorld(baseWorld))
	{
		baseWorld = sceneWorld;
	}
	const osg::Matrixd toolInBase = engine::osgMatrixFromRigidTransform(T_base_target);
	const osg::Matrixd toolWorld = toolInBase * baseWorld;
	// local = toolWorld * inv(mountWorld)，避免把基座系 T 误当成世界系（外轴 P≠I 时罗盘脱节）
	const osg::Matrixd localOnRoot = toolWorld * osg::Matrixd::inverse(sceneWorld);
	m_tcpTeachMountPat->setMatrix(localOnRoot);
	syncTcpTeachWorldPatFromMount();
	syncTcpTeachCompassAttitude();
	updateTcpTeachCompassScale();
	requestRedraw();
}

void OsgWidget::updateTcpTeachCompassHighlight(const DragAxis axis, const bool highlightRing)
{
	for (int i = 0; i < 3; ++i)
	{
		if (m_tcpTeachAxisBranch[i].valid())
		{
			m_tcpTeachAxisBranch[i]->setMatrix(osg::Matrix::identity());
		}
		if (m_tcpTeachRingBranch[i].valid())
		{
			m_tcpTeachRingBranch[i]->setMatrix(osg::Matrix::identity());
		}
	}
	if (axis == DragAxis::None)
	{
		return;
	}
	int idx = 0;
	if (axis == DragAxis::Y)
	{
		idx = 1;
	}
	else if (axis == DragAxis::Z)
	{
		idx = 2;
	}
	const osg::Matrix hi = osg::Matrix::scale(1.28f, 1.28f, 1.28f);
	if (highlightRing)
	{
		if (m_tcpTeachRingBranch[idx].valid())
		{
			m_tcpTeachRingBranch[idx]->setMatrix(hi);
		}
	}
	else if (m_tcpTeachAxisBranch[idx].valid())
	{
		m_tcpTeachAxisBranch[idx]->setMatrix(hi);
	}
}

void OsgWidget::updateTcpTeachCompassScale()
{
	if (!m_tcpTeachCompassTransform.valid() || !m_tcpTeachCompassScaleTransform.valid() || !m_viewer.valid() ||
		!m_viewer->getCamera() || !m_tcpTeachActive)
	{
		return;
	}
	osg::Vec3d eye, center, up;
	m_viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
	osg::Vec3f pivotF;
	computeTcpTeachPivotWorld(pivotF);
	const osg::Vec3d anchor(static_cast<double>(pivotF.x()), static_cast<double>(pivotF.y()),
							static_cast<double>(pivotF.z()));
	const double distance = (eye - anchor).length();
	if (m_tcpTeachGizmoRefDistance < 0.0 || m_tcpTeachGizmoRefDistance <= 1e-6)
	{
		m_tcpTeachGizmoRefDistance = std::max(1.0, distance);
		const double desiredAxisWorld =
			std::max(osg_compass::kCompassMinAxisWorld,
					 static_cast<double>(m_tcpTeachModelDiagonal) * osg_compass::kCompassModelDiagonalFactor);
		m_tcpTeachGizmoRefScale =
			std::max(0.4, std::min(800.0, desiredAxisWorld / static_cast<double>(osg_compass::kCompassAxisLength)));
	}
	double scale = m_tcpTeachGizmoRefScale * (distance / m_tcpTeachGizmoRefDistance);
	scale = std::max(0.3, std::min(1200.0, scale));
	m_tcpTeachCompassScaleTransform->setMatrix(osg::Matrixd::scale(scale, scale, scale));
}

bool OsgWidget::beginTcpTeachScreenDrag()
{
	m_tcpTeachDragScreenAxisUx = 1.0;
	m_tcpTeachDragScreenAxisUy = 0.0;
	m_tcpTeachDragMmPerPixel = 1.0;
	if (!m_tcpTeachActive || !m_viewer.valid() || !m_viewer->getCamera() || viewportWidth() <= 0 ||
		viewportHeight() <= 0)
	{
		return false;
	}
	osg::Vec3d axisW;
	if (!tcpTeachCompassUnitAxisWorld(m_tcpTeachDragAxis, axisW))
	{
		return false;
	}
	m_tcpTeachDragAxisWorld = axisW;

	float gizmoScale = 1.0f;
	if (m_tcpTeachCompassScaleTransform.valid())
	{
		const osg::Matrixd& sm = m_tcpTeachCompassScaleTransform->getMatrix();
		gizmoScale = static_cast<float>(std::max({std::abs(sm(0, 0)), std::abs(sm(1, 1)), std::abs(sm(2, 2))}));
		if (gizmoScale < 1e-6f)
		{
			gizmoScale = 1.0f;
		}
	}
	const float axisLenMm = osg_compass::kCompassAxisLength * gizmoScale;

	osg::Vec3f origin;
	computeTcpTeachPivotWorld(origin);
	const osg::Vec3f tipWorld(origin.x() + static_cast<float>(axisW.x() * static_cast<double>(axisLenMm)),
							  origin.y() + static_cast<float>(axisW.y() * static_cast<double>(axisLenMm)),
							  origin.z() + static_cast<float>(axisW.z() * static_cast<double>(axisLenMm)));

	osg::Camera* const camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	auto projectToScreen = [&](const osg::Vec3f& world, double& sx, double& sy)
	{
		const osg::Vec3d clip = osg::Vec3d(world) * mvp;
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
	};

	double ox = 0.0;
	double oy = 0.0;
	double tx = 0.0;
	double ty = 0.0;
	projectToScreen(origin, ox, oy);
	projectToScreen(tipWorld, tx, ty);
	const double vx = tx - ox;
	const double vy = ty - oy;
	const double lenPx = std::hypot(vx, vy);
	if (lenPx < 1e-3)
	{
		return false;
	}
	m_tcpTeachDragScreenAxisUx = vx / lenPx;
	m_tcpTeachDragScreenAxisUy = vy / lenPx;
	m_tcpTeachDragMmPerPixel = static_cast<double>(axisLenMm) / lenPx;
	return true;
}

double OsgWidget::tcpTeachScreenDragDsMm(const QPoint& curPos, const QPoint& lastPos) const
{
	const double dx = static_cast<double>(curPos.x()) - static_cast<double>(lastPos.x());
	const double dy = static_cast<double>(curPos.y()) - static_cast<double>(lastPos.y());
	const double dPx = dx * m_tcpTeachDragScreenAxisUx + dy * m_tcpTeachDragScreenAxisUy;
	return dPx * m_tcpTeachDragMmPerPixel;
}

bool OsgWidget::tcpTeachCompassUnitAxisWorld(const DragAxis axis, osg::Vec3d& outAxisWorld) const
{
	outAxisWorld.set(0.0, 0.0, 1.0);
	if (!m_tcpTeachActive || axis == DragAxis::None)
	{
		return false;
	}
	// 示教罗盘固定末端 TCP 轴；View 菜单 World/Local 只作用于物体 gizmo
	osg::Matrixd baseWorldOsg;
	if (!tcpTeachResolveBaseWorld(baseWorldOsg))
	{
		return false;
	}
	const engine::RigidTransform baseW = engine::rigidTransformFromOsg(baseWorldOsg);
	Eigen::Vector3d localAxis = Eigen::Vector3d::UnitZ();
	if (axis == DragAxis::X)
	{
		localAxis = Eigen::Vector3d::UnitX();
	}
	else if (axis == DragAxis::Y)
	{
		localAxis = Eigen::Vector3d::UnitY();
	}
	// 与目标姿态同源；勿用场景 FK（X/Z 可能与 Y 符号相反）
	const Eigen::Vector3d w = baseW.rotation() * m_tcpTeachTargetInBase.rotation() * localAxis;
	const double len = w.norm();
	if (len < 1e-12)
	{
		return false;
	}
	outAxisWorld.set(w.x() / len, w.y() / len, w.z() / len);
	return true;
}

void OsgWidget::computeTcpTeachPivotWorld(osg::Vec3f& outPivotWorld) const
{
	outPivotWorld.set(0.0f, 0.0f, 0.0f);
	if (m_tcpTeachWorldPat.valid())
	{
		osg::NodePath path;
		for (osg::Node* n = m_tcpTeachWorldPat.get(); n != nullptr;
			 n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
		{
			path.insert(path.begin(), n);
		}
		const osg::Vec3d w = osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(path);
		outPivotWorld.set(static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z()));
		return;
	}
	if (!m_tcpTeachMountPat.valid())
	{
		return;
	}
	osg::NodePath path;
	for (osg::Node* n = m_tcpTeachMountPat.get(); n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), n);
	}
	const osg::Vec3d w = osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(path);
	outPivotWorld.set(static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z()));
}

int OsgWidget::pickTcpTeachAxisAtScreenPos(const QPoint& mousePos, const bool preferRing, bool* outPickedRing) const
{
	if (outPickedRing)
	{
		*outPickedRing = false;
	}
	if (!m_tcpTeachActive || !m_viewer.valid() || !m_viewer->getCamera() || viewportWidth() <= 0 ||
		viewportHeight() <= 0 || !m_tcpTeachCompassNode.valid())
	{
		return kGizmoAxisNone;
	}
	const double mx = static_cast<double>(mousePos.x());
	const double my = static_cast<double>(mousePos.y());

	float gizmoScale = 1.0f;
	if (m_tcpTeachCompassScaleTransform.valid())
	{
		const osg::Matrixd& sm = m_tcpTeachCompassScaleTransform->getMatrix();
		gizmoScale = static_cast<float>(std::max({std::abs(sm(0, 0)), std::abs(sm(1, 1)), std::abs(sm(2, 2))}));
		if (gizmoScale < 1e-6f)
		{
			gizmoScale = 1.0f;
		}
	}
	const float axisLen = osg_compass::kCompassAxisLength * gizmoScale;
	const float ringRadius = 65.0f * osg_compass::kCompassGeomScale * gizmoScale;

	osg::Camera* camera = m_viewer->getCamera();
	const osg::Matrixd mvp = camera->getViewMatrix() * camera->getProjectionMatrix();
	osg::Vec3f origin;
	computeTcpTeachPivotWorld(origin);
	// 与 beginTcpTeachScreenDrag / tcpTeachCompassUnitAxisWorld 同一套世界系轴向，避免拾取与屏幕 ds 反向
	auto axisTipWorld = [&](const DragAxis axis) -> osg::Vec3f
	{
		osg::Vec3d axisW;
		if (!tcpTeachCompassUnitAxisWorld(axis, axisW))
		{
			return origin;
		}
		return origin + osg::Vec3f(static_cast<float>(axisW.x() * static_cast<double>(axisLen)),
								   static_cast<float>(axisW.y() * static_cast<double>(axisLen)),
								   static_cast<float>(axisW.z() * static_cast<double>(axisLen)));
	};

	auto projectToScreen = [&](const osg::Vec3f& world, double& sx, double& sy)
	{
		osg::Vec3d clip = osg::Vec3d(world) * mvp;
		sx = (clip.x() * 0.5 + 0.5) * static_cast<double>(viewportWidth());
		sy = (1.0 - (clip.y() * 0.5 + 0.5)) * static_cast<double>(viewportHeight());
	};

	double ox = 0, oy = 0, pxx = 0, pxy = 0, pyx = 0, pyy = 0, pzx = 0, pzy = 0;
	projectToScreen(origin, ox, oy);
	projectToScreen(axisTipWorld(DragAxis::X), pxx, pxy);
	projectToScreen(axisTipWorld(DragAxis::Y), pyx, pyy);
	projectToScreen(axisTipWorld(DragAxis::Z), pzx, pzy);

	auto distanceToSegment = [](double p0x, double p0y, double p1x, double p1y, double qx, double qy) -> double
	{
		const double vx = p1x - p0x;
		const double vy = p1y - p0y;
		const double wx = qx - p0x;
		const double wy = qy - p0y;
		const double len2 = vx * vx + vy * vy;
		if (len2 <= 1e-6)
		{
			return std::hypot(qx - p0x, qy - p0y);
		}
		const double t = std::max(0.0, std::min(1.0, (wx * vx + wy * vy) / len2));
		return std::hypot(qx - (p0x + vx * t), qy - (p0y + vy * t));
	};

	const double dx = distanceToSegment(ox, oy, pxx, pxy, mx, my);
	const double dy = distanceToSegment(ox, oy, pyx, pyy, mx, my);
	const double dz = distanceToSegment(ox, oy, pzx, pzy, mx, my);
	const double axisLenPx =
		std::max({std::hypot(pxx - ox, pxy - oy), std::hypot(pyx - ox, pyy - oy), std::hypot(pzx - ox, pzy - oy), 1.0});
	const double threshold = std::clamp(0.22 * axisLenPx, 14.0, 44.0);
	const double ringThreshold = std::clamp(0.14 * axisLenPx, 10.0, 36.0);

	osg::Vec3d axisDirW[3];
	const DragAxis dragAxes[3] = {DragAxis::X, DragAxis::Y, DragAxis::Z};
	for (int ai = 0; ai < 3; ++ai)
	{
		axisDirW[ai].set(0.0, 0.0, 1.0);
		(void)tcpTeachCompassUnitAxisWorld(dragAxes[ai], axisDirW[ai]);
	}
	auto ringPointWorld = [&](const int ringAxis, const float ca, const float sa) -> osg::Vec3f
	{
		const double rr = static_cast<double>(ringRadius);
		osg::Vec3d w(origin.x(), origin.y(), origin.z());
		if (ringAxis == kGizmoAxisX)
		{
			w += axisDirW[1] * (static_cast<double>(ca) * rr) + axisDirW[2] * (static_cast<double>(sa) * rr);
		}
		else if (ringAxis == kGizmoAxisY)
		{
			w += axisDirW[0] * (static_cast<double>(ca) * rr) + axisDirW[2] * (static_cast<double>(sa) * rr);
		}
		else
		{
			w += axisDirW[0] * (static_cast<double>(ca) * rr) + axisDirW[1] * (static_cast<double>(sa) * rr);
		}
		return osg::Vec3f(static_cast<float>(w.x()), static_cast<float>(w.y()), static_cast<float>(w.z()));
	};

	auto minDistanceToProjectedRing = [&](int axis) -> double
	{
		const int segments = 72;
		const float r = ringRadius;
		double minDist = 1e9;
		for (int i = 0; i < segments; ++i)
		{
			const float a0 = osg::PI * 2.0f * static_cast<float>(i) / static_cast<float>(segments);
			const float a1 = osg::PI * 2.0f * static_cast<float>(i + 1) / static_cast<float>(segments);
			osg::Vec3f w0, w1;
			if (axis == kGizmoAxisX)
			{
				w0 = ringPointWorld(kGizmoAxisX, std::cos(a0) * r, std::sin(a0) * r);
				w1 = ringPointWorld(kGizmoAxisX, std::cos(a1) * r, std::sin(a1) * r);
			}
			else if (axis == kGizmoAxisY)
			{
				w0 = ringPointWorld(kGizmoAxisY, std::cos(a0) * r, std::sin(a0) * r);
				w1 = ringPointWorld(kGizmoAxisY, std::cos(a1) * r, std::sin(a1) * r);
			}
			else
			{
				w0 = ringPointWorld(kGizmoAxisZ, std::cos(a0) * r, std::sin(a0) * r);
				w1 = ringPointWorld(kGizmoAxisZ, std::cos(a1) * r, std::sin(a1) * r);
			}
			double s0x = 0, s0y = 0, s1x = 0, s1y = 0;
			projectToScreen(w0, s0x, s0y);
			projectToScreen(w1, s1x, s1y);
			minDist = std::min(minDist, distanceToSegment(s0x, s0y, s1x, s1y, mx, my));
		}
		return minDist;
	};

	if (preferRing)
	{
		const double drx = minDistanceToProjectedRing(kGizmoAxisX);
		const double dry = minDistanceToProjectedRing(kGizmoAxisY);
		const double drz = minDistanceToProjectedRing(kGizmoAxisZ);
		double best = ringThreshold;
		int ringAxis = kGizmoAxisNone;
		if (drx < best)
		{
			best = drx;
			ringAxis = kGizmoAxisX;
		}
		if (dry < best)
		{
			best = dry;
			ringAxis = kGizmoAxisY;
		}
		if (drz < best)
		{
			best = drz;
			ringAxis = kGizmoAxisZ;
		}
		if (ringAxis != kGizmoAxisNone)
		{
			if (outPickedRing)
			{
				*outPickedRing = true;
			}
			return ringAxis;
		}
	}

	double minDist = threshold;
	int axis = kGizmoAxisNone;
	if (dx < minDist)
	{
		minDist = dx;
		axis = kGizmoAxisX;
	}
	if (dy < minDist)
	{
		minDist = dy;
		axis = kGizmoAxisY;
	}
	if (dz < minDist)
	{
		axis = kGizmoAxisZ;
	}
	return axis;
}

void OsgWidget::emitTcpDragTeachPoseChanged()
{
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	m_tcpTeachTargetInBase.translationMm(px, py, pz);
	m_tcpTeachTargetInBase.eulerDegForDisplay(ex, ey, ez);
	emit tcpDragTeachPoseChanged(px, py, pz, ex, ey, ez);
}

void OsgWidget::applyTcpTeachTranslationWorld(const int axisIndex, const double dsWorld)
{
	if (std::abs(dsWorld) < 1e-10)
	{
		return;
	}
	osg::Vec3d axisW = m_tcpTeachDragAxisWorld;
	if (axisW.length2() < 1e-18)
	{
		axisW = osg::Vec3d(0.0, 0.0, 1.0);
		if (axisIndex == 0)
		{
			axisW.set(1.0, 0.0, 0.0);
		}
		else if (axisIndex == 1)
		{
			axisW.set(0.0, 1.0, 0.0);
		}
	}
	osg::Matrixd baseWorldOsg;
	if (!tcpTeachResolveBaseWorld(baseWorldOsg))
	{
		return;
	}
	// 只改进基座系平移，避免 OSG 往返改姿态
	const engine::RigidTransform baseW = engine::rigidTransformFromOsg(baseWorldOsg);
	const Eigen::Vector3d axisEigen(axisW.x(), axisW.y(), axisW.z());
	Eigen::Vector3d t = m_tcpTeachTargetInBase.translationMm();
	t += baseW.rotation().conjugate() * (axisEigen * dsWorld);
	m_tcpTeachTargetInBase.setTranslationMm(t);
	syncTcpTeachWorldPatFromTarget();
}

void OsgWidget::applyTcpTeachTranslationBody(const int axisIndex, const double dsWorld)
{
	if (std::abs(dsWorld) < 1e-10)
	{
		return;
	}
	Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
	if (axisIndex == 0)
	{
		axis = Eigen::Vector3d::UnitX();
	}
	else if (axisIndex == 1)
	{
		axis = Eigen::Vector3d::UnitY();
	}
	const Eigen::Vector3d delta = m_tcpTeachTargetInBase.rotation() * (axis * dsWorld);
	Eigen::Vector3d t = m_tcpTeachTargetInBase.translationMm();
	t += delta;
	m_tcpTeachTargetInBase.setTranslationMm(t);
	syncTcpTeachWorldPatFromTarget();
}

void OsgWidget::applyTcpTeachRotationWorld(const int axisIndex, const double deltaRad)
{
	if (std::abs(deltaRad) < 1e-10)
	{
		return;
	}
	osg::Matrixd baseWorldOsg;
	if (!tcpTeachResolveBaseWorld(baseWorldOsg))
	{
		return;
	}
	const engine::RigidTransform baseW = engine::rigidTransformFromOsg(baseWorldOsg);
	engine::RigidTransform toolW = baseW.composeColumn(m_tcpTeachTargetInBase);
	Eigen::Vector3d axisW = Eigen::Vector3d::UnitZ();
	if (axisIndex == 0)
	{
		axisW = Eigen::Vector3d::UnitX();
	}
	else if (axisIndex == 1)
	{
		axisW = Eigen::Vector3d::UnitY();
	}
	const Eigen::AngleAxisd aa(deltaRad, axisW.normalized());
	toolW.setRotation(Eigen::Quaterniond(aa) * toolW.rotation());
	m_tcpTeachTargetInBase = baseW.inverse().composeColumn(toolW);
	syncTcpTeachWorldPatFromTarget();
}

void OsgWidget::syncTcpTeachCompassAttitude()
{
	if (!m_tcpTeachCompassTransform.valid())
	{
		return;
	}
	// 示教罗盘始终跟 TCP；物体 gizmo 的 World/Local 不在此切换
	m_tcpTeachCompassTransform->setAttitude(osg::Quat());
}

void OsgWidget::applyTcpTeachRotationBody(const int axisIndex, const double deltaRad)
{
	if (std::abs(deltaRad) < 1e-10)
	{
		return;
	}
	Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
	if (axisIndex == 0)
	{
		axis = Eigen::Vector3d::UnitX();
	}
	else if (axisIndex == 1)
	{
		axis = Eigen::Vector3d::UnitY();
	}
	const Eigen::AngleAxisd aa(deltaRad, axis.normalized());
	Eigen::Quaterniond qNew = m_tcpTeachTargetInBase.rotation() * Eigen::Quaterniond(aa);
	m_tcpTeachTargetInBase.setRotation(qNew);
	syncTcpTeachWorldPatFromTarget();
}
