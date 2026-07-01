#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "OsgScene.h"

#include <algorithm>
#include <cmath>

#include <osg/BoundingSphere>
#include <osg/Camera>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/Transform>
#include <osg/Viewport>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

namespace {

osg::BoundingSphere worldBoundOfBackendRoot(osg::MatrixTransform* root)
{
	if (!root)
	{
		return osg::BoundingSphere();
	}
	root->dirtyBound();
	const osg::BoundingSphere loc = root->getBound();
	if (!loc.valid())
	{
		return osg::BoundingSphere();
	}
	osg::NodePath path;
	for (osg::Node* n = root; n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), n);
	}
	const osg::Matrix worldMat = osg::computeLocalToWorld(path);
	const osg::Vec4d centerWp4 = worldMat * osg::Vec4d(
		static_cast<double>(loc.center().x()),
		static_cast<double>(loc.center().y()),
		static_cast<double>(loc.center().z()), 1.0);
	const osg::Vec3d wc(centerWp4.x(), centerWp4.y(), centerWp4.z());
	const osg::Vec3d wcTranslate(worldMat(3, 0), worldMat(3, 1), worldMat(3, 2));
	// 内层去心时 outer 平移已含质心；世界坐标顶点（skip rebase）须用 loc.center() 变换
	const osg::Vec3d wcUse = (loc.center().length2() > 1e-6f) ? wc : wcTranslate;
	double maxS = 0.0;
	for (int c = 0; c < 3; ++c)
	{
		const osg::Vec3 col(worldMat(0, c), worldMat(1, c), worldMat(2, c));
		maxS = std::max(maxS, static_cast<double>(col.length()));
	}
	if (maxS < 1e-8 || maxS != maxS)
	{
		maxS = 1.0;
	}
	const float r = static_cast<float>(static_cast<double>(loc.radius()) * maxS);
	return osg::BoundingSphere(osg::Vec3(static_cast<float>(wcUse.x()), static_cast<float>(wcUse.y()),
		static_cast<float>(wcUse.z())), r);
}

} // namespace

void OsgScene::focusCameraOnBackend(const std::string& backendId)
{
	if (backendId.empty() || !m_trackballManipulator.valid() || !m_viewer.valid() || !m_viewer->getCamera())
	{
		return;
	}
	osg::BoundingSphere merged;
	bool any = false;
	for (const auto& kv : m_backendObjectRoots)
	{
		if (!kv.second.valid())
		{
			continue;
		}
		if (!isBackendDescendantOf(kv.first, backendId))
		{
			continue;
		}
		const osg::BoundingSphere w = worldBoundOfBackendRoot(kv.second.get());
		if (!w.valid())
		{
			continue;
		}
		if (!any)
		{
			merged = w;
			any = true;
		}
		else
		{
			merged.expandBy(w);
		}
	}
	// getBound may not be ready immediately after the node is added to the scene.
	// Compute the world centre from the outer MT's current matrix (which is updated on move)
	// rather than the stale file-coordinate centre in m_backendModelCenters.
	if (!any || !merged.valid())
	{
		auto it = m_backendObjectRoots.find(backendId);
		if (it != m_backendObjectRoots.end() && it->second.valid())
		{
			osg::NodePath path;
			for (osg::Node* n = it->second.get(); n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
			{
				path.insert(path.begin(), n);
			}
			const osg::Matrix worldMat = osg::computeLocalToWorld(path);
			const osg::Vec3d wc(worldMat(3, 0), worldMat(3, 1), worldMat(3, 2));
			merged = osg::BoundingSphere(osg::Vec3(wc.x(), wc.y(), wc.z()), 5000.f);
			any = merged.valid();
		}
		else
		{
			const auto cIt = m_backendModelCenters.find(backendId);
			if (cIt != m_backendModelCenters.end())
			{
				const osg::Vec3f& mc = cIt->second;
				merged = osg::BoundingSphere(osg::Vec3(mc.x(), mc.y(), mc.z()), 5000.f);
				any = merged.valid();
			}
		}
	}
	if (!any || !merged.valid())
	{
		return;
	}
	osg::Vec3d center(merged.center());
	double radius = static_cast<double>(merged.radius());
	// URDF 连杆容器 setCullingActive(false) 时 OSG 包围球常异常偏大；Trackball::home() 也会按「整场景」拟合，
	// 相机会被拉到极远，其它后端几何落在视锥外或远裁剪外，表现为「导入后全黑，删掉机器人才看见」
	static constexpr double kMaxFocusRadius = 5.0e5;
	static constexpr double kMaxEyeDistance = 2.0e6;
	if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z()) || !std::isfinite(radius))
	{
		return;
	}
	if (radius > kMaxFocusRadius)
	{
		radius = kMaxFocusRadius;
	}
	if (radius < 1e-3)
	{
		radius = 1.0;
	}
	osg::Camera* cam = m_viewer->getCamera();
	static constexpr double kFocusFovyDeg = 30.0;
	static constexpr double kNearPlane = 0.1;
	double aspect = 1.0;
	const osg::Viewport* vp = cam->getViewport();
	if (vp && vp->width() > 0 && vp->height() > 0)
	{
		aspect = static_cast<double>(vp->width()) / static_cast<double>(vp->height());
	}
	else if (viewportWidth() > 0 && viewportHeight() > 0)
	{
		const double dpr = devicePixelRatio();
		const double vw = std::max(1.0, static_cast<double>(viewportWidth()) * dpr);
		const double vh = std::max(1.0, static_cast<double>(viewportHeight()) * dpr);
		aspect = vw / vh;
	}
	const double fovYRad = osg::DegreesToRadians(kFocusFovyDeg);
	const double tanHalfY = std::tan(fovYRad * 0.5);
	const double fovXRad = 2.0 * std::atan(std::tan(fovYRad * 0.5) * aspect);
	const double tanHalfX = std::tan(fovXRad * 0.5);
	double distY = radius / std::max(1e-8, tanHalfY);
	double distX = radius / std::max(1e-8, tanHalfX);
	double dist = std::max(distY, distX) * 1.12;
	dist = std::max(dist, kNearPlane * 2.0 + radius);
	dist = std::min(dist, kMaxEyeDistance);

	osg::Vec3d dir(1.0, 1.0, 1.05);
	dir.normalize();
	osg::Vec3d eye = center + dir * dist;
	osg::Vec3d up(0.0, 0.0, 1.0);
	osg::Vec3d forward = center - eye;
	if (forward.length2() < 1e-12)
	{
		forward = -dir;
	}
	forward.normalize();
	if (std::abs(forward * up) > 0.95)
	{
		up = osg::Vec3d(0.0, 1.0, 0.0);
	}
	m_trackballManipulator->setTransformation(eye, center, up);

	// 世界单位为毫米时：需要足够大的 zFar；同时 zNear 过小 + zFar 极大 → 深度缓冲精度崩溃，表现为「全黑」
	{
		const double surfaceDist = std::max(1.0, dist - radius);
		double zNearProj = std::max(10.0, surfaceDist * 0.02);
		zNearProj = std::min(zNearProj, std::max(10.0, dist * 0.45));
		double zFarNeeded = std::min(1e9, std::max(1e4, (dist + radius * 8.0) * 1.25));
		if (zNearProj >= zFarNeeded * 0.5)
		{
			zNearProj = std::max(1.0, zFarNeeded * 1e-4);
		}
		cam->setProjectionMatrixAsPerspective(kFocusFovyDeg, aspect, zNearProj, zFarNeeded);
	}

	updateCompassScale();
	requestRedraw();
}

void OsgScene::focusCameraOnAllVisibleBackends()
{
	if (!m_root.valid() || !m_trackballManipulator.valid()
		|| !m_viewer.valid() || !m_viewer->getCamera())
	{
		return;
	}

	// 计算场景根节点的包围球（包含所有内容：后端对象、机器人、轨迹等）
	m_root->dirtyBound();
	const osg::BoundingSphere bs = m_root->getBound();

	if (!bs.valid())
	{
		setCameraViewPreset(CameraViewPreset::Iso);
		return;
	}

	osg::Vec3d center(bs.center());
	double radius = static_cast<double>(bs.radius());

	static constexpr double kMaxFocusRadius = 5.0e5;
	static constexpr double kMaxEyeDistance = 2.0e6;
	if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z()) || !std::isfinite(radius))
	{
		return;
	}
	if (radius > kMaxFocusRadius)
	{
		radius = kMaxFocusRadius;
	}
	if (radius < 1e-3)
	{
		radius = 1.0;
	}

	osg::Camera* cam = m_viewer->getCamera();
	static constexpr double kFocusFovyDeg = 30.0;
	static constexpr double kNearPlane = 0.1;
	double aspect = 1.0;
	const osg::Viewport* vp = cam->getViewport();
	if (vp && vp->width() > 0 && vp->height() > 0)
	{
		aspect = static_cast<double>(vp->width()) / static_cast<double>(vp->height());
	}
	else if (viewportWidth() > 0 && viewportHeight() > 0)
	{
		const double dpr = devicePixelRatio();
		const double vw = std::max(1.0, static_cast<double>(viewportWidth()) * dpr);
		const double vh = std::max(1.0, static_cast<double>(viewportHeight()) * dpr);
		aspect = vw / vh;
	}

	const double fovYRad = osg::DegreesToRadians(kFocusFovyDeg);
	const double tanHalfY = std::tan(fovYRad * 0.5);
	const double fovXRad = 2.0 * std::atan(std::tan(fovYRad * 0.5) * aspect);
	const double tanHalfX = std::tan(fovXRad * 0.5);
	double distY = radius / std::max(1e-8, tanHalfY);
	double distX = radius / std::max(1e-8, tanHalfX);
	double dist = std::max(distY, distX) * 1.12;
	dist = std::max(dist, kNearPlane * 2.0 + radius);
	dist = std::min(dist, kMaxEyeDistance);

	// 等轴测方向
	osg::Vec3d dir(1.0, 1.0, 1.05);
	dir.normalize();
	osg::Vec3d eye = center + dir * dist;
	osg::Vec3d up(0.0, 0.0, 1.0);
	osg::Vec3d forward = center - eye;
	if (forward.length2() < 1e-12)
	{
		forward = -dir;
	}
	forward.normalize();
	if (std::abs(forward * up) > 0.95)
	{
		up = osg::Vec3d(0.0, 1.0, 0.0);
	}
	m_trackballManipulator->setTransformation(eye, center, up);

	// 设置投影矩阵
	{
		const double surfaceDist = std::max(1.0, dist - radius);
		double zNearProj = std::max(10.0, surfaceDist * 0.02);
		zNearProj = std::min(zNearProj, std::max(10.0, dist * 0.45));
		double zFarNeeded = std::min(1e9, std::max(1e4, (dist + radius * 8.0) * 1.25));
		if (zNearProj >= zFarNeeded * 0.5)
		{
			zNearProj = std::max(1.0, zFarNeeded * 1e-4);
		}
		cam->setProjectionMatrixAsPerspective(kFocusFovyDeg, aspect, zNearProj, zFarNeeded);
	}

	updateCompassScale();
	requestRedraw();
}

void OsgScene::setCameraViewPreset(CameraViewPreset preset)
{
	if (!m_trackballManipulator.valid() || !m_viewer.valid() || !m_viewer->getCamera())
	{
		return;
	}

	osg::Vec3d eye;
	osg::Vec3d center;
	osg::Vec3d upCurrent;
	m_trackballManipulator->getTransformation(eye, center, upCurrent);

	osg::Vec3d offset = eye - center;
	double dist = offset.length();
	if (dist < 1e-3 || !std::isfinite(dist))
	{
		dist = 3000.0;
	}

	osg::Vec3d presetDir(1.0, 1.0, 1.05);
	osg::Vec3d up(0.0, 0.0, 1.0);

	switch (preset)
	{
	case CameraViewPreset::Top:
		presetDir.set(0.0, 0.0, 1.0);
		up.set(0.0, 1.0, 0.0);
		break;
	case CameraViewPreset::Bottom:
		presetDir.set(0.0, 0.0, -1.0);
		up.set(0.0, 1.0, 0.0);
		break;
	case CameraViewPreset::Front:
		presetDir.set(0.0, 1.0, 0.0);
		up.set(0.0, 0.0, 1.0);
		break;
	case CameraViewPreset::Back:
		presetDir.set(0.0, -1.0, 0.0);
		up.set(0.0, 0.0, 1.0);
		break;
	case CameraViewPreset::Right:
		presetDir.set(1.0, 0.0, 0.0);
		up.set(0.0, 0.0, 1.0);
		break;
	case CameraViewPreset::Left:
		presetDir.set(-1.0, 0.0, 0.0);
		up.set(0.0, 0.0, 1.0);
		break;
	case CameraViewPreset::Iso:
	default:
		presetDir.set(1.0, 1.0, 1.05);
		presetDir.normalize();
		up.set(0.0, 0.0, 1.0);
		break;
	}

	const osg::Vec3d forward = -presetDir;
	if (std::abs(forward * up) > 0.95)
	{
		up.set(0.0, 1.0, 0.0);
	}

	const osg::Vec3d newEye = center + presetDir * dist;
	m_trackballManipulator->setTransformation(newEye, center, up);
	updateCompassScale();
	requestRedraw();
}

void OsgScene::setCameraViewDirection(const osg::Vec3d& eyeDirectionFromCenter, const osg::Vec3d& upHint)
{
	if (!m_trackballManipulator.valid() || !m_viewer.valid() || !m_viewer->getCamera())
	{
		return;
	}

	osg::Vec3d presetDir = eyeDirectionFromCenter;
	if (presetDir.length2() < 1e-12)
	{
		return;
	}
	presetDir.normalize();

	osg::Vec3d eye;
	osg::Vec3d center;
	osg::Vec3d upCurrent;
	m_trackballManipulator->getTransformation(eye, center, upCurrent);

	osg::Vec3d offset = eye - center;
	double dist = offset.length();
	if (dist < 1e-3 || !std::isfinite(dist))
	{
		dist = 3000.0;
	}

	osg::Vec3d up = upHint;
	if (up.length2() < 1e-12)
	{
		up.set(0.0, 0.0, 1.0);
		if (std::abs(presetDir.z()) > 0.9)
		{
			up.set(0.0, 1.0, 0.0);
		}
	}
	up.normalize();

	const osg::Vec3d forward = -presetDir;
	if (std::abs(forward * up) > 0.95)
	{
		up.set(0.0, 1.0, 0.0);
	}

	const osg::Vec3d newEye = center + presetDir * dist;
	m_trackballManipulator->setTransformation(newEye, center, up);
	updateCompassScale();
	requestRedraw();
}
