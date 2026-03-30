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
#include <osg/Node>
#include <osg/PositionAttitudeTransform>
#include <osg/Transform>
#include <osg/Viewport>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

namespace {

osg::BoundingSphere worldBoundOfPat(osg::Node* sceneRoot, osg::Node* objectsGroup, osg::PositionAttitudeTransform* pat)
{
	if (!sceneRoot || !objectsGroup || !pat)
	{
		return osg::BoundingSphere();
	}
	const osg::BoundingSphere loc = pat->getBound();
	if (!loc.valid())
	{
		return osg::BoundingSphere();
	}
	osg::NodePath path;
	path.push_back(sceneRoot);
	path.push_back(objectsGroup);
	path.push_back(pat);
	const osg::Matrix worldMat = osg::computeLocalToWorld(path);
	const osg::Vec4d lp(static_cast<double>(loc.center().x()), static_cast<double>(loc.center().y()),
		static_cast<double>(loc.center().z()), 1.0);
	const osg::Vec4d wp = worldMat * lp;
	const osg::Vec3d wc(wp.x(), wp.y(), wp.z());
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
	return osg::BoundingSphere(osg::Vec3(wc.x(), wc.y(), wc.z()), r);
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
		const osg::BoundingSphere w = worldBoundOfPat(m_root.get(), m_objectsGroup.get(), kv.second.get());
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
	if (!any || !merged.valid())
	{
		return;
	}
	osg::Vec3d center(merged.center());
	double radius = static_cast<double>(merged.radius());
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
	const double maxDist = std::max(radius * 50.0, 1e7);
	dist = std::min(dist, maxDist);

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

	updateCompassScale();
	requestRedraw();
}
