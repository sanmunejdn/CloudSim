#ifndef OSGWIDGETCORE_ORTHOPROJECTIONCALLBACK_H
#define OSGWIDGETCORE_ORTHOPROJECTIONCALLBACK_H

/// @file OrthoProjectionCallback.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 相机透视/正交切换：正交半高随 Trackball 视距换算，滚轮缩放与罗盘尺度沿用视距

#include <algorithm>
#include <cmath>

#include <osg/Camera>
#include <osg/Math>
#include <osg/NodeCallback>
#include <osg/Viewport>
#include <osg/observer_ptr>
#include <osgGA/TrackballManipulator>

/// 挂到主相机 update 回调：Viewer 每帧先跑相机回调再 cull，resize 等处重置的透视会被同帧覆盖
class OrthoProjectionCallback : public osg::NodeCallback
{
public:
	/// 与各处 setProjectionMatrixAsPerspective(30, ...) 一致，切换时 pivot 平面画幅不跳
	static constexpr double kPerspectiveFovyDeg = 30.0;

	explicit OrthoProjectionCallback(osgGA::TrackballManipulator* manipulator) : m_manipulator(manipulator) {}

	void setEnabled(bool on) { m_enabled = on; }
	bool isEnabled() const { return m_enabled; }

	void operator()(osg::Node* node, osg::NodeVisitor* nv) override
	{
		if (m_enabled)
		{
			applyOrtho(*static_cast<osg::Camera*>(node));
		}
		traverse(node, nv);
	}

	void applyOrtho(osg::Camera& camera) const
	{
		double dist = m_manipulator.valid() ? m_manipulator->getDistance() : 0.0;
		if (!(dist > 1e-6) || !std::isfinite(dist))
		{
			osg::Vec3d eye;
			osg::Vec3d center;
			osg::Vec3d up;
			camera.getViewMatrixAsLookAt(eye, center, up);
			dist = (std::max)(1.0, (eye - center).length());
		}
		const double halfH = dist * std::tan(osg::DegreesToRadians(kPerspectiveFovyDeg * 0.5));
		const double halfW = halfH * viewportAspect(camera);
		// 近远面由 OSG 按包围体自动收紧；负近面保证视点身后的物体不被裁掉
		camera.setProjectionMatrixAsOrtho(-halfW, halfW, -halfH, halfH, -kDepthRangeMm, kDepthRangeMm);
	}

	static void applyPerspective(osg::Camera& camera)
	{
		camera.setProjectionMatrixAsPerspective(kPerspectiveFovyDeg, viewportAspect(camera), 10.0, 1e8);
	}

private:
	static constexpr double kDepthRangeMm = 1e8;

	static double viewportAspect(const osg::Camera& camera)
	{
		const osg::Viewport* vp = camera.getViewport();
		if (vp && vp->width() > 0 && vp->height() > 0)
		{
			return static_cast<double>(vp->width()) / static_cast<double>(vp->height());
		}
		return 1.0;
	}

	osg::observer_ptr<osgGA::TrackballManipulator> m_manipulator;
	bool m_enabled = false;
};

#endif // OSGWIDGETCORE_ORTHOPROJECTIONCALLBACK_H
