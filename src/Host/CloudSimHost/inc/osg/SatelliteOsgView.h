#ifndef CLOUDSIMHOST_SATELLITEOSGVIEW_H
#define CLOUDSIMHOST_SATELLITEOSGVIEW_H

/// @file SatelliteOsgView.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 轻量卫星视口：共享场景根 + 独立 Trackball，默认不接拾取

#include "widget_global.h"

#include <QEvent>
#include <QShowEvent>
#include <QWidget>

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/GraphicsContext>
#include <osg/Group>
#include <osg/Matrixd>
#include <osg/Vec3d>
#include <osg/ref_ptr>

class OrthoProjectionCallback;
class OsgWidget;
class QWidgetViewer;
class GraphicsWindowQt1;

namespace osgViewer
{
class Viewer;
}
namespace osgGA
{
class TrackballManipulator;
}

/// 轻量卫星视口：共享场景根 + 独立 Trackball，默认不接拾取
class OSG_WIDGET_API SatelliteOsgView : public QWidget
{
	Q_OBJECT
public:
	enum class ViewPreset
	{
		Front,
		Top,
		Right,
		Iso,
		Reset
	};

	/// \a primary 提供共享场景根与可选 GL share
	explicit SatelliteOsgView(OsgWidget* primary, QWidget* parent = nullptr);
	~SatelliteOsgView() override;

	void setSharedSceneRoot(osg::Group* root);
	void setViewDirection(const osg::Vec3d& eyeDirectionFromCenter, const osg::Vec3d& upHint);
	void applyViewPreset(ViewPreset preset);
	osg::Matrixd cameraMatrix() const;
	void setCameraMatrix(const osg::Matrixd& matrix);
	void setOrthographicProjection(bool on);
	void requestRedraw();
	/// 清屏色 + 本地渐变跟 Primary 主题对齐
	void syncBackgroundFromPrimary();

signals:
	void activated();

protected:
	bool eventFilter(QObject* watched, QEvent* event) override;
	void showEvent(QShowEvent* event) override;

private:
	void initViewer(OsgWidget* primary);
	void applyHomeView();
	void ensureLocalBackground(bool dark);
	void updateLocalGradientColors(bool dark);
	void bindSharedSceneUnderLocalRoot(osg::Group* sharedRoot);
	void ensureLocalHud();
	void updateLocalHudViewport(int framebufferWidth, int framebufferHeight);

	OsgWidget* m_primary = nullptr;
	QWidgetViewer* m_glWidget = nullptr;
	osg::ref_ptr<osgViewer::Viewer> m_viewer;
	osg::ref_ptr<osgGA::TrackballManipulator> m_manipulator;
	osg::ref_ptr<OrthoProjectionCallback> m_orthoProjectionCallback;
	osg::ref_ptr<osg::GraphicsContext> m_graphicsWindow;
	/// 卫星本地根：渐变/HUD + 共享场景（主视口 HUD 绑 Primary GC，卫星需本地副本）
	osg::ref_ptr<osg::Group> m_localRoot;
	osg::ref_ptr<osg::Camera> m_bgCamera;
	osg::ref_ptr<osg::Geometry> m_bgGeom;
	osg::ref_ptr<osg::Camera> m_axesHudCamera;
	osg::ref_ptr<osg::Camera> m_viewCubeHudCamera;
	bool m_darkBackground = false;
};

#endif // CLOUDSIMHOST_SATELLITEOSGVIEW_H
