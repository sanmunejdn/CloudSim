#ifndef CLOUDSIMHOST_IVIEWPORTOVERLAY_H
#define CLOUDSIMHOST_IVIEWPORTOVERLAY_H

/// @file IViewportOverlay.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 视口标注/跟随/高亮/草图与轨迹 overlay（IOsgWidgetView 切面）

#include <QEvent>
#include <QList>
#include <QObject>
#include <QPoint>
#include <QString>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <osg/Vec3d>
#include <osg/Vec3f>
#include <osg/Vec4>

#include <RobotOsgUiTypes.h>

/// 标注快照、相机跟随、面高亮、草图/轨迹 overlay
class IViewportOverlay
{
public:
	struct AnnotationSnapshot
	{
		QString id;
		QString displayText;
		QString backendId;
		osg::Vec3f localCentered;
		osg::Vec3f worldAnchor{};
		bool hasWorldAnchor = false;
		bool visible = true;
	};

	struct SketchSupportExtraPlane
	{
		osg::Vec3d origin{0, 0, 0};
		osg::Vec3d axisX{1, 0, 0};
		osg::Vec3d axisY{0, 1, 0};
		osg::Vec3d normal{0, 0, 1};
		float halfMm = 40.f;
	};

	using OriginPlanePickedFn = std::function<void(bool ok, int planeIndex)>;
	using SketchPlaneInputHandler = std::function<bool(QObject* watched, QEvent* event)>;

	virtual ~IViewportOverlay() = default;

	virtual QList<AnnotationSnapshot> annotationSnapshots() const = 0;
	virtual void restoreAnnotations(const QList<AnnotationSnapshot>& snapshots) = 0;
	virtual void setCameraFollowBackendId(std::string backendId) = 0;
	virtual std::string cameraFollowBackendId() const = 0;

	virtual void setSketchLineOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
									  const std::vector<std::size_t>& segmentEndExclusive,
									  const std::vector<osg::Vec4>& segmentColors,
									  const std::vector<float>& segmentWidthsPx = {}) = 0;
	virtual void clearSketchLineOverlay() = 0;
	virtual void setSketchPlaneInputHandler(SketchPlaneInputHandler handler) = 0;
	virtual void clearSketchPlaneInputHandler() = 0;
	virtual void setSketchSupportExtraPlanes(std::vector<SketchSupportExtraPlane> planes) = 0;
	virtual void clearSketchSupportExtraPlanes() = 0;
	virtual void beginOriginPlaneSelection(OriginPlanePickedFn onFinished, float halfSizeMm = 60.f) = 0;
	virtual void cancelOriginPlaneSelection() = 0;
	virtual int resolveSketchSupportOriginIndex(int screenX, int screenY) const = 0;
	virtual QPoint lastMousePos() const = 0;
	virtual void setOriginReferenceVisibility(bool originPoint, bool planeXY, bool planeXZ, bool planeYZ,
												float halfSizeMm = 60.f) = 0;

	virtual void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld) = 0;
	virtual void hideMeshElementHighlight() = 0;
};

#endif // CLOUDSIMHOST_IVIEWPORTOVERLAY_H
