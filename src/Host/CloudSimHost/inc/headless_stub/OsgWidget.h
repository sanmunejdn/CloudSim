#ifndef WIDGET_OSGWIDGET_H
#define WIDGET_OSGWIDGET_H

/// @file OsgWidget.h
/// @brief Headless 桩：满足 Host/PluginHost 编译，运行期无 OSG 视口

#include "cloudsim_host_global.h"

#include "../../OsgWidgetCore/inc/PickTypes.h"
#include "../../OsgWidgetCore/inc/RobotOsgUiTypes.h"
#include "CoreTypes.h"
#include "IRobotBackendPoseSink.h"

#include <QEvent>
#include <QList>
#include <QMetaType>
#include <QObject>
#include <QPoint>
#include <QString>
#include <QVector>
#include <QWidget>
#include <functional>
#include <string>
#include <vector>

#include <osg/Matrixd>
#include <osg/Vec3d>
#include <osg/Vec3f>
#include <osg/Vec4>

class BackendDataBase;
class MeshBackendData;
class PointCloudBackendData;

struct MeshCapturedPart;

Q_DECLARE_METATYPE(PickResult)

/// Web Headless：不继承 OsgScene，方法均为空操作
class CLOUDSIM_HOST_EXPORT OsgWidget : public QWidget, public IRobotBackendPoseSink
{
	Q_OBJECT
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

	explicit OsgWidget(QWidget* parent = nullptr) : QWidget(parent) {}

	QList<AnnotationSnapshot> annotationSnapshots() const { return {}; }
	void restoreAnnotations(const QList<AnnotationSnapshot>& snapshots) { (void)snapshots; }
	void setCameraFollowBackendId(std::string backendId) { (void)backendId; }
	std::string cameraFollowBackendId() const { return {}; }

	void setSelectedPosition(const osg::Vec3f& position) { (void)position; }
	void setSelectedRotationEulerDeg(const osg::Vec3f& eulerDeg) { (void)eulerDeg; }
	void setSelectedColor(float r, float g, float b, float a = 1.0f)
	{
		(void)r;
		(void)g;
		(void)b;
		(void)a;
	}

	bool importModelFile(const QString& filePath, QString* errorMessage = nullptr)
	{
		(void)filePath;
		(void)errorMessage;
		return false;
	}
	bool importPointCloudFile(const QString& filePath, QString* errorMessage = nullptr)
	{
		(void)filePath;
		(void)errorMessage;
		return false;
	}
	bool captureImportedPointCloudBackend(PointCloudBackendData& out, QString* errorMessage = nullptr)
	{
		(void)out;
		(void)errorMessage;
		return false;
	}
	bool capturePointCloudBackendFromScene(const std::string& backendId, PointCloudBackendData& out,
										   QString* errorMessage = nullptr)
	{
		(void)backendId;
		(void)out;
		(void)errorMessage;
		return false;
	}
	bool captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage = nullptr)
	{
		(void)out;
		(void)errorMessage;
		return false;
	}
	bool captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage = nullptr)
	{
		(void)outParts;
		(void)errorMessage;
		return false;
	}

	bool loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage = nullptr,
									   bool resetViewToHome = true)
	{
		(void)data;
		(void)errorMessage;
		(void)resetViewToHome;
		return false;
	}
	bool loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage = nullptr,
								 bool resetViewToHome = true, bool showWireOutline = true,
								 bool useSceneLighting = true)
	{
		(void)data;
		(void)errorMessage;
		(void)resetViewToHome;
		(void)showWireOutline;
		(void)useSceneLighting;
		return false;
	}
	bool loadBackendFromBackendData(const BackendDataBase& data, QString* errorMessage = nullptr,
									bool resetViewToHome = true, bool showWireOutline = true,
									bool useSceneLighting = true)
	{
		(void)data;
		(void)errorMessage;
		(void)resetViewToHome;
		(void)showWireOutline;
		(void)useSceneLighting;
		return false;
	}

	void clearStagingGeometry() {}
	void setStagingMeshPreview(const std::vector<float>& xyzTriangles, const osg::Vec4& rgba)
	{
		(void)xyzTriangles;
		(void)rgba;
	}

	void setSelectionActive(bool active) { (void)active; }
	void setPolylinePickMode(bool enabled) { (void)enabled; }
	bool polylinePickMode() const { return false; }
	void setMeshLinePickMode(bool enabled) { (void)enabled; }
	void setMeshFacePickMode(bool enabled) { (void)enabled; }
	void setLabelingClickPickMode(bool enabled, bool meshFace)
	{
		(void)enabled;
		(void)meshFace;
	}
	void setLabelingBrushPickMode(bool enabled, bool meshFace, float radiusPx)
	{
		(void)enabled;
		(void)meshFace;
		(void)radiusPx;
	}

	using OriginPlanePickedFn = std::function<void(bool ok, int planeIndex)>;
	void beginOriginPlaneSelection(OriginPlanePickedFn onFinished, float halfSizeMm = 60.f)
	{
		(void)onFinished;
		(void)halfSizeMm;
	}
	void cancelOriginPlaneSelection() {}

	struct SketchSupportExtraPlane
	{
		osg::Vec3d origin{0, 0, 0};
		osg::Vec3d axisX{1, 0, 0};
		osg::Vec3d axisY{0, 1, 0};
		osg::Vec3d normal{0, 0, 1};
		float halfMm = 40.f;
	};
	void setSketchSupportExtraPlanes(std::vector<SketchSupportExtraPlane> planes) { (void)planes; }
	void clearSketchSupportExtraPlanes() {}
	int resolveSketchSupportOriginIndex(int screenX, int screenY) const
	{
		(void)screenX;
		(void)screenY;
		return -1;
	}
	QPoint lastMousePos() const { return {}; }

	using SketchPlaneInputHandler = std::function<bool(QObject* watched, QEvent* event)>;
	void setSketchPlaneInputHandler(SketchPlaneInputHandler handler) { (void)handler; }
	void clearSketchPlaneInputHandler() {}

	void setSketchLineOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
							  const std::vector<std::size_t>& segmentEndExclusive,
							  const std::vector<osg::Vec4>& segmentColors,
							  const std::vector<float>& segmentWidthsPx = {})
	{
		(void)points;
		(void)segmentEndExclusive;
		(void)segmentColors;
		(void)segmentWidthsPx;
	}
	void clearSketchLineOverlay() {}

	bool intersectScreenWithPlaneMm(int screenX, int screenY, const osg::Vec3d& planeOrigin,
									const osg::Vec3d& planeNormal, osg::Vec3d& outHitWorldMm,
									QString* outError = nullptr) const
	{
		(void)screenX;
		(void)screenY;
		(void)planeOrigin;
		(void)planeNormal;
		(void)outHitWorldMm;
		(void)outError;
		return false;
	}

	void setOriginReferenceVisibility(bool originPoint, bool planeXY, bool planeXZ, bool planeYZ,
									  float halfSizeMm = 60.f)
	{
		(void)originPoint;
		(void)planeXY;
		(void)planeXZ;
		(void)planeYZ;
		(void)halfSizeMm;
	}

	void applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color)
	{
		(void)backendId;
		(void)color;
	}
	void setBackendObjectVisible(const std::string& backendId, bool visible)
	{
		(void)backendId;
		(void)visible;
	}
	void setBackendParent(const std::string& backendId, const std::string& parentBackendId)
	{
		(void)backendId;
		(void)parentBackendId;
	}
	void setBackendLogicalParent(const std::string& backendId, const std::string& parentBackendId)
	{
		(void)backendId;
		(void)parentBackendId;
	}
	void removeBackendObjectVisual(const std::string& backendId) { (void)backendId; }
	bool hasBackendObjectBranch(const std::string& backendId) const
	{
		(void)backendId;
		return false;
	}

	void syncSelectionFromBackend(const PointCloudBackendData& data) { (void)data; }
	void syncSelectionFromBackend(const MeshBackendData& data) { (void)data; }
	void syncSelectionForBackendId(const std::string& backendId) { (void)backendId; }
	void setPickVisualAlias(const std::string& logicalBackendId, const std::string& visualBackendId)
	{
		(void)logicalBackendId;
		(void)visualBackendId;
	}
	bool syncOuterPatFromBackend(const BackendDataBase& data)
	{
		(void)data;
		return false;
	}

	void requestRedraw() const {}
	void focusCameraOnBackend(const std::string& backendId) { (void)backendId; }
	void focusCameraOnAllVisibleBackends() {}
	void orientViewToPlane(const osg::Vec3d& focusMm, const osg::Vec3d& normal, const osg::Vec3d& upHint)
	{
		(void)focusMm;
		(void)normal;
		(void)upHint;
	}
	void setCameraViewDirection(const osg::Vec3d& eyeDirectionFromCenter, const osg::Vec3d& upHint)
	{
		(void)eyeDirectionFromCenter;
		(void)upHint;
	}
	bool getCameraViewDirectionInBackendModel(const std::string& backendIdUtf8, double outDirModel[3]) const
	{
		(void)backendIdUtf8;
		(void)outDirModel;
		return false;
	}

	void showMeshFaceHighlight(const std::vector<osg::Vec3f>& vertsWorld) { (void)vertsWorld; }
	void showMeshFaceHighlight(const osg::Vec3f& aWorld, const osg::Vec3f& bWorld, const osg::Vec3f& cWorld)
	{
		(void)aWorld;
		(void)bWorld;
		(void)cWorld;
	}
	void hideMeshElementHighlight() {}
	void showMeshFittedSurfacePreview(const std::vector<osg::Vec3f>& triangleVertsWorld)
	{
		(void)triangleVertsWorld;
	}
	void clearMeshFittedSurfacePreview() {}

	void setRawTrajectoryOverlay(const std::vector<RobotOsgUi::RawTrajectoryOverlayVertex>& points,
								 const RobotOsgUi::RawTrajectoryPreviewOptions& options = {})
	{
		(void)points;
		(void)options;
	}
	void setRawTrajectoryOverlayAxisComponents(bool showX, bool showY, bool showZ)
	{
		(void)showX;
		(void)showY;
		(void)showZ;
	}
	void setReachableWorkspaceOverlay(const RobotOsgUi::ReachableWorkspaceOverlay& overlay) { (void)overlay; }
	void clearReachableWorkspaceOverlay() {}

	void showMeshSectionPlane(const std::string& backendIdUtf8, const double originModelMm[3],
							  const double normalModel[3])
	{
		(void)backendIdUtf8;
		(void)originModelMm;
		(void)normalModel;
	}
	void beginMeshSectionPlaneEdit(const std::string& backendIdUtf8, const double originModelMm[3],
								   const double normalModel[3],
								   std::function<void(const double origin[3], const double normal[3])> onChanged)
	{
		(void)backendIdUtf8;
		(void)originModelMm;
		(void)normalModel;
		(void)onChanged;
	}
	void updateMeshSectionPlanePose(const double originModelMm[3], const double normalModel[3])
	{
		(void)originModelMm;
		(void)normalModel;
	}
	void endMeshSectionPlaneEdit() {}
	void hideMeshSectionPlane() {}
	void setMeshSectionPlanePreviewVisible(bool visible) { (void)visible; }

	bool getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const
	{
		(void)backendId;
		(void)outWorld;
		return false;
	}
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat)
	{
		(void)backendId;
		(void)worldMat;
	}
	bool getBackendRootWorldMatrix(const std::string& backendId, cloudsim::core::Mat4& outWorld) const override
	{
		(void)backendId;
		(void)outWorld;
		return false;
	}
	void setBackendRootWorldMatrixFromWorld(const std::string& backendId,
											const cloudsim::core::Mat4& worldColumnMajor) override
	{
		(void)backendId;
		(void)worldColumnMajor;
	}
	bool tryGetBackendModelCenterMm(const std::string& backendId, double& outCx, double& outCy,
									double& outCz) const override
	{
		(void)backendId;
		(void)outCx;
		(void)outCy;
		(void)outCz;
		return false;
	}
	bool tryGetBackendPointLocalToWorldMatrix(const std::string& backendId, double outColMajor16[16]) const
	{
		(void)backendId;
		(void)outColMajor16;
		return false;
	}
	std::string resolvePickScopeBackendId(const std::string& backendId) const { return backendId; }
	bool isTransformGizmoDragging() const { return false; }

signals:
	void polylinePickCommitted(QVector<float> polylineScreenXy, QVector<double> mvpMatrix, int viewportWidth,
							   int viewportHeight);
	void polylinePickCanceled();
	void meshPickCommitted(PickResult pick, int pickKind);
	void labelingClickCommitted(PickResult pick);
	void labelingBrushStroke(QVector<int> indices);
	void labelingBrushFinished();
	void labelingPickCanceled();
};

#endif // WIDGET_OSGWIDGET_H
