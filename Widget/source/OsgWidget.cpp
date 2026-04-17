#include "OsgWidget.h"

#include "BackendVisualRegistry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>

#include <QFile>
#include <QFileInfo>
#include <QMouseEvent>
#include <QRegExp>
#include <QStringList>
#include <QTextStream>
#include <QVBoxLayout>

#include <osg/Array>
#include <osg/Camera>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/PrimitiveSet>
#include <osg/Group>
#include <osg/Light>
#include <osg/LightSource>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/Point>
#include <osg/PolygonOffset>
#include <osg/PositionAttitudeTransform>
#include <osg/ShapeDrawable>
#include <osg/Shape>
#include <osg/Transform>
#include <osg/StateSet>
#include <osg/StateAttribute>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Vec4>
#include <osg/NodeVisitor>
#include <osg/NodeCallback>
#include <osg/AutoTransform>
#include <osg/BoundingSphere>
#include <osgDB/Options>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgGA/EventQueue>
#include <osgGA/GUIEventAdapter>
#include <osgGA/StateSetManipulator>
#include <osgGA/TrackballManipulator>
#include <osgText/Text>
#include <osgViewer/GraphicsWindow>

#include "MeshBackendData.h"
#include "PointCloudBackendData.h"

#include <osgViewer/Viewer>

#include "OsgWidgetBackendLoadController.h"
#include "OsgWidgetImportController.h"
#include "OsgWidgetCaptureController.h"
#include "OsgWidgetPickAnnotationController.h"
#include "OsgWidgetColorController.h"
#include "OsgWidgetTransformHierarchyController.h"

#include "GraphicsWindowQt1.h"
#include "ObjectTransformOperation.h"
#include "PointPickOperation.h"
#include "MeshEdgeFacePickOperation.h"
#include "QWidgetViewer.h"

OsgWidget::OsgWidget(QWidget* parent)
	: QWidget(parent)
{
	m_feedbackTimer.start();
	m_pointPickOperation = std::make_unique<PointPickOperation>(this);
	m_objectTransformOperation = std::make_unique<ObjectTransformOperation>(this);
	m_meshElementPickOperation = std::make_unique<MeshEdgeFacePickOperation>(this);
	m_importController = std::make_unique<OsgWidgetImportController>();
	m_backendLoadController = std::make_unique<OsgWidgetBackendLoadController>();
	m_captureController = std::make_unique<OsgWidgetCaptureController>();
	m_pickAnnotationController = std::make_unique<OsgWidgetPickAnnotationController>();
	initScene();
	initUi();
	setRequestRedraw([this]() {
		if (m_glWidget)
		{
			m_glWidget->update();
		}
	});
	if (m_glWidget)
	{
		setViewportPixels(m_glWidget->width(), m_glWidget->height());
		setDevicePixelRatio(m_glWidget->devicePixelRatio());
	}
	initViewer();
}

OsgWidget::~OsgWidget() = default;

bool OsgWidget::hasImportedContent() const
{
	if (m_stagingGroup.valid() && m_stagingGroup->getNumChildren() > 0)
	{
		return true;
	}
	return !m_backendObjectRoots.empty();
}

void OsgWidget::applyRigidRotationAboutWorldPivot(const std::vector<std::string>& backendIds, const osg::Vec3f& pivotWorld,
	const osg::Quat& deltaRotation)
{
	for (const std::string& id : backendIds)
	{
		const auto it = m_backendObjectRoots.find(id);
		if (it == m_backendObjectRoots.end() || !it->second.valid())
		{
			continue;
		}
		osg::PositionAttitudeTransform* pat = it->second.get();
		const osg::Vec3f p = pat->getPosition();
		const osg::Quat q = pat->getAttitude();
		const osg::Vec3f rel = p - pivotWorld;
		const osg::Vec3f pNew = deltaRotation * rel + pivotWorld;
		const osg::Quat qNew = deltaRotation * q;
		pat->setPosition(pNew);
		pat->setAttitude(qNew);
	}
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
	}
	if (m_glWidget)
	{
		m_glWidget->update();
	}
}

osg::Vec3f OsgWidget::averageBackendRootPositionWorld(const std::vector<std::string>& backendIds) const
{
	osg::Vec3f sum(0.0f, 0.0f, 0.0f);
	int n = 0;
	for (const std::string& id : backendIds)
	{
		const auto it = m_backendObjectRoots.find(id);
		if (it == m_backendObjectRoots.end() || !it->second.valid())
		{
			continue;
		}
		sum += it->second->getPosition();
		++n;
	}
	if (n <= 0)
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	return sum * (1.0f / static_cast<float>(n));
}

namespace
{

osg::NodePath nodePathToSceneRoot(osg::Node* leaf)
{
	osg::NodePath path;
	for (osg::Node* n = leaf; n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), n);
	}
	return path;
}

struct PoseMat
{
	double m[16]{};
};

PoseMat poseMatIdentity()
{
	PoseMat r{};
	r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0;
	return r;
}

PoseMat poseMatMul(const PoseMat& a, const PoseMat& b)
{
	PoseMat r{};
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			r.m[col * 4 + row] = a.m[0 * 4 + row] * b.m[col * 4 + 0]
				+ a.m[1 * 4 + row] * b.m[col * 4 + 1]
				+ a.m[2 * 4 + row] * b.m[col * 4 + 2]
				+ a.m[3 * 4 + row] * b.m[col * 4 + 3];
		}
	}
	return r;
}

PoseMat poseMatTranslate(double x, double y, double z)
{
	PoseMat r = poseMatIdentity();
	r.m[12] = x;
	r.m[13] = y;
	r.m[14] = z;
	return r;
}

PoseMat poseMatFromRpy(double roll, double pitch, double yaw)
{
	const double cr = std::cos(roll);
	const double sr = std::sin(roll);
	const double cp = std::cos(pitch);
	const double sp = std::sin(pitch);
	const double cy = std::cos(yaw);
	const double sy = std::sin(yaw);
	PoseMat r = poseMatIdentity();
	r.m[0] = cy * cp;
	r.m[1] = cy * sp * sr - sy * cr;
	r.m[2] = cy * sp * cr + sy * sr;
	r.m[4] = sy * cp;
	r.m[5] = sy * sp * sr + cy * cr;
	r.m[6] = sy * sp * cr - cy * sr;
	r.m[8] = -sp;
	r.m[9] = cp * sr;
	r.m[10] = cp * cr;
	return r;
}

PoseMat poseMatFromXyzRpy(double x, double y, double z, double roll, double pitch, double yaw)
{
	return poseMatMul(poseMatTranslate(x, y, z), poseMatFromRpy(roll, pitch, yaw));
}

osg::Matrixd poseMatToOsgLikeUrdf(const PoseMat& m)
{
	osg::Matrixd o;
	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			if (r == 3 || c == 3)
			{
				o(r, c) = m.m[r * 4 + c];
			}
			else
			{
				o(r, c) = m.m[c * 4 + r];
			}
		}
	}
	return o;
}

osg::ref_ptr<osg::Geode> createInstructionPoseAxisGeode(float axisLengthMm, bool lineMotion)
{
	osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array;
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	const osg::Vec4 xColor = lineMotion ? osg::Vec4(1.0f, 0.4f, 0.4f, 1.0f) : osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f);
	const osg::Vec4 yColor = lineMotion ? osg::Vec4(0.4f, 1.0f, 0.4f, 1.0f) : osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f);
	const osg::Vec4 zColor = lineMotion ? osg::Vec4(0.4f, 0.6f, 1.0f, 1.0f) : osg::Vec4(0.1f, 0.3f, 1.0f, 1.0f);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(axisLengthMm, 0.0f, 0.0f));
	colors->push_back(xColor);
	colors->push_back(xColor);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(0.0f, axisLengthMm, 0.0f));
	colors->push_back(yColor);
	colors->push_back(yColor);

	verts->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	verts->push_back(osg::Vec3(0.0f, 0.0f, axisLengthMm));
	colors->push_back(zColor);
	colors->push_back(zColor);

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(verts.get());
	geom->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);
	geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, static_cast<GLsizei>(verts->size())));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geom.get());

	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
	ss->setMode(GL_BLEND, osg::StateAttribute::ON);
	osg::ref_ptr<osg::LineWidth> lw = new osg::LineWidth(lineMotion ? 3.0f : 2.5f);
	ss->setAttributeAndModes(lw.get(), osg::StateAttribute::ON);
	return geode;
}

} // namespace

bool OsgWidget::getBackendRootWorldMatrix(const std::string& backendId, osg::Matrixd& outWorld) const
{
	const auto it = m_backendObjectRoots.find(backendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		return false;
	}
	osg::PositionAttitudeTransform* pat = it->second.get();
	outWorld = osg::computeLocalToWorld(nodePathToSceneRoot(pat));
	return true;
}

void OsgWidget::setBackendRootWorldMatrixFromWorld(const std::string& backendId, const osg::Matrixd& worldMat)
{
	const auto it = m_backendObjectRoots.find(backendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		return;
	}
	osg::PositionAttitudeTransform* pat = it->second.get();
	osg::Node* parent = pat->getNumParents() > 0 ? pat->getParent(0) : nullptr;
	osg::Matrixd parentWorld;
	if (!parent)
	{
		parentWorld.makeIdentity();
	}
	else
	{
		parentWorld = osg::computeLocalToWorld(nodePathToSceneRoot(parent));
	}
	const osg::Matrixd local = osg::Matrixd::inverse(parentWorld) * worldMat;
	osg::Vec3d trans;
	osg::Quat rot;
	osg::Vec3d scale;
	osg::Quat so;
	local.decompose(trans, rot, scale, so);
	pat->setPosition(osg::Vec3f(static_cast<float>(trans.x()), static_cast<float>(trans.y()),
		static_cast<float>(trans.z())));
	pat->setAttitude(rot);
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
	}
	if (m_glWidget)
	{
		m_glWidget->update();
	}
}

void OsgWidget::setInstructionPoseAxes(const std::vector<InstructionPoseAxis>& axes)
{
	if (!m_trajectoryOverlayGroup.valid())
	{
		return;
	}
	if (!m_instructionPoseAxesGroup.valid())
	{
		m_instructionPoseAxesGroup = new osg::Group;
		m_instructionPoseAxesGroup->setName("InstructionPoseAxes");
	}
	osg::Group* parentGroup = m_trajectoryOverlayGroup.get();
	if (!axes.empty() && !axes.front().robotBackendId.empty())
	{
		const auto it = m_backendObjectRoots.find(axes.front().robotBackendId);
		if (it != m_backendObjectRoots.end() && it->second.valid())
		{
			// Prefer attaching under robot assembly root (PAT child) so axis local matrix
			// uses the same local frame as URDF joint/link hierarchy.
			osg::Group* robotLocalRoot = nullptr;
			if (it->second->getNumChildren() > 0)
			{
				robotLocalRoot = dynamic_cast<osg::Group*>(it->second->getChild(0));
			}
			parentGroup = robotLocalRoot ? robotLocalRoot : static_cast<osg::Group*>(it->second.get());
		}
	}
	while (m_instructionPoseAxesGroup->getNumParents() > 0)
	{
		osg::Group* p = m_instructionPoseAxesGroup->getParent(0);
		if (!p)
		{
			break;
		}
		p->removeChild(m_instructionPoseAxesGroup.get());
	}
	if (parentGroup)
	{
		parentGroup->addChild(m_instructionPoseAxesGroup.get());
	}
	m_instructionPoseAxesGroup->removeChildren(0, m_instructionPoseAxesGroup->getNumChildren());

	for (const InstructionPoseAxis& a : axes)
	{
		osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
		mt->setName(a.lineMotion ? "LINE_TargetAxis" : "PTP_TargetAxis");
		const osg::Vec3f p = a.positionMm;
		osg::Matrixd m;
		if (a.hasLocalMatrix)
		{
			for (int r = 0; r < 4; ++r)
			{
				for (int c = 0; c < 4; ++c)
				{
					m(r, c) = a.localMatrix[r * 4 + c];
				}
			}
		}
		else
		{
			static constexpr double kPi = 3.14159265358979323846;
			const double rr = static_cast<double>(a.eulerDeg.x()) * kPi / 180.0;
			const double rp = static_cast<double>(a.eulerDeg.y()) * kPi / 180.0;
			const double ry = static_cast<double>(a.eulerDeg.z()) * kPi / 180.0;
			const PoseMat mLocal = poseMatFromXyzRpy(
				static_cast<double>(p.x()),
				static_cast<double>(p.y()),
				static_cast<double>(p.z()),
				rr, rp, ry);
			m = poseMatToOsgLikeUrdf(mLocal);
		}
		mt->setMatrix(m);
		mt->addChild(createInstructionPoseAxisGeode(a.lineMotion ? 100.0f : 80.0f, a.lineMotion).get());
		m_instructionPoseAxesGroup->addChild(mt.get());
	}
	if (m_glWidget)
	{
		m_glWidget->update();
	}
}

void OsgWidget::clearInstructionPoseAxes()
{
	if (m_instructionPoseAxesGroup.valid())
	{
		m_instructionPoseAxesGroup->removeChildren(0, m_instructionPoseAxesGroup->getNumChildren());
	}
	if (m_glWidget)
	{
		m_glWidget->update();
	}
}

void OsgWidget::setViewerBackgroundForDarkUi(bool dark)
{
	m_darkUiTheme = dark;
	if (!m_viewer.valid() || !m_viewer->getCamera())
	{
		return;
	}
	// Dark UI: medium-light gray canvas (readable vs. ~53 dock gray). Light UI: near-white.
	const osg::Vec4 color = dark ? osg::Vec4(0.40f, 0.40f, 0.42f, 1.0f) : osg::Vec4(0.98f, 0.98f, 0.98f, 1.0f);
	m_viewer->getCamera()->setClearColor(color);
	const osg::Vec4 annotationTextColor = dark
		? osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)
		: osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f);
	for (auto& a : m_annotations)
	{
		if (a.textDrawable.valid())
		{
			a.textDrawable->setColor(annotationTextColor);
		}
	}
	if (m_glWidget)
	{
		m_glWidget->update();
	}
}

void OsgWidget::clearStagingGeometry()
{
	if (m_stagingGroup.valid())
	{
		m_stagingGroup->removeChildren(0, m_stagingGroup->getNumChildren());
	}
}

osg::Node* OsgWidget::stagingGeometryRoot() const
{
	if (!m_stagingGroup.valid() || m_stagingGroup->getNumChildren() < 1)
	{
		return nullptr;
	}
	return m_stagingGroup->getChild(0);
}

void OsgWidget::applyVisibilityMaskForBackend(const std::string& backendId)
{
	auto it = m_backendObjectRoots.find(backendId);
	if (it == m_backendObjectRoots.end() || !it->second.valid())
	{
		return;
	}
	bool vis = true;
	auto v = m_backendVisibility.find(backendId);
	if (v != m_backendVisibility.end())
	{
		vis = v->second;
	}
	it->second->setNodeMask(vis ? 0xffffffffu : 0u);
}

void OsgWidget::setBackendObjectVisible(const std::string& backendId, bool visible)
{
	m_backendVisibility[backendId] = visible;
	applyVisibilityMaskForBackend(backendId);
}

void OsgWidget::setBackendParent(const std::string& backendId, const std::string& parentBackendId)
{
	OsgWidgetTransformHierarchyController::setBackendParent(*this, backendId, parentBackendId);
}

void OsgWidget::removeBackendObjectVisual(const std::string& backendId)
{
	OsgWidgetTransformHierarchyController::removeBackendObjectVisual(*this, backendId);
}

bool OsgWidget::hasBackendObjectBranch(const std::string& backendId) const
{
	const auto it = m_backendObjectRoots.find(backendId);
	return it != m_backendObjectRoots.end() && it->second.valid();
}

void OsgWidget::syncSelectionFromBackend(const PointCloudBackendData& data)
{
	OsgScene::syncGizmoAndPickFromBackend(data);
	cacheSelectionPoseFromSelectedTransform();
	syncCompassGizmoOrientation();
	OsgWidgetTransformHierarchyController::finalizeSelectionSync(*this);
}

void OsgWidget::syncSelectionFromBackend(const MeshBackendData& data)
{
	OsgScene::syncGizmoAndPickFromBackend(data);
	cacheSelectionPoseFromSelectedTransform();
	syncCompassGizmoOrientation();
	OsgWidgetTransformHierarchyController::finalizeSelectionSync(*this);
}

void OsgWidget::syncSelectionForBackendId(const std::string& backendId)
{
	OsgWidgetTransformHierarchyController::syncSelectionForBackendId(*this, backendId);
}

osg::ref_ptr<osg::Geode> OsgWidget::buildPointCloudGeode(const PointCloudBackendData& data, QString* errorMessage) const
{
	std::string err;
	osg::ref_ptr<osg::Geode> geode = BackendVisualRegistry::buildPointCloudGeode(data, errorMessage ? &err : nullptr);
	if (!geode && errorMessage)
	{
		*errorMessage = QString::fromStdString(err);
	}
	return geode;
}

bool OsgWidget::upsertPointCloudBranchInScene(const PointCloudBackendData& data, QString* errorMessage, bool resetViewToHome)
{
	MeshVisualOptions meshOpts{};
	BranchBuildResult built;
	std::string err;
	if (!BackendVisualRegistry::buildOuterBranch(data, meshOpts, built, errorMessage ? &err : nullptr))
	{
		if (errorMessage)
		{
			*errorMessage = QString::fromStdString(err);
		}
		return false;
	}
	osg::ref_ptr<osg::PositionAttitudeTransform> outer = built.outer;
	const std::string id = data.id();
	const osg::Vec3f center = built.modelCenter;
	const float diagonal = built.diagonal;
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid() && m_backendObjectsGroup.valid())
	{
		m_backendObjectsGroup->removeChild(it->second.get());
	}
	m_backendObjectRoots.erase(id);
	m_backendObjectsGroup->addChild(outer.get());
	m_backendObjectRoots.insert(std::make_pair(id, std::move(outer)));
	m_backendModelCenters[id] = center;
	if (m_activeBackendId == id || m_activeBackendId.empty())
	{
		m_activeModelDiagonal = diagonal;
	}
	if (m_backendVisibility.find(id) == m_backendVisibility.end())
	{
		m_backendVisibility[id] = true;
	}
	applyVisibilityMaskForBackend(id);
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		if (resetViewToHome)
		{
			outer->dirtyBound();
			focusCameraOnBackend(id);
		}
	}
	return true;
}

osg::ref_ptr<osg::Node> OsgWidget::buildMeshGeode(const MeshBackendData& data, QString* errorMessage,
	bool showWireOutline, bool useSceneLighting) const
{
	MeshVisualOptions opt;
	opt.showWireOutline = showWireOutline;
	opt.useSceneLighting = useSceneLighting;
	std::string err;
	osg::ref_ptr<osg::Node> node = BackendVisualRegistry::buildMeshDisplayNode(data, opt, errorMessage ? &err : nullptr);
	if (!node && errorMessage)
	{
		*errorMessage = QString::fromStdString(err);
	}
	return node;
}

bool OsgWidget::upsertMeshBranchInScene(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
	bool showWireOutline, bool useSceneLighting)
{
	MeshVisualOptions meshOpts;
	meshOpts.showWireOutline = showWireOutline;
	meshOpts.useSceneLighting = useSceneLighting;
	BranchBuildResult built;
	std::string err;
	if (!BackendVisualRegistry::buildOuterBranch(data, meshOpts, built, errorMessage ? &err : nullptr))
	{
		if (errorMessage)
		{
			*errorMessage = QString::fromStdString(err);
		}
		return false;
	}
	osg::ref_ptr<osg::PositionAttitudeTransform> outer = built.outer;
	const std::string id = data.id();
	if (useSceneLighting)
	{
		m_litMeshBackendIds.insert(id);
	}
	else
	{
		m_litMeshBackendIds.erase(id);
	}
	const osg::Vec3f center = built.modelCenter;
	const float diagonal = built.diagonal;
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid() && m_backendObjectsGroup.valid())
	{
		m_backendObjectsGroup->removeChild(it->second.get());
	}
	m_backendObjectRoots.erase(id);
	m_backendObjectsGroup->addChild(outer.get());
	m_backendObjectRoots.insert(std::make_pair(id, std::move(outer)));
	m_backendModelCenters[id] = center;
	if (m_activeBackendId == id || m_activeBackendId.empty())
	{
		m_activeModelDiagonal = diagonal;
	}
	if (m_backendVisibility.find(id) == m_backendVisibility.end())
	{
		m_backendVisibility[id] = true;
	}
	applyVisibilityMaskForBackend(id);
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		if (resetViewToHome)
		{
			outer->dirtyBound();
			focusCameraOnBackend(id);
		}
	}
	return true;
}

bool OsgWidget::importModelFile(const QString& filePath, QString* errorMessage)
{
	return m_importController
		? m_importController->importModelFile(*this, filePath, errorMessage)
		: false;
}

osg::Node* OsgWidget::loadXyzPointCloud(const QString& filePath, QString* errorMessage)
{
	QFile file(filePath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		if (errorMessage) *errorMessage = QStringLiteral("Cannot open xyz file.");
		return nullptr;
	}

	QTextStream in(&file);
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	points->reserve(50000);

	while (!in.atEnd())
	{
		const QString line = in.readLine().trimmed();
		if (line.isEmpty() || line.startsWith('#'))
		{
			continue;
		}

		const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
		if (parts.size() < 3)
		{
			continue;
		}

		bool okX = false, okY = false, okZ = false;
		const float x = parts[0].toFloat(&okX);
		const float y = parts[1].toFloat(&okY);
		const float z = parts[2].toFloat(&okZ);
		if (okX && okY && okZ)
		{
			points->push_back(osg::Vec3(x, y, z));
		}
	}

	if (points->empty())
	{
		if (errorMessage) *errorMessage = QStringLiteral("No valid points found in xyz file.");
		return nullptr;
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(points.get());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points->size())));

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
	geometry->setColorArray(colors.get(), osg::Array::BIND_OVERALL);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geometry.get());
	geode->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f));
	return geode.release();
}

osg::Node* OsgWidget::loadAsciiPlyPointCloud(const QString& filePath, QString* errorMessage)
{
	QFile file(filePath);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		if (errorMessage) *errorMessage = QStringLiteral("Cannot open ply file.");
		return nullptr;
	}

	QTextStream in(&file);
	QString line;
	bool isAscii = false;
	bool headerEnded = false;
	int vertexCount = 0;

	while (!in.atEnd())
	{
		line = in.readLine().trimmed();
		if (line.startsWith(QStringLiteral("format ascii")))
		{
			isAscii = true;
		}
		else if (line.startsWith(QStringLiteral("element vertex")))
		{
			const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
			if (parts.size() >= 3) vertexCount = parts[2].toInt();
		}
		else if (line == QStringLiteral("end_header"))
		{
			headerEnded = true;
			break;
		}
	}

	if (!headerEnded || !isAscii || vertexCount <= 0)
	{
		if (errorMessage) *errorMessage = QStringLiteral("Only ascii ply fallback is supported.");
		return nullptr;
	}

	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	points->reserve(static_cast<unsigned int>(vertexCount));
	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->reserve(static_cast<unsigned int>(vertexCount));

	for (int i = 0; i < vertexCount && !in.atEnd(); ++i)
	{
		line = in.readLine().trimmed();
		if (line.isEmpty()) continue;
		const QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
		if (parts.size() < 3) continue;
		bool okX = false, okY = false, okZ = false;
		const float x = parts[0].toFloat(&okX);
		const float y = parts[1].toFloat(&okY);
		const float z = parts[2].toFloat(&okZ);
		if (!(okX && okY && okZ)) continue;
		points->push_back(osg::Vec3(x, y, z));

		if (parts.size() >= 6)
		{
			bool okR = false, okG = false, okB = false;
			const float r = parts[3].toFloat(&okR) / 255.0f;
			const float g = parts[4].toFloat(&okG) / 255.0f;
			const float b = parts[5].toFloat(&okB) / 255.0f;
			if (okR && okG && okB) colors->push_back(osg::Vec4(r, g, b, 1.0f));
			else colors->push_back(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
		}
		else
		{
			colors->push_back(osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f));
		}
	}

	if (points->empty())
	{
		if (errorMessage) *errorMessage = QStringLiteral("No valid vertex data in ply.");
		return nullptr;
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(points.get());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points->size())));
	geometry->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geometry.get());
	geode->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f));
	return geode.release();
}

bool OsgWidget::importPointCloudFile(const QString& filePath, QString* errorMessage)
{
	return m_importController
		? m_importController->importPointCloudFile(*this, filePath, errorMessage)
		: false;
}

void OsgWidget::initUi()
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(0);

	QGLFormat fmt;
	fmt.setDepthBufferSize(24);
	fmt.setDoubleBuffer(true);

	m_glWidget = new QWidgetViewer(fmt, this);
	m_glWidget->setMinimumSize(640, 480);
	layout->addWidget(m_glWidget);
	m_glWidget->installEventFilter(this);
}

void OsgWidget::initViewer()
{
	m_viewer = new osgViewer::Viewer;
	m_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
	m_viewer->setRunFrameScheme(osgViewer::Viewer::CONTINUOUS);

	// GraphicsContext must be attached to the main camera before setSceneData, otherwise OSG can
	// touch invalid viewer/GC state during scene compile (often Debug-only access violations).
	auto* gwQt = new GraphicsWindowQt1(m_glWidget);
	m_graphicsWindow = gwQt;
	gwQt->setViewer(m_viewer.get());
	m_glWidget->setGraphicsWindow(gwQt);

	connect(m_glWidget, &QWidgetViewer::windowResized, this, [this](int w, int h) {
		if (m_graphicsWindow.valid())
		{
			static_cast<GraphicsWindowQt1*>(m_graphicsWindow.get())->updateSize(w, h);
		}
		setViewportPixels(w, h);
		if (m_glWidget)
		{
			setDevicePixelRatio(m_glWidget->devicePixelRatio());
		}
		if (m_viewer.valid() && m_viewer->getCamera())
		{
			m_viewer->getCamera()->setViewport(0, 0, w, h);
			const double aspect = static_cast<double>((std::max)(1, w))
				/ static_cast<double>((std::max)(1, h));
			m_viewer->getCamera()->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
		}
		updateWorldAxesHudViewport(w, h);
	});

	m_viewer->getCamera()->setGraphicsContext(m_graphicsWindow.get());
	m_viewer->getCamera()->setCullMask(0xffffffffu);
	// Some OSG builds crash in Debug when viewport/projection are set before the graphics context is fully valid.
	// Defer these calls unless the context reports valid; resize callback will configure them again.
	if (m_graphicsWindow.valid() && m_graphicsWindow->valid())
	{
		m_viewer->getCamera()->setViewport(0, 0, m_glWidget->width(), m_glWidget->height());
		const double aspect = static_cast<double>((std::max)(1, m_glWidget->width()))
			/ static_cast<double>((std::max)(1, m_glWidget->height()));
		m_viewer->getCamera()->setProjectionMatrixAsPerspective(30.0, aspect, 10.0, 1e8);
	}

	applyHeadlightToViewer(m_viewer.get());
	m_viewer->setSceneData(m_root.get());
	m_viewer->addEventHandler(new osgGA::StateSetManipulator(m_viewer->getCamera()->getOrCreateStateSet()));

	m_trackballManipulator = new osgGA::TrackballManipulator;
	m_trackballManipulator->setAllowThrow(false);
	m_viewer->setCameraManipulator(m_trackballManipulator.get());
	m_viewer->getEventQueue()->syncWindowRectangleWithGraphicsContext();

	setViewerBackgroundForDarkUi(false);
	m_viewer->getCamera()->setViewMatrixAsLookAt(
		osg::Vec3(3, 3, 3),
		osg::Vec3(0, 0, 0),
		osg::Vec3(0, 0, 1));

	connect(&m_frameTimer, &QTimer::timeout, this, [this]() {
		updateCompassScale();
		if (m_pickAnnotationController)
		{
			m_pickAnnotationController->updateAnnotationScales(*this);
		}
		if (m_glWidget) m_glWidget->update();
	});
	m_frameTimer.start(16);

	OsgScene::initWorldAxesHud();
}

osg::Node* OsgWidget::createCompassNode()
{
	return OsgScene::createCompassNode();
}

void OsgWidget::setSelectionActive(bool active)
{
	m_selectionActive = active;
	if (!active)
	{
		m_gizmoReferenceDistance = -1.0;
		m_gizmoReferenceScale = 1.0;
		m_hasLastSelectionPose = false;
	}
	updateCompassHighlight(DragAxis::None);
	emit activeAxisChanged(QStringLiteral("None"));
	refreshCompassDrawVisibility();
}

void OsgWidget::attachCompassGraphics()
{
	OsgScene::attachCompassGraphics();
}

void OsgWidget::detachCompassGraphics()
{
	OsgScene::detachCompassGraphics();
}

void OsgWidget::syncCompassGizmoOrientation()
{
	if (!m_compassTransform.valid() || !m_selectedTransform.valid())
	{
		return;
	}
	// Model origin in outer local frame is -m_modelCenter; compass offset matches. World mode: draw axes in world
	// directions while pivot stays on parent (inverse cancels object rotation on child attitude).
	if (m_transformGizmoFrame == TransformGizmoFrame::World)
	{
		const osg::Quat q = m_selectedTransform->getAttitude();
		m_compassTransform->setAttitude(q.inverse());
	}
	else
	{
		m_compassTransform->setAttitude(osg::Quat());
	}
}

void OsgWidget::refreshCompassDrawVisibility()
{
	OsgScene::refreshCompassDrawVisibility();
	syncCompassGizmoOrientation();
}

void OsgWidget::syncCameraManipulatorForModes()
{
	if (!m_viewer.valid())
	{
		return;
	}
	// Keep camera navigation enabled in all modes. Specific edit operations
	// consume events only when they really need to (e.g. dragging gizmo axis).
	const bool blockCameraNav = false;
	// resetPosition=false keeps the current camera when re-attaching the trackball after object/point modes
	// or ESC (OSG default true would jump to viewer "home" every time).
	m_viewer->setCameraManipulator(blockCameraNav ? nullptr : m_trackballManipulator.get(), false);
	if (!blockCameraNav)
	{
		// Point/object modes swallow mouse presses in Qt but releases still reach osgGA; the
		// accumulated button mask then disagrees with reality and TrackballManipulator stops responding.
		resetNavigationInputQueues();
		if (m_glWidget)
		{
			m_glWidget->setFocus(Qt::OtherFocusReason);
			m_glWidget->update();
		}
	}
}

bool OsgWidget::pickAndActivateBackendAtScreenPos(const QPoint& mousePos)
{
	if (!m_glWidget)
	{
		return false;
	}
	const bool ok = OsgScene::pickAndActivateBackendAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()));
	if (ok)
	{
		refreshAnnotationTexts();
		setSelectionActive(true);
		syncCompassGizmoOrientation();
	}
	return ok;
}

void OsgWidget::setObjectSelectionMode(bool enabled)
{
	m_objectSelectionMode = enabled;
	if (enabled)
	{
		m_pointPickMode = false;
		m_meshLinePickMode = false;
		m_meshFacePickMode = false;
		// Drop any in-progress view navigation before Qt starts swallowing mouse for gizmo.
		resetNavigationInputQueues();
	}
	syncCameraManipulatorForModes();
	if (!enabled)
	{
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
	}
	refreshCompassDrawVisibility();
}

bool OsgWidget::objectSelectionMode() const
{
	return m_objectSelectionMode;
}

void OsgWidget::setTransformGizmoFrame(TransformGizmoFrame frame)
{
	m_transformGizmoFrame = frame;
	syncCompassGizmoOrientation();
}

void OsgWidget::setPointPickMode(bool enabled)
{
	m_pointPickMode = enabled;
	if (enabled)
	{
		m_objectSelectionMode = false;
		m_meshLinePickMode = false;
		m_meshFacePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		emit pointPickFeedback(QStringLiteral("Point Pick: move mouse over point cloud."));
		resetNavigationInputQueues();
	}
	else
	{
		clearPointPickMarker();
		emit pointPickFeedback(QStringLiteral("Point Pick: off"));
	}
	syncCameraManipulatorForModes();
	refreshCompassDrawVisibility();
}

bool OsgWidget::pointPickMode() const
{
	return m_pointPickMode;
}

void OsgWidget::setMeshLinePickMode(bool enabled)
{
	m_meshLinePickMode = enabled;
	if (enabled)
	{
		m_objectSelectionMode = false;
		m_pointPickMode = false;
		m_meshFacePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		hideMeshElementHighlight();
		emit pointPickFeedback(QStringLiteral("Mesh Line Pick: move mouse over mesh edges."));
		resetNavigationInputQueues();
	}
	else
	{
		hideMeshElementHighlight();
		emit pointPickFeedback(QStringLiteral("Mesh Line Pick: off"));
	}
	syncCameraManipulatorForModes();
	refreshCompassDrawVisibility();
}

bool OsgWidget::meshLinePickMode() const
{
	return m_meshLinePickMode;
}

void OsgWidget::setMeshFacePickMode(bool enabled)
{
	m_meshFacePickMode = enabled;
	if (enabled)
	{
		m_objectSelectionMode = false;
		m_pointPickMode = false;
		m_meshLinePickMode = false;
		m_dragging = false;
		m_rotating = false;
		m_dragAxis = DragAxis::None;
		updateCompassHighlight(DragAxis::None);
		emit activeAxisChanged(QStringLiteral("None"));
		hideMeshElementHighlight();
		emit pointPickFeedback(QStringLiteral("Mesh Face Pick: move mouse over mesh faces."));
		resetNavigationInputQueues();
	}
	else
	{
		hideMeshElementHighlight();
		emit pointPickFeedback(QStringLiteral("Mesh Face Pick: off"));
	}
	syncCameraManipulatorForModes();
	refreshCompassDrawVisibility();
}

bool OsgWidget::meshFacePickMode() const
{
	return m_meshFacePickMode;
}

QString OsgWidget::pointCloudPluginReport() const
{
	auto hasReader = [](const char* ext) -> bool {
		return osgDB::Registry::instance()->getReaderWriterForExtension(ext) != nullptr;
	};

	// xyz uses built-in loader in this widget, so it is always available.
	const QString ply = hasReader("ply") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString las = hasReader("las") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString laz = hasReader("laz") ? QStringLiteral("OK") : QStringLiteral("Missing");
	const QString xyz = QStringLiteral("OK (built-in)");
	return QStringLiteral("PointCloud plugin check | ply:%1 las:%2 laz:%3 xyz:%4")
		.arg(ply).arg(las).arg(laz).arg(xyz);
}

osg::Vec3f OsgWidget::selectedPosition() const
{
	if (!m_selectedTransform.valid())
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	return m_selectedTransform->getPosition() - m_modelCenter;
}

void OsgWidget::setSelectedPosition(const osg::Vec3f& position)
{
	if (!m_selectedTransform.valid())
	{
		return;
	}
	m_selectedTransform->setPosition(m_modelCenter + position);
	syncActiveBackendRootFromSelectedTransform();
	refreshAnnotationTexts();
	syncCompassGizmoOrientation();
	emit selectedObjectPoseChanged(position.x(), position.y(), position.z());
}

osg::Vec3f OsgWidget::selectedRotationEulerDeg() const
{
	if (!m_selectedTransform.valid())
	{
		return osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	return OsgScene::quatToEulerDeg(m_selectedTransform->getAttitude());
}

void OsgWidget::setSelectedRotationEulerDeg(const osg::Vec3f& eulerDeg)
{
	if (!m_selectedTransform.valid())
	{
		return;
	}
	m_selectedTransform->setAttitude(OsgScene::eulerDegToQuat(eulerDeg));
	syncActiveBackendRootFromSelectedTransform();
	refreshAnnotationTexts();
	syncCompassGizmoOrientation();
	emit selectedObjectRotationChanged(eulerDeg.x(), eulerDeg.y(), eulerDeg.z());
}

void OsgWidget::syncActiveBackendRootFromSelectedTransform()
{
	OsgScene::syncActiveBackendRootFromSelectedTransform();
}

void OsgWidget::setSelectedColor(float r, float g, float b, float a)
{
	OsgWidgetColorController::applyColorToActiveBackendObject(*this, osg::Vec4(r, g, b, a));
	emit selectedObjectColorChanged(r, g, b, a);
}

OsgWidget::DragAxis OsgWidget::pickAxisAtScreenPos(const QPoint& mousePos, bool preferRing) const
{
	const int axis = OsgScene::pickAxisAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), preferRing);
	return static_cast<DragAxis>(axis);
}

void OsgWidget::updateCompassHighlight(DragAxis axis)
{
	OsgScene::updateCompassHighlight(static_cast<int>(axis));
}

QString OsgWidget::axisToString(DragAxis axis) const
{
	if (axis == DragAxis::X) return QStringLiteral("X");
	if (axis == DragAxis::Y) return QStringLiteral("Y");
	if (axis == DragAxis::Z) return QStringLiteral("Z");
	return QStringLiteral("None");
}

void OsgWidget::updateCompassScale()
{
	OsgScene::updateCompassScale();
}

void OsgWidget::clearPointAnnotations()
{
	for (const AnnotationEntry& a : m_annotations)
	{
		emit annotationRemoved(QString::fromStdString(a.id));
		if (!a.transform.valid())
		{
			continue;
		}
		if (m_annotationGroup.valid())
		{
			m_annotationGroup->removeChild(a.transform.get());
		}
	}
	if (m_annotationGroup.valid())
	{
		// Ensure legacy gizmo-attached annotations are removed too.
		m_annotationGroup->removeChildren(0, m_annotationGroup->getNumChildren());
	}
	m_annotations.clear();
	m_annotationCounter = 0;
}

bool OsgWidget::pickPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld) const
{
	return OsgScene::pickPointAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), outPointWorld);
}

bool OsgWidget::pickNearestPointAtScreenPos(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx, bool previewOnly) const
{
	return OsgScene::pickNearestPointAtScreenPos(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), outPointWorld, outDistancePx, previewOnly);
}

bool OsgWidget::pickPointByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, double& outDistancePx) const
{
	return OsgScene::pickPointByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()), outPointWorld, outDistancePx);
}

bool OsgWidget::pickMeshFaceByRayIntersection(const QPoint& mousePos,
	osg::Vec3f& outPointWorld,
	osg::Vec3f& outAWorld,
	osg::Vec3f& outBWorld,
	osg::Vec3f& outCWorld,
	osg::Vec3f& outNormalWorld,
	std::vector<osg::Vec3f>* outMergedCoplanarVertsWorld) const
{
	return OsgScene::pickMeshFaceByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
		outPointWorld, outAWorld, outBWorld, outCWorld, outNormalWorld, outMergedCoplanarVertsWorld);
}

bool OsgWidget::pickMeshEdgeByRayIntersection(const QPoint& mousePos, osg::Vec3f& outPointWorld, osg::Vec3f& outEdgeAWorld, osg::Vec3f& outEdgeBWorld) const
{
	return OsgScene::pickMeshEdgeByRayIntersection(static_cast<double>(mousePos.x()), static_cast<double>(mousePos.y()),
		outPointWorld, outEdgeAWorld, outEdgeBWorld);
}

void OsgWidget::addPointAnnotation(const osg::Vec3f& pointWorld)
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->addPointAnnotation(*this, pointWorld);
	}
}

void OsgWidget::refreshAnnotationTexts()
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->refreshAnnotationTexts(*this);
	}
}

bool OsgWidget::setAnnotationVisible(const QString& annotationId, bool visible)
{
	return m_pickAnnotationController
		? m_pickAnnotationController->setAnnotationVisible(*this, annotationId, visible)
		: false;
}

bool OsgWidget::removeAnnotation(const QString& annotationId)
{
	return m_pickAnnotationController
		? m_pickAnnotationController->removeAnnotation(*this, annotationId)
		: false;
}

void OsgWidget::clearAllAnnotations()
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->clearAllAnnotations(*this);
	}
	else
	{
		clearPointAnnotations();
	}
}

QList<OsgWidget::AnnotationSnapshot> OsgWidget::annotationSnapshots() const
{
	return m_pickAnnotationController
		? m_pickAnnotationController->annotationSnapshots(*this)
		: QList<AnnotationSnapshot>();
}

void OsgWidget::restoreAnnotations(const QList<AnnotationSnapshot>& snapshots)
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->restoreAnnotations(*this, snapshots);
	}
}

void OsgWidget::updatePointPickMarker(const osg::Vec3f& pointWorld, bool hit)
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->updatePointPickMarker(*this, pointWorld, hit);
	}
}

void OsgWidget::clearPointPickMarker()
{
	if (m_pickAnnotationController)
	{
		m_pickAnnotationController->clearPointPickMarker(*this);
	}
}

void OsgWidget::applyColorToStagingGeometry(const osg::Vec4& color)
{
	OsgWidgetColorController::applyColorToStagingGeometry(*this, color);
}

void OsgWidget::applyColorToBackendObject(const std::string& backendId, const osg::Vec4& color)
{
	OsgWidgetColorController::applyColorToBackendObject(*this, backendId, color);
}

void OsgWidget::applyColorToActiveBackendObject(const osg::Vec4& color)
{
	OsgWidgetColorController::applyColorToActiveBackendObject(*this, color);
}

bool OsgWidget::eventFilter(QObject* watched, QEvent* event)
{
	// Escape: never deliver to QWidgetViewer/osgGA. OSG handlers (e.g. StateSetManipulator stack) and
	// unpaired press/release in the event queue have caused apparent "freezes"; Qt may also propagate
	// Esc to parent shortcuts. We only use Esc to leave edit modes; in pure view mode it is a no-op.
	if (watched == m_glWidget
		&& (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease))
	{
		const auto* keyEvent = static_cast<QKeyEvent*>(event);
		if (keyEvent->key() == Qt::Key_Escape)
		{
			if (event->type() == QEvent::KeyPress)
			{
				if (m_objectSelectionMode)
				{
					m_dragging = false;
					m_rotating = false;
					m_dragAxis = DragAxis::None;
					setSelectionActive(false);
					setObjectSelectionMode(false);
					emit selectionCanceledByEsc();
					if (m_glWidget)
					{
						m_glWidget->update();
					}
					return true;
				}
				if (m_pointPickMode)
				{
					emit selectionCanceledByEsc();
					if (m_glWidget)
					{
						m_glWidget->update();
					}
					return true;
				}
				if (m_meshLinePickMode || m_meshFacePickMode)
				{
					m_meshLinePickMode = false;
					m_meshFacePickMode = false;
					hideMeshElementHighlight();
					emit selectionCanceledByEsc();
					if (m_glWidget)
					{
						m_glWidget->update();
					}
					return true;
				}
			}
			return true;
		}
	}

	if (m_pointPickOperation && m_pointPickOperation->handleEvent(watched, event))
	{
		return true;
	}

	if (m_meshElementPickOperation && m_meshElementPickOperation->handleEvent(watched, event))
	{
		return true;
	}

	if (m_objectTransformOperation && m_objectTransformOperation->handleEvent(watched, event))
	{
		return true;
	}

	return QWidget::eventFilter(watched, event);
}

void OsgWidget::clearImportedContent()
{
	clearStagingGeometry();
	hideMeshElementHighlight();
	if (m_backendObjectsGroup.valid())
	{
		m_backendObjectsGroup->removeChildren(0, m_backendObjectsGroup->getNumChildren());
	}
	if (m_robotAssemblyGroup.valid())
	{
		m_robotAssemblyGroup->removeChildren(0, m_robotAssemblyGroup->getNumChildren());
	}
	if (m_trajectoryOverlayGroup.valid())
	{
		m_trajectoryOverlayGroup->removeChildren(0, m_trajectoryOverlayGroup->getNumChildren());
	}
	clearPointAnnotations();
	m_litMeshBackendIds.clear();
	m_backendObjectRoots.clear();
	m_backendParentIds.clear();
	m_backendModelCenters.clear();
	m_backendVisibility.clear();
	m_hasLastSelectionPose = false;
	m_activeBackendId.clear();
	m_activeBackendOuterPat = nullptr;
	m_pickablePointsLocal.clear();
	m_pickablePointsPreviewLocal.clear();
	m_pickablePointsCenteredLocal.clear();
	m_kdNodes.clear();
	m_kdRoot = -1;
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
	}
}

bool OsgWidget::captureImportedPointCloudBackend(PointCloudBackendData& out, QString* errorMessage)
{
	return m_captureController
		? m_captureController->captureImportedPointCloudBackend(*this, out, errorMessage)
		: false;
}

bool OsgWidget::captureImportedMeshBackend(MeshBackendData& out, QString* errorMessage)
{
	return m_captureController
		? m_captureController->captureImportedMeshBackend(*this, out, errorMessage)
		: false;
}

bool OsgWidget::captureImportedMeshBackendHierarchy(std::vector<MeshCapturedPart>& outParts, QString* errorMessage)
{
	return m_captureController
		? m_captureController->captureImportedMeshBackendHierarchy(*this, outParts, errorMessage)
		: false;
}

bool OsgWidget::loadPointCloudFromBackendData(const PointCloudBackendData& data, QString* errorMessage, bool resetViewToHome)
{
	return m_backendLoadController
		? m_backendLoadController->loadPointCloudFromBackendData(*this, data, errorMessage, resetViewToHome)
		: false;
}

bool OsgWidget::loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome,
	bool showWireOutline, bool useSceneLighting)
{
	return m_backendLoadController
		? m_backendLoadController->loadMeshFromBackendData(*this, data, errorMessage, resetViewToHome, showWireOutline,
			  useSceneLighting)
		: false;
}

bool OsgWidget::isBackendMeshLit(const std::string& backendId) const
{
	return m_litMeshBackendIds.find(backendId) != m_litMeshBackendIds.end();
}

// 【中文】添加层级化机器人场景图（动态层级法）
QString OsgWidget::addHierarchicalRobotScene(osg::Group* robotAssembly, const QString& displayName)
{
	if (!robotAssembly || !m_robotAssemblyGroup.valid())
	{
		return QString();
	}

	// 【中文】生成唯一的后端 ID
	static int s_robotSceneCounter = 0;
	const QString backendId = QStringLiteral("RobotScene_%1_%2")
		.arg(displayName.isEmpty() ? QStringLiteral("URDF") : displayName)
		.arg(++s_robotSceneCounter);
	const std::string stdId = backendId.toStdString();

	// 【中文】与其它后端一致：用 PAT 包裹根节点，便于 m_backendObjectRoots 类型统一及变换/可见性
	osg::ref_ptr<osg::PositionAttitudeTransform> outer = new osg::PositionAttitudeTransform;
	outer->setName(displayName.isEmpty() ? "RobotHierarchy" : displayName.toStdString());
	robotAssembly->dirtyBound();
	outer->addChild(robotAssembly);
	m_robotAssemblyGroup->addChild(outer.get());
	outer->dirtyBound();
	const osg::BoundingSphere bs = outer->getBound();
	if (bs.valid())
	{
		m_backendModelCenters[stdId] = osg::Vec3f(bs.center());
		if (m_activeBackendId.empty())
		{
			m_activeModelDiagonal = (std::max)(1.0f, bs.radius() * 2.0f);
		}
	}
	else
	{
		m_backendModelCenters[stdId] = osg::Vec3f(0.0f, 0.0f, 0.0f);
	}
	if (m_backendVisibility.find(stdId) == m_backendVisibility.end())
	{
		m_backendVisibility[stdId] = true;
	}
	applyVisibilityMaskForBackend(stdId);

	// 【中文】禁止对 map 使用 operator[] 赋值 ref_ptr（MSVC + OSG 3.6.5 在 ref_ptr::assign 上 C2440）
	m_backendObjectRoots.insert(std::make_pair(stdId, std::move(outer)));
	m_litMeshBackendIds.insert(stdId);

	// 【中文】刷新场景并对准相机（与 upsertMeshBranchInScene 一致，否则易停留在默认视角看不到模型）
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		focusCameraOnBackend(stdId);
	}
	if (m_glWidget)
	{
		m_glWidget->update();
	}

	return backendId;
}

// 【中文】移除层级化机器人场景图
void OsgWidget::removeHierarchicalRobotScene(const QString& backendId)
{
	if (backendId.isEmpty())
	{
		return;
	}
	const std::string stdId = backendId.toStdString();

	auto it = m_backendObjectRoots.find(stdId);
	if (it != m_backendObjectRoots.end() && it->second.valid() && m_robotAssemblyGroup.valid())
	{
		m_robotAssemblyGroup->removeChild(it->second.get());
		m_backendObjectRoots.erase(it);
	}
	m_litMeshBackendIds.erase(stdId);

	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
	}
}

