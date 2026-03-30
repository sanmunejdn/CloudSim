#include "OsgWidget.h"

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
	OsgScene::syncGizmoAndPickFromPointCloudBackend(data);
	cacheSelectionPoseFromSelectedTransform();
	OsgWidgetTransformHierarchyController::finalizeSelectionSync(*this);
}

void OsgWidget::syncSelectionFromBackend(const MeshBackendData& data)
{
	OsgScene::syncGizmoAndPickFromMeshBackend(data);
	cacheSelectionPoseFromSelectedTransform();
	OsgWidgetTransformHierarchyController::finalizeSelectionSync(*this);
}

void OsgWidget::syncSelectionForBackendId(const std::string& backendId)
{
	OsgWidgetTransformHierarchyController::syncSelectionForBackendId(*this, backendId);
}

osg::ref_ptr<osg::Geode> OsgWidget::buildPointCloudGeode(const PointCloudBackendData& data, QString* errorMessage) const
{
	const std::vector<float>& xyz = data.pointPositionsXyz();
	if (xyz.size() < 3U || (xyz.size() % 3U) != 0U)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Invalid point buffer in backend data.");
		}
		return nullptr;
	}
	osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
	points->reserve(xyz.size() / 3U);
	for (std::size_t i = 0; i + 2 < xyz.size(); i += 3)
	{
		points->push_back(osg::Vec3(xyz[i], xyz[i + 1], xyz[i + 2]));
	}
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(points.get());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points->size())));
	const std::vector<float>& rgba = data.pointVertexRgba();
	if (data.hasPerVertexColors() && rgba.size() == xyz.size() / 3U * 4U)
	{
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->reserve(rgba.size() / 4U);
		for (std::size_t i = 0; i + 3 < rgba.size(); i += 4)
		{
			colors->push_back(osg::Vec4(rgba[i], rgba[i + 1], rgba[i + 2], rgba[i + 3]));
		}
		geometry->setColorArray(colors.get(), osg::Array::BIND_PER_VERTEX);
	}
	else
	{
		const BackendColor c = data.color();
		osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
		colors->push_back(osg::Vec4(c.r, c.g, c.b, c.a));
		geometry->setColorArray(colors.get(), osg::Array::BIND_OVERALL);
	}
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geometry.get());
	geode->getOrCreateStateSet()->setAttribute(new osg::Point(2.0f));
	return geode;
}

bool OsgWidget::upsertPointCloudBranchInScene(const PointCloudBackendData& data, QString* errorMessage, bool resetViewToHome)
{
	osg::ref_ptr<osg::Geode> geode = buildPointCloudGeode(data, errorMessage);
	if (!geode)
	{
		return false;
	}
	const std::string id = data.id();
	const osg::Vec3f center = computePointCloudCenterFromXyz(data.pointPositionsXyz());
	const float diagonal = computePointCloudDiagonalFromXyz(data.pointPositionsXyz());
	osg::ref_ptr<osg::PositionAttitudeTransform> inner = new osg::PositionAttitudeTransform;
	inner->setPosition(-center);
	inner->addChild(geode.get());
	const BackendVec3 p = data.pose();
	const BackendVec3 r = data.rotation();
	osg::ref_ptr<osg::PositionAttitudeTransform> outer = new osg::PositionAttitudeTransform;
	const osg::Vec3f pose(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
	outer->setPosition(center + pose);
	outer->setAttitude(OsgScene::eulerDegToQuat(osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z))));
	outer->addChild(inner.get());
	osg::StateSet* oss = outer->getOrCreateStateSet();
	oss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid() && m_objectsGroup.valid())
	{
		m_objectsGroup->removeChild(it->second.get());
	}
	m_backendObjectRoots[id] = outer;
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
	m_objectsGroup->addChild(outer.get());
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		if (resetViewToHome)
		{
			m_viewer->home();
		}
	}
	return true;
}

namespace {

static inline int64_t quantVertexCoord(float v)
{
	return static_cast<int64_t>(std::llround(static_cast<double>(v) * 1000000.0));
}

struct WeldKey
{
	int64_t x, y, z;
	bool operator==(const WeldKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct WeldKeyHash
{
	size_t operator()(const WeldKey& k) const noexcept
	{
		size_t h = 1469598103934665603ull;
		h ^= static_cast<size_t>(k.x);
		h *= 1099511628211ull;
		h ^= static_cast<size_t>(k.y);
		h *= 1099511628211ull;
		h ^= static_cast<size_t>(k.z);
		h *= 1099511628211ull;
		return h;
	}
};

struct EdgeIdxKey
{
	std::uint32_t lo, hi;
	bool operator==(const EdgeIdxKey& o) const { return lo == o.lo && hi == o.hi; }
};

struct EdgeIdxKeyHash
{
	size_t operator()(const EdgeIdxKey& k) const noexcept
	{
		return (static_cast<size_t>(k.lo) << 32) | static_cast<size_t>(k.hi);
	}
};

/// 按「共面连通面片」建外轮廓：同一面片内三角共享边出现 2 次（内部剖分线）不画；仅出现 1 次的边为该片外边界（含与邻面/网格开口的边）。
osg::ref_ptr<osg::Geometry> buildMeshOutlineWireGeometry(const std::vector<float>& soup, const osg::Vec4& fillColor)
{
	const std::size_t nTri = soup.size() / 9U;
	if (nTri == 0U)
	{
		return nullptr;
	}
	std::unordered_map<WeldKey, std::uint32_t, WeldKeyHash> weld;
	weld.reserve(nTri * 2U);
	std::vector<osg::Vec3> verts;
	verts.reserve(nTri * 2U);
	std::vector<std::array<std::uint32_t, 3>> triIdx(nTri);

	auto addVertex = [&](float px, float py, float pz) -> std::uint32_t
	{
		const WeldKey k{ quantVertexCoord(px), quantVertexCoord(py), quantVertexCoord(pz) };
		const auto it = weld.find(k);
		if (it != weld.end())
		{
			return it->second;
		}
		const std::uint32_t id = static_cast<std::uint32_t>(verts.size());
		verts.emplace_back(px, py, pz);
		weld.emplace(k, id);
		return id;
	};

	for (std::size_t t = 0; t < nTri; ++t)
	{
		const std::size_t b = t * 9U;
		triIdx[t][0] = addVertex(soup[b + 0], soup[b + 1], soup[b + 2]);
		triIdx[t][1] = addVertex(soup[b + 3], soup[b + 4], soup[b + 5]);
		triIdx[t][2] = addVertex(soup[b + 6], soup[b + 7], soup[b + 8]);
	}

	std::unordered_map<EdgeIdxKey, std::vector<std::uint32_t>, EdgeIdxKeyHash> edgeTris;
	edgeTris.reserve(nTri * 2U);

	auto addEdge = [&](std::uint32_t a, std::uint32_t b, std::uint32_t triId)
	{
		if (a == b)
		{
			return;
		}
		const EdgeIdxKey ek{ std::min(a, b), std::max(a, b) };
		edgeTris[ek].push_back(triId);
	};

	for (std::size_t t = 0; t < nTri; ++t)
	{
		const auto& tr = triIdx[t];
		const auto tid = static_cast<std::uint32_t>(t);
		addEdge(tr[0], tr[1], tid);
		addEdge(tr[1], tr[2], tid);
		addEdge(tr[2], tr[0], tid);
	}

	std::vector<osg::Vec3f> triNor(nTri);
	for (std::size_t t = 0; t < nTri; ++t)
	{
		const osg::Vec3& p0 = verts[triIdx[t][0]];
		const osg::Vec3& p1 = verts[triIdx[t][1]];
		const osg::Vec3& p2 = verts[triIdx[t][2]];
		const osg::Vec3 e1 = p1 - p0;
		const osg::Vec3 e2 = p2 - p0;
		osg::Vec3 n = e1 ^ e2;
		const float len2 = n.length2();
		if (len2 > 1e-20f)
		{
			n.normalize();
			triNor[t] = osg::Vec3f(n.x(), n.y(), n.z());
		}
		else
		{
			triNor[t] = osg::Vec3f(0.0f, 0.0f, 1.0f);
		}
	}

	float minx = verts[0].x(), maxx = verts[0].x();
	float miny = verts[0].y(), maxy = verts[0].y();
	float minz = verts[0].z(), maxz = verts[0].z();
	for (const osg::Vec3& p : verts)
	{
		minx = std::min(minx, p.x());
		maxx = std::max(maxx, p.x());
		miny = std::min(miny, p.y());
		maxy = std::max(maxy, p.y());
		minz = std::min(minz, p.z());
		maxz = std::max(maxz, p.z());
	}
	const float dx = maxx - minx;
	const float dy = maxy - miny;
	const float dz = maxz - minz;
	const float diag = std::sqrt(dx * dx + dy * dy + dz * dz);
	const float planeEps = std::max(1e-7f, diag * 1e-6f);

	auto neighborsOf = [&](std::uint32_t t) -> std::vector<std::uint32_t>
	{
		std::vector<std::uint32_t> nb;
		for (int e = 0; e < 3; ++e)
		{
			const std::uint32_t a = triIdx[t][static_cast<std::size_t>(e)];
			const std::uint32_t b = triIdx[t][static_cast<std::size_t>((e + 1) % 3)];
			const EdgeIdxKey ek{ std::min(a, b), std::max(a, b) };
			const auto it = edgeTris.find(ek);
			if (it == edgeTris.end())
			{
				continue;
			}
			for (const std::uint32_t tid : it->second)
			{
				if (tid != t)
				{
					nb.push_back(tid);
				}
			}
		}
		return nb;
	};

	auto coplanarWithSeed = [&](std::uint32_t seed, std::uint32_t u) -> bool
	{
		const osg::Vec3f& nSeed = triNor[seed];
		const osg::Vec3 p0Seed = verts[triIdx[seed][0]];
		const osg::Vec3f& nu = triNor[u];
		if (std::fabs(static_cast<double>(nSeed.x() * nu.x() + nSeed.y() * nu.y() + nSeed.z() * nu.z())) < 0.998)
		{
			return false;
		}
		for (int k = 0; k < 3; ++k)
		{
			const osg::Vec3& v = verts[triIdx[u][static_cast<std::size_t>(k)]];
			const osg::Vec3 w = v - p0Seed;
			const float d = std::fabs(
				nSeed.x() * static_cast<float>(w.x()) + nSeed.y() * static_cast<float>(w.y()) + nSeed.z() * static_cast<float>(w.z()));
			if (d > planeEps)
			{
				return false;
			}
		}
		return true;
	};

	osg::ref_ptr<osg::Vec3Array> lineVerts = new osg::Vec3Array;
	std::vector<int> patchOf(static_cast<std::size_t>(nTri), -1);

	for (std::size_t seed = 0; seed < nTri; ++seed)
	{
		if (patchOf[seed] != -1)
		{
			continue;
		}
		const int patchId = static_cast<int>(seed);
		std::vector<std::uint32_t> patchTris;
		std::queue<std::uint32_t> q;
		patchOf[seed] = patchId;
		patchTris.push_back(static_cast<std::uint32_t>(seed));
		q.push(static_cast<std::uint32_t>(seed));
		while (!q.empty())
		{
			const std::uint32_t cur = q.front();
			q.pop();
			for (const std::uint32_t u : neighborsOf(cur))
			{
				if (patchOf[u] != -1)
				{
					continue;
				}
				if (!coplanarWithSeed(static_cast<std::uint32_t>(seed), u))
				{
					continue;
				}
				patchOf[u] = patchId;
				patchTris.push_back(u);
				q.push(u);
			}
		}

		std::unordered_map<EdgeIdxKey, int, EdgeIdxKeyHash> edgeCountInPatch;
		for (const std::uint32_t ti : patchTris)
		{
			for (int e = 0; e < 3; ++e)
			{
				const std::uint32_t a = triIdx[ti][static_cast<std::size_t>(e)];
				const std::uint32_t b = triIdx[ti][static_cast<std::size_t>((e + 1) % 3)];
				const EdgeIdxKey ek{ std::min(a, b), std::max(a, b) };
				edgeCountInPatch[ek]++;
			}
		}
		for (const auto& ep : edgeCountInPatch)
		{
			if (ep.second == 1)
			{
				lineVerts->push_back(verts[ep.first.lo]);
				lineVerts->push_back(verts[ep.first.hi]);
			}
		}
	}

	if (lineVerts->empty())
	{
		return nullptr;
	}

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(lineVerts.get());
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(lineVerts->size())));
	osg::ref_ptr<osg::Vec4Array> mcWire = new osg::Vec4Array;
	mcWire->push_back(osg::Vec4(
		std::max(0.12f, fillColor.r() * 0.38f),
		std::max(0.12f, fillColor.g() * 0.38f),
		std::max(0.12f, fillColor.b() * 0.38f),
		fillColor.a()));
	geom->setColorArray(mcWire.get(), osg::Array::BIND_OVERALL);
	return geom;
}

} // namespace

osg::ref_ptr<osg::Node> OsgWidget::buildMeshGeode(const MeshBackendData& data, QString* errorMessage) const
{
	const std::vector<float>& soup = data.triangleSoup();
	if (soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Invalid mesh buffer in backend data.");
		}
		return nullptr;
	}
	osg::ref_ptr<osg::Vec3Array> va = new osg::Vec3Array;
	va->reserve(soup.size() / 3U);
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3)
	{
		va->push_back(osg::Vec3(soup[i], soup[i + 1], soup[i + 2]));
	}
	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(va.get());
	geometry->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(va->size())));
	osg::ref_ptr<osg::Vec4Array> mc = new osg::Vec4Array;
	const BackendColor c = data.color();
	const osg::Vec4 fillColor(c.r, c.g, c.b, c.a);
	mc->push_back(fillColor);
	geometry->setColorArray(mc.get(), osg::Array::BIND_OVERALL);
	osg::ref_ptr<osg::Geode> geodeFill = new osg::Geode;
	geodeFill->addDrawable(geometry.get());

	osg::ref_ptr<osg::Group> grp = new osg::Group;
	grp->addChild(geodeFill.get());
	osg::ref_ptr<osg::Geometry> geometryWire = buildMeshOutlineWireGeometry(soup, fillColor);
	if (geometryWire.valid())
	{
		osg::ref_ptr<osg::Geode> geodeWire = new osg::Geode;
		geodeWire->setName("meshWireOverlay");
		geodeWire->addDrawable(geometryWire.get());
		osg::StateSet* ssWire = geodeWire->getOrCreateStateSet();
		ssWire->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
		ssWire->setAttributeAndModes(new osg::LineWidth(1.0f));
		ssWire->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
		grp->addChild(geodeWire.get());
	}
	return grp;
}

bool OsgWidget::upsertMeshBranchInScene(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome)
{
	osg::ref_ptr<osg::Node> meshRoot = buildMeshGeode(data, errorMessage);
	if (!meshRoot)
	{
		return false;
	}
	const std::string id = data.id();
	const osg::Vec3f center = computeMeshCenterFromSoup(data.triangleSoup());
	const float diagonal = computeMeshDiagonalFromSoup(data.triangleSoup());
	osg::ref_ptr<osg::PositionAttitudeTransform> inner = new osg::PositionAttitudeTransform;
	inner->setPosition(-center);
	inner->addChild(meshRoot.get());
	osg::ref_ptr<osg::PositionAttitudeTransform> outer = new osg::PositionAttitudeTransform;
	const BackendVec3 p = data.pose();
	const BackendVec3 r = data.rotation();
	const osg::Vec3f pose(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
	outer->setPosition(center + pose);
	outer->setAttitude(OsgScene::eulerDegToQuat(osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z))));
	outer->addChild(inner.get());
	osg::StateSet* oss = outer->getOrCreateStateSet();
	oss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid() && m_objectsGroup.valid())
	{
		m_objectsGroup->removeChild(it->second.get());
	}
	m_backendObjectRoots[id] = outer;
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
	m_objectsGroup->addChild(outer.get());
	if (m_viewer.valid())
	{
		m_viewer->setSceneData(m_root.get());
		if (resetViewToHome)
		{
			m_viewer->home();
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
	m_viewer->setSceneData(m_root.get());
	m_viewer->addEventHandler(new osgGA::StateSetManipulator(m_viewer->getCamera()->getOrCreateStateSet()));

	m_trackballManipulator = new osgGA::TrackballManipulator;
	m_trackballManipulator->setAllowThrow(false);
	m_viewer->setCameraManipulator(m_trackballManipulator.get());
	m_viewer->getEventQueue()->syncWindowRectangleWithGraphicsContext();

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
		}
		updateWorldAxesHudViewport(w, h);
	});

	m_viewer->getCamera()->setGraphicsContext(m_graphicsWindow.get());
	m_viewer->getCamera()->setViewport(0, 0, m_glWidget->width(), m_glWidget->height());
	m_viewer->getCamera()->setCullMask(0xffffffffu);
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

void OsgWidget::refreshCompassDrawVisibility()
{
	OsgScene::refreshCompassDrawVisibility();
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
	if (m_objectsGroup.valid())
	{
		m_objectsGroup->removeChildren(0, m_objectsGroup->getNumChildren());
	}
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

bool OsgWidget::loadMeshFromBackendData(const MeshBackendData& data, QString* errorMessage, bool resetViewToHome)
{
	return m_backendLoadController
		? m_backendLoadController->loadMeshFromBackendData(*this, data, errorMessage, resetViewToHome)
		: false;
}

