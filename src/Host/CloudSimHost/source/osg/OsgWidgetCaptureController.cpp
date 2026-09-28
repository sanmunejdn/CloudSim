/// @file OsgWidgetCaptureController.cpp
/// @brief OsgWidgetCapture 控制

#include "OsgWidgetCaptureController.h"

#include "BackendPoseOsg.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"

#include <string>
#include <unordered_map>

#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/PrimitiveSet>

namespace
{
inline osg::Vec3d mulVertex(const osg::Vec3& v, const osg::Matrixd& m)
{
	return osg::Vec3d(v.x(), v.y(), v.z()) * m;
}

inline osg::Vec3d mulVertex(const osg::Vec3d& v, const osg::Matrixd& m)
{
	return v * m;
}

template <typename VecArray>
void appendTriangle(std::vector<float>& soup, const VecArray* verts, unsigned i0, unsigned i1, unsigned i2,
					const osg::Matrixd& l2w)
{
	const osg::Vec3d a = mulVertex((*verts)[i0], l2w);
	const osg::Vec3d b = mulVertex((*verts)[i1], l2w);
	const osg::Vec3d c = mulVertex((*verts)[i2], l2w);
	soup.push_back(static_cast<float>(a.x()));
	soup.push_back(static_cast<float>(a.y()));
	soup.push_back(static_cast<float>(a.z()));
	soup.push_back(static_cast<float>(b.x()));
	soup.push_back(static_cast<float>(b.y()));
	soup.push_back(static_cast<float>(b.z()));
	soup.push_back(static_cast<float>(c.x()));
	soup.push_back(static_cast<float>(c.y()));
	soup.push_back(static_cast<float>(c.z()));
}

template <typename VecArray>
void triangulatePrimitiveSet(osg::PrimitiveSet* ps, const VecArray* verts, const osg::Matrixd& l2w,
							 std::vector<float>& soup)
{
	if (!ps || !verts)
	{
		return;
	}
	const GLenum mode = ps->getMode();

	if (mode == GL_TRIANGLES)
	{
		osg::DrawArrays* da = dynamic_cast<osg::DrawArrays*>(ps);
		if (da)
		{
			const unsigned first = da->getFirst();
			const unsigned count = da->getCount();
			for (unsigned i = 0; i + 2 < count; i += 3)
			{
				appendTriangle(soup, verts, first + i, first + i + 1, first + i + 2, l2w);
			}
			return;
		}
		osg::DrawElements* de = dynamic_cast<osg::DrawElements*>(ps);
		if (de)
		{
			const unsigned n = de->getNumIndices();
			for (unsigned i = 0; i + 2 < n; i += 3)
			{
				appendTriangle(soup, verts, de->index(i), de->index(i + 1), de->index(i + 2), l2w);
			}
		}
		return;
	}

	if (mode == GL_TRIANGLE_STRIP)
	{
		osg::DrawArrays* da = dynamic_cast<osg::DrawArrays*>(ps);
		if (!da)
		{
			return;
		}
		const unsigned first = da->getFirst();
		const unsigned count = da->getCount();
		for (unsigned i = 0; i + 2 < count; ++i)
		{
			const unsigned a = first + i;
			const unsigned b = first + i + 1;
			const unsigned c = first + i + 2;
			if ((i % 2U) == 0U)
			{
				appendTriangle(soup, verts, a, b, c, l2w);
			}
			else
			{
				appendTriangle(soup, verts, b, a, c, l2w);
			}
		}
		return;
	}

	if (mode == GL_TRIANGLE_FAN)
	{
		osg::DrawArrays* da = dynamic_cast<osg::DrawArrays*>(ps);
		if (!da)
		{
			return;
		}
		const unsigned first = da->getFirst();
		const unsigned count = da->getCount();
		if (count < 3)
		{
			return;
		}
		const unsigned hub = first;
		for (unsigned i = 1; i + 1 < count; ++i)
		{
			appendTriangle(soup, verts, hub, first + i, first + i + 1, l2w);
		}
	}
}

template <typename VecArray>
void extractPointsFromPrimitive(osg::PrimitiveSet* ps, const VecArray* verts, const osg::Matrixd& l2w,
								const osg::Vec4Array* colors, osg::Geometry::AttributeBinding colorBind,
								std::vector<float>& xyz, std::vector<float>& rgba)
{
	if (!ps || !verts || ps->getMode() != GL_POINTS)
	{
		return;
	}
	const bool useVertexColor = colors && colorBind == osg::Geometry::BIND_PER_VERTEX;

	osg::DrawArrays* da = dynamic_cast<osg::DrawArrays*>(ps);
	if (da)
	{
		const unsigned first = da->getFirst();
		const unsigned count = da->getCount();
		for (unsigned i = 0; i < count; ++i)
		{
			const unsigned vi = first + i;
			const osg::Vec3d p = mulVertex((*verts)[vi], l2w);
			xyz.push_back(static_cast<float>(p.x()));
			xyz.push_back(static_cast<float>(p.y()));
			xyz.push_back(static_cast<float>(p.z()));
			if (useVertexColor && colors && vi < colors->size())
			{
				const osg::Vec4& c = (*colors)[vi];
				rgba.push_back(c.r());
				rgba.push_back(c.g());
				rgba.push_back(c.b());
				rgba.push_back(c.a());
			}
		}
		return;
	}

	osg::DrawElements* de = dynamic_cast<osg::DrawElements*>(ps);
	if (de)
	{
		const unsigned n = de->getNumIndices();
		for (unsigned i = 0; i < n; ++i)
		{
			const unsigned vi = de->index(i);
			const osg::Vec3d p = mulVertex((*verts)[vi], l2w);
			xyz.push_back(static_cast<float>(p.x()));
			xyz.push_back(static_cast<float>(p.y()));
			xyz.push_back(static_cast<float>(p.z()));
			if (useVertexColor && colors && vi < colors->size())
			{
				const osg::Vec4& c = (*colors)[vi];
				rgba.push_back(c.r());
				rgba.push_back(c.g());
				rgba.push_back(c.b());
				rgba.push_back(c.a());
			}
		}
	}
}

template <typename VecArray>
void processGeometryForMesh(osg::Geometry* geom, const osg::Matrixd& l2w, std::vector<float>& soup)
{
	if (!geom)
	{
		return;
	}
	const VecArray* verts = dynamic_cast<VecArray*>(geom->getVertexArray());
	if (!verts)
	{
		return;
	}
	for (unsigned int pi = 0; pi < geom->getNumPrimitiveSets(); ++pi)
	{
		triangulatePrimitiveSet(geom->getPrimitiveSet(pi), verts, l2w, soup);
	}
}

template <typename VecArray>
void processGeometryForPoints(osg::Geometry* geom, const osg::Matrixd& l2w, std::vector<float>& xyz,
							  std::vector<float>& rgba)
{
	if (!geom)
	{
		return;
	}
	const VecArray* verts = dynamic_cast<VecArray*>(geom->getVertexArray());
	if (!verts)
	{
		return;
	}
	const osg::Vec4Array* colors = dynamic_cast<const osg::Vec4Array*>(geom->getColorArray());
	const osg::Geometry::AttributeBinding cb = geom->getColorBinding();
	for (unsigned int pi = 0; pi < geom->getNumPrimitiveSets(); ++pi)
	{
		extractPointsFromPrimitive(geom->getPrimitiveSet(pi), verts, l2w, colors, cb, xyz, rgba);
	}
}

void collectAllDrawableVerticesToFloats(osg::Node* node, std::vector<float>& xyz, const osg::Matrixd& stagingRootWorld)
{
	if (!node)
	{
		return;
	}
	struct AllVertsVisitor : osg::NodeVisitor
	{
		std::vector<float>& out;
		const osg::Matrixd& stagingWorld;
		explicit AllVertsVisitor(std::vector<float>& o, const osg::Matrixd& stagingW)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), out(o), stagingWorld(stagingW)
		{
		}
		void apply(osg::Geode& geode) override
		{
			const osg::Matrixd l2w = osg::computeLocalToWorld(getNodePath());
			const osg::Matrixd geomToStaging = l2w * osg::Matrixd::inverse(stagingWorld);
			for (unsigned int di = 0; di < geode.getNumDrawables(); ++di)
			{
				osg::Geometry* geom = geode.getDrawable(di) ? geode.getDrawable(di)->asGeometry() : nullptr;
				if (!geom || !geom->getVertexArray())
				{
					continue;
				}
				const osg::Vec3Array* va = dynamic_cast<osg::Vec3Array*>(geom->getVertexArray());
				if (va)
				{
					out.reserve(out.size() + va->size() * 3U);
					for (const osg::Vec3& v : *va)
					{
						const osg::Vec3d p = osg::Vec3d(v.x(), v.y(), v.z()) * geomToStaging;
						out.push_back(static_cast<float>(p.x()));
						out.push_back(static_cast<float>(p.y()));
						out.push_back(static_cast<float>(p.z()));
					}
					continue;
				}
				const osg::Vec3dArray* vd = dynamic_cast<osg::Vec3dArray*>(geom->getVertexArray());
				if (vd)
				{
					out.reserve(out.size() + vd->size() * 3U);
					for (const osg::Vec3d& v : *vd)
					{
						const osg::Vec3d p = v * geomToStaging;
						out.push_back(static_cast<float>(p.x()));
						out.push_back(static_cast<float>(p.y()));
						out.push_back(static_cast<float>(p.z()));
					}
				}
			}
			traverse(geode);
		}
	} visitor(xyz, stagingRootWorld);
	node->accept(visitor);
}

osg::Matrixd stagingRootWorldMatrix(OsgWidget& self, osg::Node* src)
{
	osg::NodePath path;
	if (self.m_stagingGroup.valid())
	{
		path.push_back(self.m_stagingGroup.get());
	}
	if (src)
	{
		path.push_back(src);
	}
	return path.empty() ? osg::Matrixd::identity() : osg::computeLocalToWorld(path);
}

} // namespace

bool OsgWidgetCaptureController::captureImportedPointCloudBackend(OsgWidget& self, PointCloudBackendData& out,
																  QString* errorMessage)
{
	osg::Node* src = self.stagingGeometryRoot();
	if (!src)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("No imported geometry.");
		}
		return false;
	}
	std::vector<float> xyz;
	std::vector<float> rgba;
	const osg::Matrixd stagingRootWorld = stagingRootWorldMatrix(self, src);
	struct PointCloudCaptureVisitor : osg::NodeVisitor
	{
		std::vector<float>& xyzR;
		std::vector<float>& rgbaR;
		const osg::Matrixd& stagingWorld;
		PointCloudCaptureVisitor(std::vector<float>& x, std::vector<float>& r, const osg::Matrixd& stagingW)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), xyzR(x), rgbaR(r), stagingWorld(stagingW)
		{
		}
		void apply(osg::Geode& geode) override
		{
			const osg::Matrixd l2w = osg::computeLocalToWorld(getNodePath());
			const osg::Matrixd geomToStaging = l2w * osg::Matrixd::inverse(stagingWorld);
			for (unsigned int di = 0; di < geode.getNumDrawables(); ++di)
			{
				osg::Geometry* geom = geode.getDrawable(di) ? geode.getDrawable(di)->asGeometry() : nullptr;
				if (!geom)
				{
					continue;
				}
				processGeometryForPoints<osg::Vec3Array>(geom, geomToStaging, xyzR, rgbaR);
				processGeometryForPoints<osg::Vec3dArray>(geom, geomToStaging, xyzR, rgbaR);
			}
			traverse(geode);
		}
	} visitor(xyz, rgba, stagingRootWorld);
	src->accept(visitor);
	if (rgba.size() != xyz.size() / 3U * 4U)
	{
		rgba.clear();
	}
	// OSG readers often expose point clouds as vertex arrays with non-GL_POINTS primitives;
	// pickable cache already flattened all drawable vertices in the same coordinate frame.
	if (xyz.empty() && !self.m_pickablePointsLocal.empty())
	{
		xyz.reserve(self.m_pickablePointsLocal.size() * 3U);
		for (const osg::Vec3f& p : self.m_pickablePointsLocal)
		{
			xyz.push_back(p.x());
			xyz.push_back(p.y());
			xyz.push_back(p.z());
		}
		rgba.clear();
	}
	if (xyz.empty() && src)
	{
		collectAllDrawableVerticesToFloats(src, xyz, stagingRootWorld);
		rgba.clear();
	}
	if (xyz.empty())
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral("No point positions found to store (no GL_POINTS and no pickable vertices).");
		}
		return false;
	}
	out.setPointBuffers(std::move(xyz), std::move(rgba));
	out.setWorldMatrix(backend_pose_osg::backendWorldMatrixFromOsgMatrix(stagingRootWorld));
	return true;
}

bool OsgWidgetCaptureController::capturePointCloudBackendFromScene(OsgWidget& self, const std::string& backendId,
																   PointCloudBackendData& out, QString* errorMessage)
{
	const auto it = self.m_backendObjectRoots.find(backendId);
	if (it == self.m_backendObjectRoots.end() || !it->second.valid())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("No backend visual branch for point cloud.");
		}
		return false;
	}

	osg::Node* src = it->second.get();
	std::vector<float> xyz;
	std::vector<float> rgba;
	const osg::Matrixd identity;
	struct BackendPointCaptureVisitor : osg::NodeVisitor
	{
		std::vector<float>& xyzR;
		std::vector<float>& rgbaR;
		const osg::Matrixd& localM;
		BackendPointCaptureVisitor(std::vector<float>& x, std::vector<float>& r, const osg::Matrixd& m)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), xyzR(x), rgbaR(r), localM(m)
		{
		}
		void apply(osg::Geode& geode) override
		{
			for (unsigned int di = 0; di < geode.getNumDrawables(); ++di)
			{
				osg::Geometry* geom = geode.getDrawable(di) ? geode.getDrawable(di)->asGeometry() : nullptr;
				if (!geom)
				{
					continue;
				}
				processGeometryForPoints<osg::Vec3Array>(geom, localM, xyzR, rgbaR);
				processGeometryForPoints<osg::Vec3dArray>(geom, localM, xyzR, rgbaR);
			}
			traverse(geode);
		}
	} visitor(xyz, rgba, identity);
	src->accept(visitor);
	if (rgba.size() != xyz.size() / 3U * 4U)
	{
		rgba.clear();
	}
	if (xyz.empty())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("No point positions found in backend visual branch.");
		}
		return false;
	}
	out.setPointBuffers(std::move(xyz), std::move(rgba));
	return true;
}

bool OsgWidgetCaptureController::captureImportedMeshBackend(OsgWidget& self, MeshBackendData& out,
															QString* errorMessage)
{
	osg::Node* src = self.stagingGeometryRoot();
	if (!src)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("No imported geometry.");
		}
		return false;
	}
	std::vector<float> soup;
	struct MeshCaptureVisitor : osg::NodeVisitor
	{
		std::vector<float>& soupR;
		MeshCaptureVisitor(std::vector<float>& s) : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), soupR(s)
		{
		}
		void apply(osg::Geode& geode) override
		{
			const osg::Matrixd l2w = osg::computeLocalToWorld(getNodePath());
			for (unsigned int di = 0; di < geode.getNumDrawables(); ++di)
			{
				osg::Geometry* geom = geode.getDrawable(di) ? geode.getDrawable(di)->asGeometry() : nullptr;
				if (!geom)
				{
					continue;
				}
				processGeometryForMesh<osg::Vec3Array>(geom, l2w, soupR);
				processGeometryForMesh<osg::Vec3dArray>(geom, l2w, soupR);
			}
			traverse(geode);
		}
	} visitor(soup);
	src->accept(visitor);
	if (soup.empty())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("No triangle geometry found to store.");
		}
		return false;
	}
	out.setTriangleSoup(std::move(soup));
	return true;
}

bool OsgWidgetCaptureController::captureImportedMeshBackendHierarchy(OsgWidget& self,
																	 std::vector<MeshCapturedPart>& outParts,
																	 QString* errorMessage)
{
	outParts.clear();
	osg::Node* src = self.stagingGeometryRoot();
	if (!src)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("No imported geometry.");
		}
		return false;
	}

	struct HierarchyVisitor : osg::NodeVisitor
	{
		std::vector<MeshCapturedPart>& parts;
		std::unordered_map<std::string, std::size_t> keyToIndex;
		int autoNameCounter = 1;

		explicit HierarchyVisitor(std::vector<MeshCapturedPart>& out)
			: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), parts(out)
		{
		}

		static std::string nodeNameOrAuto(const osg::Node* node, int& autoCounter)
		{
			if (!node)
			{
				return {};
			}
			const std::string n = node->getName();
			if (!n.empty())
			{
				return n;
			}
			return std::string("node_") + std::to_string(autoCounter++);
		}

		void apply(osg::Geode& geode) override
		{
			const osg::Matrixd l2w = osg::computeLocalToWorld(getNodePath());
			std::vector<float> soup;
			for (unsigned int di = 0; di < geode.getNumDrawables(); ++di)
			{
				osg::Geometry* geom = geode.getDrawable(di) ? geode.getDrawable(di)->asGeometry() : nullptr;
				if (!geom)
				{
					continue;
				}
				processGeometryForMesh<osg::Vec3Array>(geom, l2w, soup);
				processGeometryForMesh<osg::Vec3dArray>(geom, l2w, soup);
			}
			if (soup.empty())
			{
				traverse(geode);
				return;
			}

			std::vector<std::string> tokens;
			tokens.reserve(getNodePath().size());
			for (osg::Node* n : getNodePath())
			{
				if (!n)
				{
					continue;
				}
				tokens.push_back(nodeNameOrAuto(n, autoNameCounter));
			}
			if (tokens.empty())
			{
				tokens.push_back(nodeNameOrAuto(&geode, autoNameCounter));
			}

			std::string partKey;
			for (std::size_t i = 0; i < tokens.size(); ++i)
			{
				if (i > 0)
				{
					partKey += "/";
				}
				partKey += tokens[i];
			}
			std::string parentKey;
			if (tokens.size() > 1)
			{
				for (std::size_t i = 0; i + 1 < tokens.size(); ++i)
				{
					if (i > 0)
					{
						parentKey += "/";
					}
					parentKey += tokens[i];
				}
			}

			auto it = keyToIndex.find(partKey);
			if (it == keyToIndex.end())
			{
				MeshCapturedPart part;
				part.partPath = QString::fromStdString(partKey);
				part.parentPartPath = QString::fromStdString(parentKey);
				part.displayName = QString::fromStdString(tokens.back());
				part.triangleSoup = std::move(soup);
				keyToIndex.emplace(partKey, parts.size());
				parts.push_back(std::move(part));
			}
			else
			{
				std::vector<float>& dst = parts[it->second].triangleSoup;
				dst.insert(dst.end(), soup.begin(), soup.end());
			}
			traverse(geode);
		}
	} visitor(outParts);

	src->accept(visitor);
	if (outParts.empty())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("No triangle geometry found to store.");
		}
		return false;
	}
	return true;
}
