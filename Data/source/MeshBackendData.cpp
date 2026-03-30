#include "MeshBackendData.h"

#include "BackendObjectAttribute.h"
#include "BackendPropertyRow.h"
#include "geometry_base64.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <unordered_set>
#include <CGAL/IO/polygon_soup_io.h>
#include <CGAL/Simple_cartesian.h>

#include "dl_creationadapter.h"
#include "dl_dxf.h"

#include <sstream>
#include <string>
#include <vector>

#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>

#include <gp_Pnt.hxx>

MeshBackendData::MeshBackendData()
{
	setName("Model");
	BackendColor c;
	c.r = 0.65f;
	c.g = 0.82f;
	c.b = 0.95f;
	c.a = 1.0f;
	m_color = c;

	// Reuse common backend attributes (pose/rotation/color) so property inspector editing
	// works the same for all backend types.
	m_attributes.push_back(std::make_shared<BackendPoseAttribute>());
	m_attributes.push_back(std::make_shared<BackendRotationAttribute>());
	m_attributes.push_back(std::make_shared<BackendDisplayColorAttribute>());
}

std::string MeshBackendData::className() const
{
	return "Model";
}

bool MeshBackendData::hasGeometry() const
{
	return !m_triangleSoup.empty();
}

BackendBoundingBox MeshBackendData::geometryBounds() const
{
	return m_bounds;
}

std::size_t MeshBackendData::geometryElementCount() const
{
	return m_triangleSoup.size() / 9U;
}

void MeshBackendData::clearGeometry()
{
	m_triangleSoup.clear();
	m_bounds = BackendBoundingBox{};
}

void MeshBackendData::setPose(const BackendVec3& position)
{
	m_position = position;
}

BackendVec3 MeshBackendData::pose() const
{
	return m_position;
}

void MeshBackendData::setRotation(const BackendVec3& eulerDeg)
{
	m_rotation = eulerDeg;
}

BackendVec3 MeshBackendData::rotation() const
{
	return m_rotation;
}

void MeshBackendData::setColor(const BackendColor& color)
{
	m_color = color;
}

BackendColor MeshBackendData::color() const
{
	return m_color;
}

nlohmann::json MeshBackendData::snapshotPropertyRows() const
{
	nlohmann::json rows = BackendDataBase::snapshotPropertyRows();
	for (const auto& attr : m_attributes)
	{
		attr->appendRows(*this, rows);
	}

	backend_property_json::appendRow(
		rows, "mesh.triangle_count", "Triangles", false, std::to_string(geometryElementCount()));
	return rows;
}

bool MeshBackendData::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg)
{
	for (auto& attr : m_attributes)
	{
		if (!attr->handlesKey(*this, key))
		{
			continue;
		}
		if (attr->apply(*this, key, value, errMsg))
		{
			return true;
		}
		return false;
	}
	return BackendDataBase::applyPropertyChange(key, value, errMsg);
}

void MeshBackendData::setTriangleSoup(std::vector<float> xyzPerTriangleVertex)
{
	if (xyzPerTriangleVertex.size() % 9U != 0U)
	{
		clearGeometry();
		return;
	}
	m_triangleSoup = std::move(xyzPerTriangleVertex);
	recomputeBounds();
}

void MeshBackendData::recomputeBounds()
{
	m_bounds = BackendBoundingBox{};
	if (m_triangleSoup.size() < 3U)
	{
		return;
	}
	double minX = m_triangleSoup[0];
	double minY = m_triangleSoup[1];
	double minZ = m_triangleSoup[2];
	double maxX = minX;
	double maxY = minY;
	double maxZ = minZ;
	for (std::size_t i = 0; i + 2 < m_triangleSoup.size(); i += 3)
	{
		const double x = m_triangleSoup[i];
		const double y = m_triangleSoup[i + 1];
		const double z = m_triangleSoup[i + 2];
		minX = std::min(minX, x);
		minY = std::min(minY, y);
		minZ = std::min(minZ, z);
		maxX = std::max(maxX, x);
		maxY = std::max(maxY, y);
		maxZ = std::max(maxZ, z);
	}
	m_bounds.min.x = minX;
	m_bounds.min.y = minY;
	m_bounds.min.z = minZ;
	m_bounds.max.x = maxX;
	m_bounds.max.y = maxY;
	m_bounds.max.z = maxZ;
	m_bounds.valid = true;
}

bool MeshBackendData::writeProjectEmbeddedGeometry(std::string& outTriangleSoupBase64) const
{
	outTriangleSoupBase64.clear();
	if (m_triangleSoup.empty() || (m_triangleSoup.size() % 9U) != 0U)
	{
		return false;
	}
	outTriangleSoupBase64 = geometryBase64EncodeFloats(m_triangleSoup);
	return !outTriangleSoupBase64.empty();
}

bool MeshBackendData::readProjectEmbeddedGeometry(const std::string& triangleSoupBase64)
{
	std::vector<float> soup;
	if (!geometryBase64DecodeFloats(triangleSoupBase64, soup) || soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		return false;
	}
	setTriangleSoup(std::move(soup));
	return !m_triangleSoup.empty();
}

namespace {

void meshLoadErr(std::string* errMsg, const char* text)
{
	if (errMsg)
	{
		*errMsg = text;
	}
}

std::string meshLowerExtension(const std::string& path)
{
	const auto dot = path.find_last_of('.');
	if (dot == std::string::npos)
	{
		return {};
	}
	std::string ext = path.substr(dot + 1);
	for (char& c : ext)
	{
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	return ext;
}

static void meshPushTri(std::vector<float>& soup, double ax, double ay, double az, double bx, double by, double bz,
	double cx, double cy, double cz)
{
	const double abx = bx - ax;
	const double aby = by - ay;
	const double abz = bz - az;
	const double acx = cx - ax;
	const double acy = cy - ay;
	const double acz = cz - az;
	const double nx = aby * acz - abz * acy;
	const double ny = abz * acx - abx * acz;
	const double nz = abx * acy - aby * acx;
	if (nx * nx + ny * ny + nz * nz < 1e-30)
	{
		return;
	}
	soup.push_back(static_cast<float>(ax));
	soup.push_back(static_cast<float>(ay));
	soup.push_back(static_cast<float>(az));
	soup.push_back(static_cast<float>(bx));
	soup.push_back(static_cast<float>(by));
	soup.push_back(static_cast<float>(bz));
	soup.push_back(static_cast<float>(cx));
	soup.push_back(static_cast<float>(cy));
	soup.push_back(static_cast<float>(cz));
}

static void meshAddTraceTriangles(std::vector<float>& soup, const DL_TraceData& d)
{
	const double* x = d.x;
	const double* y = d.y;
	const double* z = d.z;
	auto same = [&](int a, int b) {
		const double dx = x[a] - x[b];
		const double dy = y[a] - y[b];
		const double dz = z[a] - z[b];
		return dx * dx + dy * dy + dz * dz < 1e-24;
	};
	if (same(2, 3))
	{
		meshPushTri(soup, x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
	}
	else
	{
		meshPushTri(soup, x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
		meshPushTri(soup, x[0], y[0], z[0], x[2], y[2], z[2], x[3], y[3], z[3]);
	}
}

static void meshAppendShapeTriangles(const TopoDS_Shape& shape, std::vector<float>& soup)
{
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
	{
		const TopoDS_Face face = TopoDS::Face(exp.Current());
		TopLoc_Location loc;
		Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
		if (tri.IsNull() || !tri->HasGeometry() || tri->NbTriangles() <= 0)
		{
			continue;
		}
		for (Standard_Integer ti = 1; ti <= tri->NbTriangles(); ++ti)
		{
			const Poly_Triangle& t = tri->Triangle(ti);
			Standard_Integer n1 = 0, n2 = 0, n3 = 0;
			t.Get(n1, n2, n3);
			gp_Pnt p1 = tri->Node(n1);
			gp_Pnt p2 = tri->Node(n2);
			gp_Pnt p3 = tri->Node(n3);
			p1.Transform(loc.Transformation());
			p2.Transform(loc.Transformation());
			p3.Transform(loc.Transformation());
			soup.push_back(static_cast<float>(p1.X()));
			soup.push_back(static_cast<float>(p1.Y()));
			soup.push_back(static_cast<float>(p1.Z()));
			soup.push_back(static_cast<float>(p2.X()));
			soup.push_back(static_cast<float>(p2.Y()));
			soup.push_back(static_cast<float>(p2.Z()));
			soup.push_back(static_cast<float>(p3.X()));
			soup.push_back(static_cast<float>(p3.Y()));
			soup.push_back(static_cast<float>(p3.Z()));
		}
	}
}

static bool meshHasChildren(const TopoDS_Shape& shape)
{
	TopoDS_Iterator it(shape);
	return it.More();
}

static std::string meshShapeTypeName(const TopAbs_ShapeEnum t)
{
	switch (t)
	{
	case TopAbs_COMPOUND: return "Compound";
	case TopAbs_COMPSOLID: return "CompSolid";
	case TopAbs_SOLID: return "Solid";
	case TopAbs_SHELL: return "Shell";
	case TopAbs_FACE: return "Face";
	case TopAbs_WIRE: return "Wire";
	case TopAbs_EDGE: return "Edge";
	case TopAbs_VERTEX: return "Vertex";
	default: return "Shape";
	}
}

static void meshCollectStepHierarchyRecursive(
	const TopoDS_Shape& shape,
	const std::string& path,
	const std::string& parentPath,
	std::vector<MeshHierarchyPart>& outParts)
{
	const bool hasChildren = meshHasChildren(shape);
	if (!hasChildren)
	{
		MeshHierarchyPart part;
		part.partPath = path;
		part.parentPartPath = parentPath;
		part.displayName = meshShapeTypeName(shape.ShapeType()) + "_" + path;
		meshAppendShapeTriangles(shape, part.triangleSoup);
		if (!part.triangleSoup.empty())
		{
			outParts.push_back(std::move(part));
		}
		return;
	}

	int childIndex = 0;
	for (TopoDS_Iterator it(shape); it.More(); it.Next(), ++childIndex)
	{
		const TopoDS_Shape child = it.Value();
		const std::string childPath = path.empty() ? std::to_string(childIndex) : (path + "/" + std::to_string(childIndex));
		meshCollectStepHierarchyRecursive(child, childPath, path, outParts);
	}
}

class MeshDxfCollector final : public DL_CreationAdapter
{
public:
	std::vector<float> soup;

	void add3dFace(const DL_3dFaceData& data) override { meshAddTraceTriangles(soup, data); }
	void addSolid(const DL_SolidData& data) override { meshAddTraceTriangles(soup, data); }
	void addTrace(const DL_TraceData& data) override { meshAddTraceTriangles(soup, data); }

	void addPolyline(const DL_PolylineData& pd) override
	{
		flushPolyline();
		m_polylineFlags = pd.flags;
		m_polyM = pd.m;
		m_polyN = pd.n;
		m_havePolylineHeader = true;
		m_polyVerts.clear();
	}

	void addVertex(const DL_VertexData& data) override
	{
		if (!m_havePolylineHeader)
		{
			return;
		}
		m_polyVerts.push_back(data.x);
		m_polyVerts.push_back(data.y);
		m_polyVerts.push_back(data.z);
	}

	void endEntity() override { flushPolyline(); }

	void endSequence() override { flushPolyline(); }

private:
	int m_polylineFlags = 0;
	unsigned m_polyM = 0;
	unsigned m_polyN = 0;
	bool m_havePolylineHeader = false;
	std::vector<double> m_polyVerts;

	void flushPolyline()
	{
		if (!m_havePolylineHeader)
		{
			return;
		}
		const std::size_t npts = m_polyVerts.size() / 3;
		if (npts < 3)
		{
			m_polyVerts.clear();
			m_havePolylineHeader = false;
			return;
		}
		const int flags = m_polylineFlags;
		if (flags & 32)
		{
			m_polyVerts.clear();
			m_havePolylineHeader = false;
			return;
		}
		const unsigned m = m_polyM;
		const unsigned n = m_polyN;
		if ((flags & 16) && m >= 2 && n >= 2 && static_cast<std::size_t>(m) * static_cast<std::size_t>(n) == npts)
		{
			for (unsigned i = 0; i + 1 < m; ++i)
			{
				for (unsigned j = 0; j + 1 < n; ++j)
				{
					const std::size_t i00 = static_cast<std::size_t>(i) * n + j;
					const std::size_t i10 = (static_cast<std::size_t>(i) + 1) * n + j;
					const std::size_t i11 = (static_cast<std::size_t>(i) + 1) * n + (j + 1);
					const std::size_t i01 = static_cast<std::size_t>(i) * n + (j + 1);
					const double ax = m_polyVerts[i00 * 3];
					const double ay = m_polyVerts[i00 * 3 + 1];
					const double az = m_polyVerts[i00 * 3 + 2];
					const double bx = m_polyVerts[i10 * 3];
					const double by = m_polyVerts[i10 * 3 + 1];
					const double bz = m_polyVerts[i10 * 3 + 2];
					const double cx = m_polyVerts[i11 * 3];
					const double cy = m_polyVerts[i11 * 3 + 1];
					const double cz = m_polyVerts[i11 * 3 + 2];
					const double dx = m_polyVerts[i01 * 3];
					const double dy = m_polyVerts[i01 * 3 + 1];
					const double dz = m_polyVerts[i01 * 3 + 2];
					meshPushTri(soup, ax, ay, az, bx, by, bz, cx, cy, cz);
					meshPushTri(soup, ax, ay, az, cx, cy, cz, dx, dy, dz);
				}
			}
		}
		else if (flags & 1)
		{
			for (std::size_t k = 1; k + 1 < npts; ++k)
			{
				meshPushTri(soup, m_polyVerts[0], m_polyVerts[1], m_polyVerts[2], m_polyVerts[k * 3],
					m_polyVerts[k * 3 + 1], m_polyVerts[k * 3 + 2], m_polyVerts[(k + 1) * 3], m_polyVerts[(k + 1) * 3 + 1],
					m_polyVerts[(k + 1) * 3 + 2]);
			}
		}
		m_polyVerts.clear();
		m_havePolylineHeader = false;
	}
};

struct DxfMatrix4
{
	double m[16];
};

static DxfMatrix4 dxfIdentityMatrix()
{
	DxfMatrix4 r{};
	for (int i = 0; i < 16; ++i)
	{
		r.m[i] = 0.0;
	}
	r.m[0] = 1.0;
	r.m[5] = 1.0;
	r.m[10] = 1.0;
	r.m[15] = 1.0;
	return r;
}

static DxfMatrix4 dxfMultiply(const DxfMatrix4& a, const DxfMatrix4& b)
{
	DxfMatrix4 r{};
	for (int row = 0; row < 4; ++row)
	{
		for (int col = 0; col < 4; ++col)
		{
			double v = 0.0;
			for (int k = 0; k < 4; ++k)
			{
				v += a.m[row * 4 + k] * b.m[k * 4 + col];
			}
			r.m[row * 4 + col] = v;
		}
	}
	return r;
}

static DxfMatrix4 dxfTranslate(double tx, double ty, double tz)
{
	DxfMatrix4 r = dxfIdentityMatrix();
	r.m[3] = tx;
	r.m[7] = ty;
	r.m[11] = tz;
	return r;
}

static DxfMatrix4 dxfScale(double sx, double sy, double sz)
{
	DxfMatrix4 r = dxfIdentityMatrix();
	r.m[0] = sx;
	r.m[5] = sy;
	r.m[10] = sz;
	return r;
}

static DxfMatrix4 dxfRotateZDeg(double deg)
{
	const double rad = deg * 3.14159265358979323846 / 180.0;
	const double c = std::cos(rad);
	const double s = std::sin(rad);
	DxfMatrix4 r = dxfIdentityMatrix();
	r.m[0] = c;
	r.m[1] = -s;
	r.m[4] = s;
	r.m[5] = c;
	return r;
}

static void dxfApplyPoint(const DxfMatrix4& m, double x, double y, double z, double& ox, double& oy, double& oz)
{
	ox = m.m[0] * x + m.m[1] * y + m.m[2] * z + m.m[3];
	oy = m.m[4] * x + m.m[5] * y + m.m[6] * z + m.m[7];
	oz = m.m[8] * x + m.m[9] * y + m.m[10] * z + m.m[11];
}

struct DxfInsertInstance
{
	std::string blockName;
	double ipx = 0.0;
	double ipy = 0.0;
	double ipz = 0.0;
	double sx = 1.0;
	double sy = 1.0;
	double sz = 1.0;
	double angleDeg = 0.0;
	int cols = 1;
	int rows = 1;
	double colSp = 0.0;
	double rowSp = 0.0;
};

struct DxfBlockDef
{
	double bpx = 0.0;
	double bpy = 0.0;
	double bpz = 0.0;
	std::vector<float> localSoup;
	std::vector<DxfInsertInstance> inserts;
};

class MeshDxfHierarchyCollector final : public DL_CreationAdapter
{
public:
	std::map<std::string, DxfBlockDef> blocks;
	std::vector<float> modelSoup;
	std::vector<DxfInsertInstance> rootInserts;
	std::unordered_set<std::string> hiddenLayers;

	void addLayer(const DL_LayerData& data) override
	{
		(void)data;
	}

	void addBlock(const DL_BlockData& data) override
	{
		flushPolyline();
		DxfBlockDef& b = blocks[data.name];
		b.bpx = data.bpx;
		b.bpy = data.bpy;
		b.bpz = data.bpz;
		m_blockStack.push_back(data.name);
	}

	void endBlock() override
	{
		flushPolyline();
		if (!m_blockStack.empty())
		{
			m_blockStack.pop_back();
		}
	}

	void addInsert(const DL_InsertData& data) override
	{
		flushPolyline();
		if (!entityVisible())
		{
			return;
		}
		DxfInsertInstance ins;
		ins.blockName = data.name;
		ins.ipx = data.ipx;
		ins.ipy = data.ipy;
		ins.ipz = data.ipz;
		ins.sx = data.sx;
		ins.sy = data.sy;
		ins.sz = data.sz;
		ins.angleDeg = data.angle;
		ins.cols = data.cols;
		ins.rows = data.rows;
		ins.colSp = data.colSp;
		ins.rowSp = data.rowSp;
		if (m_blockStack.empty())
		{
			rootInserts.push_back(ins);
		}
		else
		{
			blocks[m_blockStack.back()].inserts.push_back(ins);
		}
	}

	void add3dFace(const DL_3dFaceData& data) override { if (entityVisible()) meshAddTraceTriangles(targetSoup(), data); }
	void addSolid(const DL_SolidData& data) override { if (entityVisible()) meshAddTraceTriangles(targetSoup(), data); }
	void addTrace(const DL_TraceData& data) override { if (entityVisible()) meshAddTraceTriangles(targetSoup(), data); }

	void addPolyline(const DL_PolylineData& pd) override
	{
		flushPolyline();
		if (!entityVisible())
		{
			m_havePolylineHeader = false;
			m_polyVerts.clear();
			return;
		}
		m_polylineFlags = pd.flags;
		m_polyM = pd.m;
		m_polyN = pd.n;
		m_havePolylineHeader = true;
		m_polyVerts.clear();
	}

	void addVertex(const DL_VertexData& data) override
	{
		if (!m_havePolylineHeader)
		{
			return;
		}
		m_polyVerts.push_back(data.x);
		m_polyVerts.push_back(data.y);
		m_polyVerts.push_back(data.z);
	}

	void endEntity() override { flushPolyline(); }
	void endSequence() override { flushPolyline(); }

private:
	std::vector<std::string> m_blockStack;
	int m_polylineFlags = 0;
	unsigned m_polyM = 0;
	unsigned m_polyN = 0;
	bool m_havePolylineHeader = false;
	std::vector<double> m_polyVerts;
	bool entityVisible()
	{
		return true;
	}

	std::vector<float>& targetSoup()
	{
		if (m_blockStack.empty())
		{
			return modelSoup;
		}
		return blocks[m_blockStack.back()].localSoup;
	}

	void flushPolyline()
	{
		if (!m_havePolylineHeader)
		{
			return;
		}
		std::vector<float>& soup = targetSoup();
		const std::size_t npts = m_polyVerts.size() / 3;
		if (npts >= 3)
		{
			const int flags = m_polylineFlags;
			if (!(flags & 32))
			{
				const unsigned m = m_polyM;
				const unsigned n = m_polyN;
				if ((flags & 16) && m >= 2 && n >= 2 && static_cast<std::size_t>(m) * static_cast<std::size_t>(n) == npts)
				{
					for (unsigned i = 0; i + 1 < m; ++i)
					{
						for (unsigned j = 0; j + 1 < n; ++j)
						{
							const std::size_t i00 = static_cast<std::size_t>(i) * n + j;
							const std::size_t i10 = (static_cast<std::size_t>(i) + 1) * n + j;
							const std::size_t i11 = (static_cast<std::size_t>(i) + 1) * n + (j + 1);
							const std::size_t i01 = static_cast<std::size_t>(i) * n + (j + 1);
							meshPushTri(soup, m_polyVerts[i00 * 3], m_polyVerts[i00 * 3 + 1], m_polyVerts[i00 * 3 + 2],
								m_polyVerts[i10 * 3], m_polyVerts[i10 * 3 + 1], m_polyVerts[i10 * 3 + 2], m_polyVerts[i11 * 3],
								m_polyVerts[i11 * 3 + 1], m_polyVerts[i11 * 3 + 2]);
							meshPushTri(soup, m_polyVerts[i00 * 3], m_polyVerts[i00 * 3 + 1], m_polyVerts[i00 * 3 + 2],
								m_polyVerts[i11 * 3], m_polyVerts[i11 * 3 + 1], m_polyVerts[i11 * 3 + 2], m_polyVerts[i01 * 3],
								m_polyVerts[i01 * 3 + 1], m_polyVerts[i01 * 3 + 2]);
						}
					}
				}
				else if (flags & 1)
				{
					for (std::size_t k = 1; k + 1 < npts; ++k)
					{
						meshPushTri(soup, m_polyVerts[0], m_polyVerts[1], m_polyVerts[2], m_polyVerts[k * 3],
							m_polyVerts[k * 3 + 1], m_polyVerts[k * 3 + 2], m_polyVerts[(k + 1) * 3], m_polyVerts[(k + 1) * 3 + 1],
							m_polyVerts[(k + 1) * 3 + 2]);
					}
				}
			}
		}
		m_polyVerts.clear();
		m_havePolylineHeader = false;
	}
};

static DxfMatrix4 dxfComposeInsertTransform(const DxfInsertInstance& ins, const DxfBlockDef& block, double arrayDx, double arrayDy)
{
	DxfMatrix4 m = dxfIdentityMatrix();
	m = dxfMultiply(dxfTranslate(ins.ipx, ins.ipy, ins.ipz), m);
	m = dxfMultiply(dxfRotateZDeg(ins.angleDeg), m);
	m = dxfMultiply(dxfScale(ins.sx == 0.0 ? 1.0 : ins.sx, ins.sy == 0.0 ? 1.0 : ins.sy, ins.sz == 0.0 ? 1.0 : ins.sz), m);
	m = dxfMultiply(dxfTranslate(arrayDx, arrayDy, 0.0), m);
	m = dxfMultiply(dxfTranslate(-block.bpx, -block.bpy, -block.bpz), m);
	return m;
}

static void dxfAppendTransformedSoup(const std::vector<float>& localSoup, const DxfMatrix4& xf, std::vector<float>& outSoup)
{
	outSoup.reserve(outSoup.size() + localSoup.size());
	for (std::size_t i = 0; i + 2 < localSoup.size(); i += 3)
	{
		double ox = 0.0, oy = 0.0, oz = 0.0;
		dxfApplyPoint(xf, localSoup[i], localSoup[i + 1], localSoup[i + 2], ox, oy, oz);
		outSoup.push_back(static_cast<float>(ox));
		outSoup.push_back(static_cast<float>(oy));
		outSoup.push_back(static_cast<float>(oz));
	}
}

static void dxfExpandInsertRecursive(const DxfInsertInstance& ins, const DxfMatrix4& parentXf,
	const std::string& parentPath, int& partCounter, const std::map<std::string, DxfBlockDef>& blocks,
	std::vector<MeshHierarchyPart>& outParts, std::vector<std::string>& stack)
{
	const auto it = blocks.find(ins.blockName);
	if (it == blocks.end())
	{
		return;
	}
	const DxfBlockDef& block = it->second;
	if (std::find(stack.begin(), stack.end(), ins.blockName) != stack.end())
	{
		return;
	}
	stack.push_back(ins.blockName);
	const int cols = std::max(1, ins.cols);
	const int rows = std::max(1, ins.rows);
	for (int r = 0; r < rows; ++r)
	{
		for (int c = 0; c < cols; ++c)
		{
			const double dx = static_cast<double>(c) * ins.colSp;
			const double dy = static_cast<double>(r) * ins.rowSp;
			const DxfMatrix4 local = dxfComposeInsertTransform(ins, block, dx, dy);
			const DxfMatrix4 world = dxfMultiply(parentXf, local);
			const std::string path = std::string("dxf_part_") + std::to_string(++partCounter);
			if (!block.localSoup.empty())
			{
				MeshHierarchyPart part;
				part.partPath = path;
				part.parentPartPath = parentPath;
				part.displayName = ins.blockName;
				dxfAppendTransformedSoup(block.localSoup, world, part.triangleSoup);
				if (!part.triangleSoup.empty())
				{
					outParts.push_back(std::move(part));
				}
			}
			for (const DxfInsertInstance& childIns : block.inserts)
			{
				dxfExpandInsertRecursive(childIns, world, path, partCounter, blocks, outParts, stack);
			}
		}
	}
	stack.pop_back();
}

} // namespace

bool MeshBackendData::loadStepHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts, std::string* errMsg)
{
	outParts.clear();
	STEPControl_Reader reader;
	const IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
	if (status != IFSelect_RetDone)
	{
		meshLoadErr(errMsg, "OCCT STEP read failed.");
		return false;
	}
	const bool ok = reader.TransferRoots();
	if (!ok)
	{
		meshLoadErr(errMsg, "OCCT STEP transfer failed.");
		return false;
	}
	const TopoDS_Shape shape = reader.OneShape();
	if (shape.IsNull())
	{
		meshLoadErr(errMsg, "OCCT STEP produced an empty shape.");
		return false;
	}

	const Standard_Real linDeflectionRel = 0.01;
	const Standard_Boolean isRelative = Standard_True;
	const Standard_Real angDeflection = 0.5;
	BRepMesh_IncrementalMesh mesher(shape, linDeflectionRel, isRelative, angDeflection, Standard_False);
	(void)mesher;

	meshCollectStepHierarchyRecursive(shape, "0", std::string(), outParts);
	if (outParts.empty())
	{
		meshLoadErr(errMsg, "OCCT STEP hierarchy triangulation produced no mesh parts.");
		return false;
	}
	return true;
}

bool MeshBackendData::loadDxfHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts, std::string* errMsg)
{
	outParts.clear();
	DL_Dxf dxf;
	MeshDxfHierarchyCollector collector;
	if (!dxf.in(path, &collector))
	{
		meshLoadErr(errMsg, "Could not open DXF file.");
		return false;
	}
	int counter = 0;
	std::vector<std::string> stack;
	const DxfMatrix4 identity = dxfIdentityMatrix();

	auto expandInsertListAsRoot = [&](const std::vector<DxfInsertInstance>& insList) {
		for (const DxfInsertInstance& ins : insList)
		{
			dxfExpandInsertRecursive(ins, identity, std::string(), counter, collector.blocks, outParts, stack);
		}
	};

	// Priority 1: explicit root inserts parsed outside any BLOCK.
	expandInsertListAsRoot(collector.rootInserts);

	// Priority 2: many CAD exports put model-space content in *Model_Space BLOCK.
	if (outParts.empty())
	{
		const auto modelIt = collector.blocks.find("*Model_Space");
		if (modelIt != collector.blocks.end())
		{
			const DxfBlockDef& ms = modelIt->second;
			if (!ms.localSoup.empty())
			{
				MeshHierarchyPart root;
				root.partPath = "dxf_modelspace_root";
				root.parentPartPath.clear();
				root.displayName = "Model_Space";
				root.triangleSoup = ms.localSoup;
				outParts.push_back(std::move(root));
			}
			expandInsertListAsRoot(ms.inserts);
		}
	}

	// Priority 3: fallback to paper-space blocks if model space not present.
	if (outParts.empty())
	{
		for (const char* paperName : { "*Paper_Space", "*Paper_Space0" })
		{
			const auto it = collector.blocks.find(paperName);
			if (it == collector.blocks.end())
			{
				continue;
			}
			const DxfBlockDef& ps = it->second;
			if (!ps.localSoup.empty())
			{
				MeshHierarchyPart root;
				root.partPath = std::string("dxf_") + paperName + "_root";
				root.parentPartPath.clear();
				root.displayName = paperName;
				root.triangleSoup = ps.localSoup;
				outParts.push_back(std::move(root));
			}
			expandInsertListAsRoot(ps.inserts);
		}
	}

	// Priority 4: last fallback, if there are still no roots but blocks exist, expand all block inserts.
	if (outParts.empty())
	{
		for (const auto& kv : collector.blocks)
		{
			expandInsertListAsRoot(kv.second.inserts);
		}
	}

	// Keep model-space direct geometry if there was no insert-expanded result.
	if (outParts.empty() && !collector.modelSoup.empty())
	{
		MeshHierarchyPart root;
		root.partPath = "dxf_part_root";
		root.parentPartPath.clear();
		root.displayName = "DXF_Model";
		root.triangleSoup = collector.modelSoup;
		outParts.push_back(std::move(root));
	}
	if (outParts.empty())
	{
		meshLoadErr(errMsg, "DXF contained no triangulatable geometry.");
		return false;
	}
	return true;
}

bool MeshBackendData::loadFromFile(const std::string& path, std::string* errMsg)
{
	clearGeometry();
	const std::string ext = meshLowerExtension(path);
	if (ext.empty())
	{
		meshLoadErr(errMsg, "Missing file extension.");
		return false;
	}

	// STEP 入口：使用 OCCT 读取并离散化成三角形 soup。
	if (ext == "step" || ext == "stp")
	{
		// 1) 读取 STEP
		STEPControl_Reader reader;
		const IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
		if (status != IFSelect_RetDone)
		{
			meshLoadErr(errMsg, "OCCT STEP read failed.");
			return false;
		}
		const bool ok = reader.TransferRoots();
		if (!ok)
		{
			meshLoadErr(errMsg, "OCCT STEP transfer failed.");
			return false;
		}
		const TopoDS_Shape shape = reader.OneShape();
		if (shape.IsNull())
		{
			meshLoadErr(errMsg, "OCCT STEP produced an empty shape.");
			return false;
		}

		// 2) 计算包围盒，按相对边长选择网格离散精度（对尺寸更鲁棒）
		Bnd_Box box;
		BRepBndLib::Add(shape, box);
		(void)box; // box 用于调参时可以再扩展

		// 线偏差：相对（每条边长度 * 该比例），避免绝对尺度导致三角形过大/过小
		const Standard_Real linDeflectionRel = 0.01; // 1% 边长
		const Standard_Boolean isRelative = Standard_True;
		const Standard_Real angDeflection = 0.5; // OCCT 默认口径

		BRepMesh_IncrementalMesh mesher(shape, linDeflectionRel, isRelative, angDeflection, Standard_False);

		// 3) 遍历面，提取三角化数据并写入 soup（每三角形 9 个 float：三顶点 xyz）
		std::vector<float> soup;
		for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
		{
			const TopoDS_Face face = TopoDS::Face(exp.Current());
			TopLoc_Location loc;
			Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
			if (tri.IsNull() || !tri->HasGeometry() || tri->NbTriangles() <= 0)
			{
				continue;
			}

			// 节点与三角形索引在 OCCT 中通常是 1-based
			for (Standard_Integer ti = 1; ti <= tri->NbTriangles(); ++ti)
			{
				const Poly_Triangle& t = tri->Triangle(ti);
				Standard_Integer n1 = 0, n2 = 0, n3 = 0;
				t.Get(n1, n2, n3);

				gp_Pnt p1 = tri->Node(n1);
				gp_Pnt p2 = tri->Node(n2);
				gp_Pnt p3 = tri->Node(n3);
				p1.Transform(loc.Transformation());
				p2.Transform(loc.Transformation());
				p3.Transform(loc.Transformation());

				soup.push_back(static_cast<float>(p1.X()));
				soup.push_back(static_cast<float>(p1.Y()));
				soup.push_back(static_cast<float>(p1.Z()));
				soup.push_back(static_cast<float>(p2.X()));
				soup.push_back(static_cast<float>(p2.Y()));
				soup.push_back(static_cast<float>(p2.Z()));
				soup.push_back(static_cast<float>(p3.X()));
				soup.push_back(static_cast<float>(p3.Y()));
				soup.push_back(static_cast<float>(p3.Z()));
			}
		}

		if (soup.empty())
		{
			meshLoadErr(errMsg, "OCCT STEP triangulation produced an empty triangle soup.");
			return false;
		}

		setTriangleSoup(std::move(soup));
		return !m_triangleSoup.empty();
	}

	if (ext == "dxf")
	{
		DL_Dxf dxf;
		MeshDxfCollector collector;
		if (!dxf.in(path, &collector))
		{
			meshLoadErr(errMsg, "Could not open DXF file.");
			return false;
		}
		if (collector.soup.empty())
		{
			meshLoadErr(errMsg,
				"DXF contained no triangulatable geometry (3DFACE/TRACE/SOLID or closed polyline / polygon mesh).");
			return false;
		}
		setTriangleSoup(std::move(collector.soup));
		return true;
	}

	// CGAL 入口（保持原逻辑）
	if (!(ext == "obj" || ext == "stl" || ext == "ply" || ext == "off"))
	{
		meshLoadErr(errMsg, "CGAL backend supports .obj .stl .ply .off; other formats use the OSG import path.");
		return false;
	}

	using K = CGAL::Simple_cartesian<double>;
	using Point3 = K::Point_3;
	std::vector<Point3> points;
	std::vector<std::vector<std::size_t>> polygons;
	if (!CGAL::IO::read_polygon_soup(path, points, polygons))
	{
		meshLoadErr(errMsg, "CGAL could not read mesh (polygon soup).");
		return false;
	}
	if (points.empty())
	{
		meshLoadErr(errMsg, "Mesh file contains no vertices.");
		return false;
	}

	std::vector<float> soup;
	for (const auto& poly : polygons)
	{
		if (poly.size() < 3U)
		{
			continue;
		}
		for (std::size_t k = 1; k + 1 < poly.size(); ++k)
		{
			const std::size_t i0 = poly[0];
			const std::size_t i1 = poly[k];
			const std::size_t i2 = poly[k + 1];
			if (i0 >= points.size() || i1 >= points.size() || i2 >= points.size())
			{
				continue;
			}
			const Point3& p0 = points[i0];
			const Point3& p1 = points[i1];
			const Point3& p2 = points[i2];
			soup.push_back(static_cast<float>(p0.x()));
			soup.push_back(static_cast<float>(p0.y()));
			soup.push_back(static_cast<float>(p0.z()));
			soup.push_back(static_cast<float>(p1.x()));
			soup.push_back(static_cast<float>(p1.y()));
			soup.push_back(static_cast<float>(p1.z()));
			soup.push_back(static_cast<float>(p2.x()));
			soup.push_back(static_cast<float>(p2.y()));
			soup.push_back(static_cast<float>(p2.z()));
		}
	}
	if (soup.empty())
	{
		meshLoadErr(errMsg, "No triangles extracted from mesh file.");
		return false;
	}
	setTriangleSoup(std::move(soup));
	return true;
}
