/// @file MeshBackendData_dxf.cpp
/// @brief Mesh 后端数据

#include "pch.h"

#include "MeshBackendData.h"
#include "MeshBackendData_loaders.h"
#include "RunLogger.h"
#include "dl_creationadapter.h"
#include "dl_dxf.h"

namespace mesh_backend_load
{
static void meshAddTraceTriangles(std::vector<float>& soup, const DL_TraceData& d)
{
	const double* x = d.x;
	const double* y = d.y;
	const double* z = d.z;
	auto same = [&](int a, int b)
	{
		const double dx = x[a] - x[b];
		const double dy = y[a] - y[b];
		const double dz = z[a] - z[b];
		return dx * dx + dy * dy + dz * dz < 1e-24;
	};
	if (same(2, 3)) // 退化四边形
	{
		mesh_backend_load::meshPushTri(soup, x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
	}
	else
	{
		mesh_backend_load::meshPushTri(soup, x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]);
		mesh_backend_load::meshPushTri(soup, x[0], y[0], z[0], x[2], y[2], z[2], x[3], y[3], z[3]);
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
					mesh_backend_load::meshPushTri(soup, ax, ay, az, bx, by, bz, cx, cy, cz);
					mesh_backend_load::meshPushTri(soup, ax, ay, az, cx, cy, cz, dx, dy, dz);
				}
			}
		}
		else if (flags & 1)
		{
			for (std::size_t k = 1; k + 1 < npts; ++k)
			{
				mesh_backend_load::meshPushTri(soup, m_polyVerts[0], m_polyVerts[1], m_polyVerts[2], m_polyVerts[k * 3],
											   m_polyVerts[k * 3 + 1], m_polyVerts[k * 3 + 2], m_polyVerts[(k + 1) * 3],
											   m_polyVerts[(k + 1) * 3 + 1], m_polyVerts[(k + 1) * 3 + 2]);
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

	void addLayer(const DL_LayerData& data) override { (void)data; }

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

	void add3dFace(const DL_3dFaceData& data) override
	{
		if (entityVisible())
			meshAddTraceTriangles(targetSoup(), data);
	}
	void addSolid(const DL_SolidData& data) override
	{
		if (entityVisible())
			meshAddTraceTriangles(targetSoup(), data);
	}
	void addTrace(const DL_TraceData& data) override
	{
		if (entityVisible())
			meshAddTraceTriangles(targetSoup(), data);
	}

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
	bool entityVisible() { return true; }

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
				if ((flags & 16) && m >= 2 && n >= 2 &&
					static_cast<std::size_t>(m) * static_cast<std::size_t>(n) == npts)
				{
					for (unsigned i = 0; i + 1 < m; ++i)
					{
						for (unsigned j = 0; j + 1 < n; ++j)
						{
							const std::size_t i00 = static_cast<std::size_t>(i) * n + j;
							const std::size_t i10 = (static_cast<std::size_t>(i) + 1) * n + j;
							const std::size_t i11 = (static_cast<std::size_t>(i) + 1) * n + (j + 1);
							const std::size_t i01 = static_cast<std::size_t>(i) * n + (j + 1);
							mesh_backend_load::meshPushTri(
								soup, m_polyVerts[i00 * 3], m_polyVerts[i00 * 3 + 1], m_polyVerts[i00 * 3 + 2],
								m_polyVerts[i10 * 3], m_polyVerts[i10 * 3 + 1], m_polyVerts[i10 * 3 + 2],
								m_polyVerts[i11 * 3], m_polyVerts[i11 * 3 + 1], m_polyVerts[i11 * 3 + 2]);
							mesh_backend_load::meshPushTri(
								soup, m_polyVerts[i00 * 3], m_polyVerts[i00 * 3 + 1], m_polyVerts[i00 * 3 + 2],
								m_polyVerts[i11 * 3], m_polyVerts[i11 * 3 + 1], m_polyVerts[i11 * 3 + 2],
								m_polyVerts[i01 * 3], m_polyVerts[i01 * 3 + 1], m_polyVerts[i01 * 3 + 2]);
						}
					}
				}
				else if (flags & 1)
				{
					for (std::size_t k = 1; k + 1 < npts; ++k)
					{
						mesh_backend_load::meshPushTri(soup, m_polyVerts[0], m_polyVerts[1], m_polyVerts[2],
													   m_polyVerts[k * 3], m_polyVerts[k * 3 + 1],
													   m_polyVerts[k * 3 + 2], m_polyVerts[(k + 1) * 3],
													   m_polyVerts[(k + 1) * 3 + 1], m_polyVerts[(k + 1) * 3 + 2]);
					}
				}
			}
		}
		m_polyVerts.clear();
		m_havePolylineHeader = false;
	}
};

static DxfMatrix4 dxfComposeInsertTransform(const DxfInsertInstance& ins, const DxfBlockDef& block, double arrayDx,
											double arrayDy)
{
	DxfMatrix4 m = dxfIdentityMatrix();
	m = dxfMultiply(dxfTranslate(ins.ipx, ins.ipy, ins.ipz), m);
	m = dxfMultiply(dxfRotateZDeg(ins.angleDeg), m);
	m = dxfMultiply(dxfScale(ins.sx == 0.0 ? 1.0 : ins.sx, ins.sy == 0.0 ? 1.0 : ins.sy, ins.sz == 0.0 ? 1.0 : ins.sz),
					m);
	m = dxfMultiply(dxfTranslate(arrayDx, arrayDy, 0.0), m);
	m = dxfMultiply(dxfTranslate(-block.bpx, -block.bpy, -block.bpz), m);
	return m;
}

static void dxfAppendTransformedSoup(const std::vector<float>& localSoup, const DxfMatrix4& xf,
									 std::vector<float>& outSoup)
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
									 const std::string& parentPath, int& partCounter,
									 const std::map<std::string, DxfBlockDef>& blocks,
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

bool meshLoadDxfSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg)
{
	soup.clear();
	DL_Dxf dxf;
	MeshDxfCollector collector;
	if (!dxf.in(path, &collector))
	{
		mesh_backend_load::meshLoadErr(errMsg, "Could not open DXF file.");
		return false;
	}
	if (collector.soup.empty())
	{
		mesh_backend_load::meshLoadErr(
			errMsg, "DXF contained no triangulatable geometry (3DFACE/TRACE/SOLID or closed polyline / polygon mesh).");
		return false;
	}
	soup = std::move(collector.soup);
	return true;
}

} // namespace mesh_backend_load

bool MeshBackendData::loadDxfHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts,
											   std::string* errMsg)
{
	outParts.clear();
	DL_Dxf dxf;
	mesh_backend_load::MeshDxfHierarchyCollector collector;
	if (!dxf.in(path, &collector))
	{
		mesh_backend_load::meshLoadErr(errMsg, "Could not open DXF file.");
		return false;
	}
	int counter = 0;
	std::vector<std::string> stack;
	const mesh_backend_load::DxfMatrix4 identity = mesh_backend_load::dxfIdentityMatrix();

	auto expandInsertListAsRoot = [&](const std::vector<mesh_backend_load::DxfInsertInstance>& insList)
	{
		for (const mesh_backend_load::DxfInsertInstance& ins : insList)
		{
			mesh_backend_load::dxfExpandInsertRecursive(ins, identity, std::string(), counter, collector.blocks,
														outParts, stack);
		}
	};

	// 优先：BLOCK 外显式根 INSERT
	expandInsertListAsRoot(collector.rootInserts);

	// 次选：*Model_Space BLOCK
	if (outParts.empty())
	{
		const auto modelIt = collector.blocks.find("*Model_Space");
		if (modelIt != collector.blocks.end())
		{
			const mesh_backend_load::DxfBlockDef& ms = modelIt->second;
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

	// 再次：纸空间 BLOCK
	if (outParts.empty())
	{
		for (const char* paperName : {"*Paper_Space", "*Paper_Space0"})
		{
			const auto it = collector.blocks.find(paperName);
			if (it == collector.blocks.end())
			{
				continue;
			}
			const mesh_backend_load::DxfBlockDef& ps = it->second;
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

	// 兜底：无根时展开全部 BLOCK INSERT
	if (outParts.empty())
	{
		for (const auto& kv : collector.blocks)
		{
			expandInsertListAsRoot(kv.second.inserts);
		}
	}

	// 无 INSERT 结果时保留模型空间几何
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
		mesh_backend_load::meshLoadErr(errMsg, "DXF contained no triangulatable geometry.");
		return false;
	}
	RunLogger::info("[MeshBackendData] DXF hierarchy loaded successfully.");
	return true;
}
