#include "PointCloudBackendData.h"

#include "PlyIo.h"
#include "geometry_base64.h"
#include "../../PropertyCore/inc/PropertyAttribute.h"

#include <CGAL/IO/io.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/Simple_cartesian.h>
#include <boost/cstdint.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <cctype>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <tuple>

PointCloudBackendData::PointCloudBackendData()
{
	setName("PointCloud");
	BackendColor c;
	c.r = 0.65f;
	c.g = 0.82f;
	c.b = 0.95f;
	c.a = 1.0f;
	m_color = c;
	m_attributes.push_back(makeBackendPoseAttribute());
	m_attributes.push_back(makeBackendRotationAttribute());
	m_attributes.push_back(makeBackendDisplayColorAttribute());
}

std::string PointCloudBackendData::className() const
{
	return "PointCloudBackendData";
}

bool PointCloudBackendData::hasGeometry() const
{
	return !m_xyz.empty() || m_pointCount > 0U;
}

BackendBoundingBox PointCloudBackendData::geometryBounds() const
{
	return m_bounds;
}

std::size_t PointCloudBackendData::geometryElementCount() const
{
	if (!m_xyz.empty())
	{
		return m_xyz.size() / 3U;
	}
	return m_pointCount;
}

void PointCloudBackendData::clearGeometry()
{
	m_pointCount = 0U;
	m_bounds = BackendBoundingBox{};
	m_xyz.clear();
	m_rgbaVertex.clear();
	m_normals.clear();
}

void PointCloudBackendData::setColor(const BackendColor& color)
{
	m_color = color;
}

BackendColor PointCloudBackendData::color() const
{
	return m_color;
}

void PointCloudBackendData::setPointCount(std::size_t count)
{
	m_pointCount = count;
}

void PointCloudBackendData::setBounds(const BackendBoundingBox& bounds)
{
	m_bounds = bounds;
}

void PointCloudBackendData::setPointBuffers(std::vector<float> xyz, std::vector<float> rgbaPerVertex)
{
	setPointBuffers(std::move(xyz), std::move(rgbaPerVertex), {});
}

void PointCloudBackendData::setPointBuffers(
	std::vector<float> xyz,
	std::vector<float> rgbaPerVertex,
	std::vector<float> normalsNxNyNz)
{
	if (xyz.size() % 3U != 0U)
	{
		clearGeometry();
		return;
	}
	const std::size_t n = xyz.size() / 3U;
	if (!rgbaPerVertex.empty() && rgbaPerVertex.size() != n * 4U)
	{
		rgbaPerVertex.clear();
	}
	if (!normalsNxNyNz.empty() && normalsNxNyNz.size() != n * 3U)
	{
		normalsNxNyNz.clear();
	}
	m_xyz = std::move(xyz);
	m_rgbaVertex = std::move(rgbaPerVertex);
	m_normals = std::move(normalsNxNyNz);
	m_pointCount = m_xyz.empty() ? 0U : n;
	recomputeBoundsFromPoints();
}

void PointCloudBackendData::setPointNormals(std::vector<float> normalsNxNyNz)
{
	const std::size_t n = m_xyz.size() / 3U;
	if (n == 0U || normalsNxNyNz.size() != n * 3U)
	{
		m_normals.clear();
		return;
	}
	m_normals = std::move(normalsNxNyNz);
}

void PointCloudBackendData::recomputeBoundsFromPoints()
{
	if (m_xyz.size() < 3U)
	{
		m_bounds = BackendBoundingBox{};
		return;
	}
	double minX = m_xyz[0];
	double minY = m_xyz[1];
	double minZ = m_xyz[2];
	double maxX = minX;
	double maxY = minY;
	double maxZ = minZ;
	for (std::size_t i = 0; i + 2 < m_xyz.size(); i += 3)
	{
		const double x = m_xyz[i];
		const double y = m_xyz[i + 1];
		const double z = m_xyz[i + 2];
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

bool PointCloudBackendData::writeProjectEmbeddedGeometry(std::string& outXyzBase64, std::string& outRgbaPerVertexBase64) const
{
	outXyzBase64.clear();
	outRgbaPerVertexBase64.clear();
	if (m_xyz.empty() || (m_xyz.size() % 3U) != 0U)
	{
		return false;
	}
	outXyzBase64 = geometryBase64EncodeFloats(m_xyz);
	if (outXyzBase64.empty())
	{
		return false;
	}
	if (!m_rgbaVertex.empty() && m_rgbaVertex.size() == (m_xyz.size() / 3U) * 4U)
	{
		outRgbaPerVertexBase64 = geometryBase64EncodeFloats(m_rgbaVertex);
	}
	return true;
}

bool PointCloudBackendData::readProjectEmbeddedGeometry(const std::string& xyzBase64, const std::string& rgbaPerVertexBase64)
{
	std::vector<float> xyz;
	if (!geometryBase64DecodeFloats(xyzBase64, xyz) || xyz.size() < 3U || (xyz.size() % 3U) != 0U)
	{
		return false;
	}
	std::vector<float> rgba;
	if (!rgbaPerVertexBase64.empty())
	{
		if (!geometryBase64DecodeFloats(rgbaPerVertexBase64, rgba))
		{
			return false;
		}
		if (rgba.size() != (xyz.size() / 3U) * 4U)
		{
			rgba.clear();
		}
	}
	setPointBuffers(std::move(xyz), std::move(rgba));
	return !m_xyz.empty();
}

namespace {

// PLY 写 float xyz 减体积（~12B/点 vs double ~24B）
using PlyWriteKernel = CGAL::Simple_cartesian<float>;
using PlyWritePoint_3 = PlyWriteKernel::Point_3;
using VtxRgbWrite = std::tuple<PlyWritePoint_3, boost::uint8_t, boost::uint8_t, boost::uint8_t>;

// 读 double 兼容已有 float/double 顶点
using PlyReadKernel = CGAL::Simple_cartesian<double>;
using PlyReadPoint_3 = PlyReadKernel::Point_3;
using VtxRgbRead = std::tuple<PlyReadPoint_3, boost::uint8_t, boost::uint8_t, boost::uint8_t>;

static void setErr(std::string* errMsg, const char* text)
{
	if (errMsg)
	{
		*errMsg = text;
	}
}

static boost::uint8_t floatChannelToU8(float c)
{
	const long v = std::lround(c * 255.0f);
	return static_cast<boost::uint8_t>(std::clamp(v, 0L, 255L));
}

static void cgalRgbRowsToBackend(const std::vector<VtxRgbRead>& withRgb, PointCloudBackendData& dst)
{
	std::vector<float> xyz(withRgb.size() * 3U);
	std::vector<float> rgba(withRgb.size() * 4U);
	for (std::size_t i = 0; i < withRgb.size(); ++i)
	{
		const PlyReadPoint_3& p = std::get<0>(withRgb[i]);
		xyz[i * 3U] = static_cast<float>(p.x());
		xyz[i * 3U + 1U] = static_cast<float>(p.y());
		xyz[i * 3U + 2U] = static_cast<float>(p.z());
		const float rs = static_cast<float>(std::get<1>(withRgb[i])) / 255.0f;
		const float gs = static_cast<float>(std::get<2>(withRgb[i])) / 255.0f;
		const float bs = static_cast<float>(std::get<3>(withRgb[i])) / 255.0f;
		rgba[i * 4U] = rs;
		rgba[i * 4U + 1U] = gs;
		rgba[i * 4U + 2U] = bs;
		rgba[i * 4U + 3U] = 1.0f;
	}
	dst.setPointBuffers(std::move(xyz), std::move(rgba));
}

static bool readPlyWithCgalFromPath(const std::string& utf8Path, const PlyHeaderInfo& scan, PointCloudBackendData& dst,
	std::string* errMsg)
{
	if (!scan.cgalFormatOnLine2)
	{
		return false;
	}

	std::ifstream is(std::filesystem::path(utf8Path), scan.isAscii ? std::ios::in : std::ios::binary);
	if (!is)
	{
		setErr(errMsg, "Cannot open point PLY file.");
		return false;
	}
	CGAL::IO::set_mode(is, scan.isAscii ? CGAL::IO::ASCII : CGAL::IO::BINARY);

	if (scan.hasUcharRgb)
	{
		std::vector<VtxRgbRead> withRgb;
		VtxRgbRead keyRgb{};
		const bool rgbOk = CGAL::IO::read_PLY_with_properties(is, std::back_inserter(withRgb),
			CGAL::IO::make_ply_point_reader(CGAL::make_nth_of_tuple_property_map<0>(keyRgb)),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(keyRgb), CGAL::IO::PLY_property<boost::uint8_t>("red")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(keyRgb), CGAL::IO::PLY_property<boost::uint8_t>("green")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(keyRgb), CGAL::IO::PLY_property<boost::uint8_t>("blue")));
		if (rgbOk && !withRgb.empty())
		{
			cgalRgbRowsToBackend(withRgb, dst);
			return true;
		}
	}

	std::ifstream is2(std::filesystem::path(utf8Path), scan.isAscii ? std::ios::in : std::ios::binary);
	if (!is2)
	{
		setErr(errMsg, "Cannot open point PLY file.");
		return false;
	}
	CGAL::IO::set_mode(is2, scan.isAscii ? CGAL::IO::ASCII : CGAL::IO::BINARY);
	std::vector<PlyReadPoint_3> pts;
	const bool ptsOk = CGAL::IO::read_PLY<PlyReadPoint_3>(is2, std::back_inserter(pts));
	if (!ptsOk || pts.empty())
	{
		setErr(errMsg, "CGAL could not read PLY vertex positions.");
		return false;
	}
	std::vector<float> xyz(pts.size() * 3U);
	for (std::size_t i = 0; i < pts.size(); ++i)
	{
		xyz[i * 3U] = static_cast<float>(pts[i].x());
		xyz[i * 3U + 1U] = static_cast<float>(pts[i].y());
		xyz[i * 3U + 2U] = static_cast<float>(pts[i].z());
	}
	dst.setPointBuffers(std::move(xyz), {});
	return true;
}

static int splitAsciiNumbers(const std::string& line, double* out, int maxOut)
{
	std::istringstream ls(line);
	int n = 0;
	while (n < maxOut && ls >> out[n])
	{
		++n;
	}
	return n;
}

static bool readAsciiPlyFlexible(const std::string& utf8Path, const PlyHeaderInfo& scan, PointCloudBackendData& dst,
	std::string* errMsg)
{
	if (!scan.isAscii || scan.vertexCount <= 0 || scan.vertexHasListProperty)
	{
		setErr(errMsg, "ASCII PLY fallback needs vertex count and no list properties on vertex.");
		return false;
	}
	if (scan.ix < 0 || scan.iy < 0 || scan.iz < 0)
	{
		setErr(errMsg, "ASCII PLY fallback: missing x/y/z in header.");
		return false;
	}

	// 花括号避免 most vexing parse：fin(path) 会被解析成函数声明
	std::ifstream fin{std::filesystem::path{utf8Path}};
	if (!fin)
	{
		setErr(errMsg, "Cannot open point PLY file.");
		return false;
	}
	std::string line;
	while (std::getline(fin, line))
	{
		if (!line.empty() && line.back() == '\r')
		{
			line.pop_back();
		}
		if (line == "end_header")
		{
			break;
		}
	}

	const bool wantRgb = (scan.ir >= 0 && scan.ig >= 0 && scan.ib >= 0);
	std::vector<float> xyz;
	std::vector<float> rgba;
	xyz.reserve(static_cast<std::size_t>(scan.vertexCount) * 3U);
	if (wantRgb)
	{
		rgba.reserve(static_cast<std::size_t>(scan.vertexCount) * 4U);
	}

	double vals[64];
	for (int vi = 0; vi < scan.vertexCount; ++vi)
	{
		if (!std::getline(fin, line))
		{
			setErr(errMsg, "ASCII PLY: unexpected end of file in vertex section.");
			return false;
		}
		if (!line.empty() && line.back() == '\r')
		{
			line.pop_back();
		}
		const int nc = splitAsciiNumbers(line, vals, 64);
		if (nc <= std::max({scan.ix, scan.iy, scan.iz}))
		{
			setErr(errMsg, "ASCII PLY: bad vertex line.");
			return false;
		}
		xyz.push_back(static_cast<float>(vals[scan.ix]));
		xyz.push_back(static_cast<float>(vals[scan.iy]));
		xyz.push_back(static_cast<float>(vals[scan.iz]));
		if (wantRgb)
		{
			if (nc <= std::max({scan.ir, scan.ig, scan.ib}))
			{
				setErr(errMsg, "ASCII PLY: missing color components on vertex line.");
				return false;
			}
			float r = static_cast<float>(vals[scan.ir]);
			float g = static_cast<float>(vals[scan.ig]);
			float b = static_cast<float>(vals[scan.ib]);
			if (r > 1.0f || g > 1.0f || b > 1.0f)
			{
				r /= 255.0f;
				g /= 255.0f;
				b /= 255.0f;
			}
			rgba.push_back(r);
			rgba.push_back(g);
			rgba.push_back(b);
			rgba.push_back(1.0f);
		}
	}

	if (xyz.empty())
	{
		setErr(errMsg, "ASCII PLY: no vertices read.");
		return false;
	}
	if (wantRgb && rgba.size() == xyz.size() / 3U * 4U)
	{
		dst.setPointBuffers(std::move(xyz), std::move(rgba));
	}
	else
	{
		dst.setPointBuffers(std::move(xyz), {});
	}
	return true;
}

} // namespace

bool PointCloudBackendData::readPointCloudFromPlyFile(const std::string& utf8Path, std::string* errMsg)
{
	PlyHeaderInfo scan;
	if (!scanPlyHeader(utf8Path, scan, errMsg))
	{
		return false;
	}

	std::string cgalErr;
	if (scan.cgalFormatOnLine2
		&& readPlyWithCgalFromPath(utf8Path, scan, *this, &cgalErr) && !pointPositionsXyz().empty())
	{
		return true;
	}

	std::string asciiErr;
	if (scan.isAscii && scan.vertexCount > 0 && !scan.vertexHasListProperty
		&& readAsciiPlyFlexible(utf8Path, scan, *this, &asciiErr) && !pointPositionsXyz().empty())
	{
		return true;
	}

	if (errMsg)
	{
		errMsg->clear();
		if (!cgalErr.empty() && scan.cgalFormatOnLine2)
		{
			*errMsg = cgalErr;
		}
		if (!asciiErr.empty())
		{
			if (!errMsg->empty())
			{
				errMsg->append(" | ");
			}
			errMsg->append(asciiErr);
		}
		if (errMsg->empty())
		{
			setErr(errMsg, "Could not read PLY point cloud (CGAL + ASCII fallback).");
		}
	}
	return false;
}

namespace {

bool readPointCloudFromXyzFilePath(PointCloudBackendData& pc, const std::string& nativePath, std::string* errMsg)
{
	std::ifstream in(nativePath);
	if (!in)
	{
		setErr(errMsg, "Cannot open XYZ file.");
		return false;
	}
	std::vector<float> xyz;
	std::string line;
	while (std::getline(in, line))
	{
		std::size_t i = 0;
		while (i < line.size() && (line[i] == ' ' || line[i] == '\t' || line[i] == '\r'))
		{
			++i;
		}
		if (i >= line.size() || line[i] == '#')
		{
			continue;
		}
		std::istringstream ls(line.substr(i));
		double vx = 0.0;
		double vy = 0.0;
		double vz = 0.0;
		if (ls >> vx >> vy >> vz)
		{
			xyz.push_back(static_cast<float>(vx));
			xyz.push_back(static_cast<float>(vy));
			xyz.push_back(static_cast<float>(vz));
		}
	}
	if (xyz.empty())
	{
		setErr(errMsg, "No valid XYZ points.");
		return false;
	}
	pc.setPointBuffers(std::move(xyz), {});
	return true;
}

} // namespace

bool PointCloudBackendData::loadFromFile(const std::string& path, std::string* errMsg)
{
	clearGeometry();
	const auto dot = path.find_last_of('.');
	if (dot == std::string::npos)
	{
		setErr(errMsg, "Missing file extension.");
		return false;
	}
	std::string ext = path.substr(dot + 1);
	for (char& c : ext)
	{
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	if (ext == "ply")
	{
		if (plyFileHasTriangleFaces(path))
		{
			setErr(errMsg, "PLY contains faces; import as mesh instead of point cloud.");
			return false;
		}
		return readPointCloudFromPlyFile(path, errMsg);
	}
	if (ext == "xyz")
	{
		return readPointCloudFromXyzFilePath(*this, path, errMsg);
	}
	if (ext == "las" || ext == "laz")
	{
		setErr(errMsg, "LAS/LAZ: use OSG import path or convert to PLY/XYZ.");
		return false;
	}
	setErr(errMsg, "Unsupported point cloud extension for CGAL backend load.");
	return false;
}

bool PointCloudBackendData::writePointCloudPlySidecar(const std::string& utf8Path, std::string* errMsg) const
{
	if (m_xyz.empty() || (m_xyz.size() % 3U) != 0U)
	{
		setErr(errMsg, "No point coordinates to write.");
		return false;
	}
	const std::size_t n = m_xyz.size() / 3U;
	const bool hasRgba = hasPerVertexColors() && m_rgbaVertex.size() == n * 4U;

	std::ofstream ofs(std::filesystem::path(utf8Path), std::ios::binary);
	if (!ofs)
	{
		setErr(errMsg, "Cannot open file for writing.");
		return false;
	}
	// PLY 二进制：end_header 后起 little-endian 顶点
	CGAL::IO::set_mode(ofs, CGAL::IO::BINARY);

	bool ok = false;
	if (hasRgba)
	{
		std::vector<VtxRgbWrite> verts;
		verts.reserve(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			const float x = m_xyz[i * 3U];
			const float y = m_xyz[i * 3U + 1U];
			const float z = m_xyz[i * 3U + 2U];
			const float rf = m_rgbaVertex[i * 4U];
			const float gf = m_rgbaVertex[i * 4U + 1U];
			const float bf = m_rgbaVertex[i * 4U + 2U];
			verts.emplace_back(PlyWritePoint_3(x, y, z), floatChannelToU8(rf), floatChannelToU8(gf), floatChannelToU8(bf));
		}
		VtxRgbWrite keyTpl{};
		ok = CGAL::IO::write_PLY_with_properties(ofs, verts,
			CGAL::IO::make_ply_point_writer(CGAL::make_nth_of_tuple_property_map<0>(keyTpl)),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(keyTpl), CGAL::IO::PLY_property<boost::uint8_t>("red")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(keyTpl), CGAL::IO::PLY_property<boost::uint8_t>("green")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(keyTpl), CGAL::IO::PLY_property<boost::uint8_t>("blue")));
	}
	else
	{
		std::vector<PlyWritePoint_3> verts;
		verts.reserve(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			verts.emplace_back(m_xyz[i * 3U], m_xyz[i * 3U + 1U], m_xyz[i * 3U + 2U]);
		}
		ok = CGAL::IO::write_PLY_with_properties(ofs, verts,
			CGAL::IO::make_ply_point_writer(CGAL::Identity_property_map<PlyWritePoint_3>()));
	}

	if (!ok)
	{
		setErr(errMsg, "Failed to write PLY point data.");
		return false;
	}
	return true;
}

bool PointCloudBackendData::readPointCloudPlySidecar(const std::string& utf8Path, std::string* errMsg)
{
	return readPointCloudFromPlyFile(utf8Path, errMsg);
}

nlohmann::json PointCloudBackendData::snapshotPropertyRows(const BackendDataManager* mgr) const
{
	nlohmann::json rows = BackendDataBase::snapshotPropertyRows(mgr);
	property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::appendRows(m_attributes, *this, rows);
	return rows;
}

bool PointCloudBackendData::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
	const BackendDataManager* mgr)
{
	if (property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::apply(
			m_attributes, *this, key, value, errMsg))
	{
		return true;
	}
	return BackendDataBase::applyPropertyChange(key, value, errMsg, mgr);
}

void PointCloudBackendData::saveDerivedJson(nlohmann::json& out) const
{
	if (m_xyz.empty() || (m_xyz.size() % 3U) != 0U)
	{
		return;
	}
	// 几何真源由 ProjectPackageIo 写 objects/{id}.ply；JSON 仅保留元数据
	out["geometry"] = nlohmann::json{
		{ "kind", "points" },
		{ "storage", "ply_sidecar" },
		{ "pointCount", geometryElementCount() } };
}

bool PointCloudBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	if (!in.contains("geometry"))
	{
		return true;
	}
	const nlohmann::json geo = in["geometry"];
	if (!geo.is_object())
	{
		if (errMsg)
		{
			*errMsg = "Point cloud geometry must be object.";
		}
		return false;
	}
	if (geo.value("kind", std::string()) != "points")
	{
		if (errMsg)
		{
			*errMsg = "Point cloud geometry kind mismatch.";
		}
		return false;
	}
	const std::string xyzBase64 = geo.value("xyzBase64", std::string());
	if (!xyzBase64.empty())
	{
		const std::string rgbaBase64 = geo.value("rgbaPerVertexBase64", std::string());
		if (!readProjectEmbeddedGeometry(xyzBase64, rgbaBase64))
		{
			if (errMsg)
			{
				*errMsg = "Point cloud geometry decode failed.";
			}
			return false;
		}
		return true;
	}
	// ply_sidecar：由 Host 从 assetRelativePath / plySidecar 加载
	return true;
}

