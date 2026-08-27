/// @file PointCloudBackendData.cpp
/// @brief PointCloud 后端数据

#include "PointCloudBackendData.h"

#include "BackendImporters.h"
#include "RunLogger.h"
#include "../../PropertyCore/inc/PropertyAttribute.h"
#include "BackendSpatial.h"
#include "BackendTypeIdentity.h"
#include "PlyIo.h"
#include "geometry_base64.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <tuple>

#include <CGAL/IO/io.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/Simple_cartesian.h>
#include <boost/cstdint.hpp>

PointCloudBackendData::PointCloudBackendData()
{
	setName(backend_type::kCatalogPointCloud);
	BackendColor c;
	c.r = 0.65f;
	c.g = 0.82f;
	c.b = 0.95f;
	c.a = 1.0f;
	m_color = c;
	appendStandardAttributesForCapabilities(*this, m_attributes);
}

std::string PointCloudBackendData::className() const
{
	return backend_type::kClassPointCloud;
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
	bumpGeometryRevision();
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
	bumpGeometryRevision();
}

void PointCloudBackendData::setBounds(const BackendBoundingBox& bounds)
{
	m_bounds = bounds;
	// B4: 独立调用时 Host 包围盒缓存需失效；当前无独立调用方，属潜在陷阱
	bumpGeometryRevision();
}

void PointCloudBackendData::setPointBuffers(std::vector<float> xyz, std::vector<float> rgbaPerVertex)
{
	setPointBuffers(std::move(xyz), std::move(rgbaPerVertex), {});
}

void PointCloudBackendData::setPointBuffers(std::vector<float> xyz, std::vector<float> rgbaPerVertex,
											std::vector<float> normalsNxNyNz)
{
	if (xyz.size() % 3U != 0U)
	{
		RunLogger::warn("[PointCloudBackendData] setPointBuffers: bad xyz size, keep existing geometry.");
		return;
	}
	const std::size_t n = xyz.size() / 3U;
	if (!rgbaPerVertex.empty() && rgbaPerVertex.size() != n * 4U)
	{
		RunLogger::warn("[PointCloudBackendData] setPointBuffers: rgba size mismatch, discard rgba.");
		rgbaPerVertex.clear();
	}
	if (!normalsNxNyNz.empty() && normalsNxNyNz.size() != n * 3U)
	{
		RunLogger::warn("[PointCloudBackendData] setPointBuffers: normals size mismatch, discard normals.");
		normalsNxNyNz.clear();
	}
	m_xyz = std::move(xyz);
	m_rgbaVertex = std::move(rgbaPerVertex);
	m_normals = std::move(normalsNxNyNz);
	m_pointCount = m_xyz.empty() ? 0U : n;
	recomputeBoundsFromPoints();
	bumpGeometryRevision();
}

void PointCloudBackendData::setPointNormals(std::vector<float> normalsNxNyNz)
{
	const std::size_t n = m_xyz.size() / 3U;
	if (n == 0U || normalsNxNyNz.size() != n * 3U)
	{
		RunLogger::warn("[PointCloudBackendData] setPointNormals: size mismatch, clear normals.");
		m_normals.clear();
		bumpGeometryRevision();
		return;
	}
	m_normals = std::move(normalsNxNyNz);
	bumpGeometryRevision();
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

bool PointCloudBackendData::writeProjectEmbeddedGeometry(std::string& outXyzBase64,
														 std::string& outRgbaPerVertexBase64) const
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

bool PointCloudBackendData::readProjectEmbeddedGeometry(const std::string& xyzBase64,
														const std::string& rgbaPerVertexBase64)
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

namespace
{
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
		const bool rgbOk = CGAL::IO::read_PLY_with_properties(
			is, std::back_inserter(withRgb),
			CGAL::IO::make_ply_point_reader(CGAL::make_nth_of_tuple_property_map<0>(keyRgb)),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(keyRgb),
						   CGAL::IO::PLY_property<boost::uint8_t>("red")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(keyRgb),
						   CGAL::IO::PLY_property<boost::uint8_t>("green")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(keyRgb),
						   CGAL::IO::PLY_property<boost::uint8_t>("blue")));
		if (rgbOk && !withRgb.empty())
		{
			cgalRgbRowsToBackend(withRgb, dst);
			return true;
		}
	}

	// P3-3: RGB 失败后复用同一文件流；read_PLY 自文件头解析 header，不可先跳过 end_header
	is.clear();
	is.seekg(0);
	if (!is)
	{
		setErr(errMsg, "Cannot rewind point PLY file.");
		return false;
	}
	CGAL::IO::set_mode(is, scan.isAscii ? CGAL::IO::ASCII : CGAL::IO::BINARY);
	std::vector<PlyReadPoint_3> pts;
	const bool ptsOk = CGAL::IO::read_PLY<PlyReadPoint_3>(is, std::back_inserter(pts));
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
	if (!scan.isAscii || scan.vertexCount == 0U || scan.vertexHasListProperty)
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
	xyz.reserve(scan.vertexCount * 3U);
	if (wantRgb)
	{
		rgba.reserve(scan.vertexCount * 4U);
	}

	// P3-3: 64 列上限放宽到 256，覆盖绝大多数 PLY 属性数；
	// 超上限才报错（避免 rgb 列靠后时误报 missing color）
	constexpr int kMaxPlyColumns = 256;
	double vals[kMaxPlyColumns];
	for (std::size_t vi = 0; vi < scan.vertexCount; ++vi)
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
		const int nc = splitAsciiNumbers(line, vals, kMaxPlyColumns);
		if (nc >= kMaxPlyColumns)
		{
			setErr(errMsg, "ASCII PLY: vertex line exceeds 256 columns (unsupported).");
			return false;
		}
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

static std::size_t plyPropertyByteSize(const std::string& type)
{
	if (type == "char" || type == "uchar" || type == "int8" || type == "uint8")
	{
		return 1U;
	}
	if (type == "short" || type == "ushort" || type == "int16" || type == "uint16")
	{
		return 2U;
	}
	if (type == "int" || type == "uint" || type == "float" || type == "int32" || type == "uint32")
	{
		return 4U;
	}
	if (type == "double" || type == "int64" || type == "uint64")
	{
		return 8U;
	}
	return 4U;
}

static std::size_t plyPropertyOffset(const PlyHeaderInfo& scan, int propIndex)
{
	std::size_t off = 0U;
	for (int i = 0; i < propIndex; ++i)
	{
		off += plyPropertyByteSize(scan.vertexProperties[static_cast<std::size_t>(i)].type);
	}
	return off;
}

static float readPlyScalarAsFloat(const std::uint8_t* row, std::size_t off, const std::string& type)
{
	if (type == "float")
	{
		float v = 0.f;
		std::memcpy(&v, row + off, sizeof(float));
		return v;
	}
	if (type == "double")
	{
		double v = 0.0;
		std::memcpy(&v, row + off, sizeof(double));
		return static_cast<float>(v);
	}
	if (type == "uchar" || type == "uint8")
	{
		return static_cast<float>(row[off]);
	}
	if (type == "char" || type == "int8")
	{
		return static_cast<float>(static_cast<std::int8_t>(row[off]));
	}
	if (type == "ushort" || type == "uint16")
	{
		std::uint16_t v = 0;
		std::memcpy(&v, row + off, sizeof(std::uint16_t));
		return static_cast<float>(v);
	}
	if (type == "short" || type == "int16")
	{
		std::int16_t v = 0;
		std::memcpy(&v, row + off, sizeof(std::int16_t));
		return static_cast<float>(v);
	}
	if (type == "uint" || type == "uint32")
	{
		std::uint32_t v = 0;
		std::memcpy(&v, row + off, sizeof(std::uint32_t));
		return static_cast<float>(v);
	}
	if (type == "int" || type == "int32")
	{
		std::int32_t v = 0;
		std::memcpy(&v, row + off, sizeof(std::int32_t));
		return static_cast<float>(v);
	}
	return 0.f;
}

static bool readBinaryPlyFlexible(const std::string& utf8Path, const PlyHeaderInfo& scan, PointCloudBackendData& dst,
								  std::string* errMsg)
{
	if (scan.isAscii || scan.vertexCount == 0U || scan.vertexHasListProperty)
	{
		setErr(errMsg, "Binary PLY fallback needs binary vertex section without list properties.");
		return false;
	}
	if (scan.ix < 0 || scan.iy < 0 || scan.iz < 0)
	{
		setErr(errMsg, "Binary PLY fallback: missing x/y/z in header.");
		return false;
	}
	if (scan.vertexProperties.empty())
	{
		setErr(errMsg, "Binary PLY fallback: no vertex property layout in header.");
		return false;
	}

	std::ifstream fin{std::filesystem::path{utf8Path}, std::ios::binary};
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

	std::size_t stride = 0U;
	for (const auto& prop : scan.vertexProperties)
	{
		stride += plyPropertyByteSize(prop.type);
	}
	if (stride == 0U)
	{
		setErr(errMsg, "Binary PLY fallback: invalid vertex stride.");
		return false;
	}

	const bool wantRgb = (scan.ir >= 0 && scan.ig >= 0 && scan.ib >= 0);
	std::vector<float> xyz;
	std::vector<float> rgba;
	xyz.reserve(scan.vertexCount * 3U);
	if (wantRgb)
	{
		rgba.reserve(scan.vertexCount * 4U);
	}

	std::vector<std::uint8_t> row(stride);
	for (std::size_t vi = 0; vi < scan.vertexCount; ++vi)
	{
		if (!fin.read(reinterpret_cast<char*>(row.data()), static_cast<std::streamsize>(stride)))
		{
			setErr(errMsg, "Binary PLY: unexpected end of file in vertex section.");
			return false;
		}
		const auto& props = scan.vertexProperties;
		xyz.push_back(readPlyScalarAsFloat(row.data(), plyPropertyOffset(scan, scan.ix), props[static_cast<std::size_t>(scan.ix)].type));
		xyz.push_back(readPlyScalarAsFloat(row.data(), plyPropertyOffset(scan, scan.iy), props[static_cast<std::size_t>(scan.iy)].type));
		xyz.push_back(readPlyScalarAsFloat(row.data(), plyPropertyOffset(scan, scan.iz), props[static_cast<std::size_t>(scan.iz)].type));
		if (wantRgb)
		{
			const float r = readPlyScalarAsFloat(row.data(), plyPropertyOffset(scan, scan.ir),
												 props[static_cast<std::size_t>(scan.ir)].type) /
							255.0f;
			const float g = readPlyScalarAsFloat(row.data(), plyPropertyOffset(scan, scan.ig),
												 props[static_cast<std::size_t>(scan.ig)].type) /
							255.0f;
			const float b = readPlyScalarAsFloat(row.data(), plyPropertyOffset(scan, scan.ib),
												 props[static_cast<std::size_t>(scan.ib)].type) /
							255.0f;
			rgba.push_back(r);
			rgba.push_back(g);
			rgba.push_back(b);
			rgba.push_back(1.0f);
		}
	}

	if (xyz.empty())
	{
		setErr(errMsg, "Binary PLY: no vertices read.");
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
	if (scan.cgalFormatOnLine2 && readPlyWithCgalFromPath(utf8Path, scan, *this, &cgalErr) &&
		!pointPositionsXyz().empty())
	{
		return true;
	}

	std::string asciiErr;
	if (scan.isAscii && scan.vertexCount > 0 && !scan.vertexHasListProperty &&
		readAsciiPlyFlexible(utf8Path, scan, *this, &asciiErr) && !pointPositionsXyz().empty())
	{
		return true;
	}

	std::string binaryErr;
	if (!scan.isAscii && scan.vertexCount > 0 && !scan.vertexHasListProperty &&
		readBinaryPlyFlexible(utf8Path, scan, *this, &binaryErr) && !pointPositionsXyz().empty())
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
		if (!binaryErr.empty())
		{
			if (!errMsg->empty())
			{
				errMsg->append(" | ");
			}
			errMsg->append(binaryErr);
		}
		if (errMsg->empty())
		{
			setErr(errMsg, "Could not read PLY point cloud (CGAL + flexible fallback).");
		}
	}
	return false;
}

bool PointCloudBackendData::loadFromFile(const std::string& path, std::string* errMsg)
{
	return backend_io::loadPointCloudFromFile(*this, path, errMsg);
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
			verts.emplace_back(PlyWritePoint_3(x, y, z), floatChannelToU8(rf), floatChannelToU8(gf),
							   floatChannelToU8(bf));
		}
		VtxRgbWrite keyTpl{};
		ok = CGAL::IO::write_PLY_with_properties(
			ofs, verts, CGAL::IO::make_ply_point_writer(CGAL::make_nth_of_tuple_property_map<0>(keyTpl)),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(keyTpl),
						   CGAL::IO::PLY_property<boost::uint8_t>("red")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(keyTpl),
						   CGAL::IO::PLY_property<boost::uint8_t>("green")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(keyTpl),
						   CGAL::IO::PLY_property<boost::uint8_t>("blue")));
	}
	else
	{
		std::vector<PlyWritePoint_3> verts;
		verts.reserve(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			verts.emplace_back(m_xyz[i * 3U], m_xyz[i * 3U + 1U], m_xyz[i * 3U + 2U]);
		}
		ok = CGAL::IO::write_PLY_with_properties(
			ofs, verts, CGAL::IO::make_ply_point_writer(CGAL::Identity_property_map<PlyWritePoint_3>()));
	}

	if (!ok)
	{
		setErr(errMsg, "Failed to write PLY point data.");
		return false;
	}
	return true;
}

bool PointCloudBackendData::writePointCloudPlySidecar(const std::string& utf8Path,
													  const std::vector<float>& xyzOverride, std::string* errMsg) const
{
	if (xyzOverride.empty() || (xyzOverride.size() % 3U) != 0U)
	{
		setErr(errMsg, "No point coordinates to write.");
		return false;
	}
	const std::size_t n = xyzOverride.size() / 3U;
	const bool hasRgba = hasPerVertexColors() && m_rgbaVertex.size() == n * 4U;

	std::ofstream ofs(std::filesystem::path(utf8Path), std::ios::binary);
	if (!ofs)
	{
		setErr(errMsg, "Cannot open file for writing.");
		return false;
	}
	CGAL::IO::set_mode(ofs, CGAL::IO::BINARY);

	bool ok = false;
	if (hasRgba)
	{
		std::vector<VtxRgbWrite> verts;
		verts.reserve(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			const float x = xyzOverride[i * 3U];
			const float y = xyzOverride[i * 3U + 1U];
			const float z = xyzOverride[i * 3U + 2U];
			const float rf = m_rgbaVertex[i * 4U];
			const float gf = m_rgbaVertex[i * 4U + 1U];
			const float bf = m_rgbaVertex[i * 4U + 2U];
			verts.emplace_back(PlyWritePoint_3(x, y, z), floatChannelToU8(rf), floatChannelToU8(gf),
							   floatChannelToU8(bf));
		}
		VtxRgbWrite keyTpl{};
		ok = CGAL::IO::write_PLY_with_properties(
			ofs, verts, CGAL::IO::make_ply_point_writer(CGAL::make_nth_of_tuple_property_map<0>(keyTpl)),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<1>(keyTpl),
						   CGAL::IO::PLY_property<boost::uint8_t>("red")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<2>(keyTpl),
						   CGAL::IO::PLY_property<boost::uint8_t>("green")),
			std::make_pair(CGAL::make_nth_of_tuple_property_map<3>(keyTpl),
						   CGAL::IO::PLY_property<boost::uint8_t>("blue")));
	}
	else
	{
		std::vector<PlyWritePoint_3> verts;
		verts.reserve(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			verts.emplace_back(xyzOverride[i * 3U], xyzOverride[i * 3U + 1U], xyzOverride[i * 3U + 2U]);
		}
		ok = CGAL::IO::write_PLY_with_properties(
			ofs, verts, CGAL::IO::make_ply_point_writer(CGAL::Identity_property_map<PlyWritePoint_3>()));
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

std::vector<float> PointCloudBackendData::worldPositionsXyz() const
{
	if (m_xyz.empty())
	{
		return {};
	}
	std::vector<float> transformed = m_xyz;
	transformXyzToWorld(transformed, worldMatrix());
	return transformed;
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
	if (property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::apply(m_attributes, *this, key, value,
																					  errMsg))
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
	out["geometry"] =
		nlohmann::json{{"kind", "points"}, {"storage", "ply_sidecar"}, {"pointCount", geometryElementCount()}};
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
	// ply_sidecar：字节由 Host ProjectPackageIo 外部写盘，Data 侧只记元数据；
	// 元数据声明了点数而 sidecar 字节缺失时，留下的是静默空对象——此处无法读到字节，
	// 只能告警提示外部协作者（Host）对账，真正的完整性校验在 Host 加载 sidecar 时做
	const std::size_t declaredCount = static_cast<std::size_t>(geo.value("pointCount", 0.0));
	if (declaredCount > 0U)
	{
		RunLogger::warn("[PointCloudBackendData] geometry declares pointCount=" + std::to_string(declaredCount) +
						" but no embedded xyzBase64; expecting external PLY sidecar. If sidecar is missing, "
						"object will stay empty.");
	}
	return true;
}
