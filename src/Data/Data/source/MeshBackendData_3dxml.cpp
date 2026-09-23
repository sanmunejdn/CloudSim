/// @file MeshBackendData_3dxml.cpp
/// @brief 3DXML（ZIP+PolygonalRep）→ 三角 soup / 装配层级；解析对齐 robdts dxmlRead/dxmlStructureRead

#include "pch.h"

#include "../third_party/tinyxml2/tinyxml2.h"
#include "MeshBackendData.h"
#include "MeshBackendData_loaders.h"
#include "RunLogger.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <zlib.h>

namespace mesh_backend_load
{
namespace
{
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

struct DxmlFacePrim
{
	std::vector<std::vector<int>> strips;
	std::vector<std::vector<int>> triangles;
	std::vector<std::vector<int>> fans;
};

struct DxmlPosition
{
	std::vector<std::vector<float>> vertices;
	std::vector<std::vector<float>> normals;
};

struct DxmlRepBundle
{
	std::vector<DxmlPosition> positions;
	std::vector<DxmlFacePrim> faces;
	std::vector<int> solidTypes;
};

struct DxmlProduct
{
	std::string name;
	std::vector<DxmlRepBundle> reps;
	std::vector<std::vector<float>> matrices; // 与 reps 对齐；每项 12 元列主序 3x4
};

struct DxmlTreeNode
{
	std::string id;
	std::string name;
	std::string type;
	std::vector<std::string> files;
	std::vector<std::unique_ptr<DxmlTreeNode>> children;
	std::vector<float> matrixValues;
	std::vector<std::vector<float>> filesMatrixValues;
};

bool readLe16(std::istream& in, std::uint16_t* v)
{
	unsigned char buf[2];
	in.read(reinterpret_cast<char*>(buf), 2);
	if (!in)
	{
		return false;
	}
	*v = static_cast<std::uint16_t>(buf[0] | (static_cast<std::uint16_t>(buf[1]) << 8));
	return true;
}

bool readLe32(std::istream& in, std::uint32_t* v)
{
	unsigned char buf[4];
	in.read(reinterpret_cast<char*>(buf), 4);
	if (!in)
	{
		return false;
	}
	*v = static_cast<std::uint32_t>(buf[0]) | (static_cast<std::uint32_t>(buf[1]) << 8) |
		 (static_cast<std::uint32_t>(buf[2]) << 16) | (static_cast<std::uint32_t>(buf[3]) << 24);
	return true;
}

bool inflateRaw(const std::vector<char>& compressed, std::uint32_t uncompressedSize, std::vector<char>* out,
				std::string* errMsg)
{
	out->assign(uncompressedSize, '\0');
	z_stream strm{};
	strm.next_in = reinterpret_cast<Bytef*>(const_cast<char*>(compressed.data()));
	strm.avail_in = static_cast<uInt>(compressed.size());
	strm.next_out = reinterpret_cast<Bytef*>(out->data());
	strm.avail_out = uncompressedSize;
	if (inflateInit2(&strm, -MAX_WBITS) != Z_OK)
	{
		meshLoadErr(errMsg, "zlib inflateInit2 failed.");
		return false;
	}
	const int rc = inflate(&strm, Z_FINISH);
	inflateEnd(&strm);
	if (rc != Z_STREAM_END)
	{
		meshLoadErr(errMsg, "zlib inflate failed for zip entry.");
		return false;
	}
	out->resize(static_cast<std::size_t>(strm.total_out));
	return true;
}

class DxmlZipArchive
{
public:
	bool open(const std::string& nativePath, std::string* errMsg)
	{
		m_file.open(nativePath, std::ios::binary);
		if (!m_file)
		{
			meshLoadErr(errMsg, "Cannot open 3DXML archive.");
			return false;
		}
		m_file.seekg(0, std::ios::end);
		const std::int64_t fileSize = static_cast<std::int64_t>(m_file.tellg());
		if (fileSize < 22)
		{
			meshLoadErr(errMsg, "3DXML archive too small.");
			return false;
		}
		std::int64_t eocdOff = -1;
		{
			const std::int64_t readSpan = (fileSize < 65557) ? fileSize : 65557;
			m_file.seekg(fileSize - readSpan);
			std::vector<char> tail(static_cast<std::size_t>(readSpan));
			m_file.read(tail.data(), readSpan);
			for (int i = static_cast<int>(tail.size()) - 22; i >= 0; --i)
			{
				const auto* p = reinterpret_cast<const unsigned char*>(tail.data() + i);
				const std::uint32_t sig = static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
										  (static_cast<std::uint32_t>(p[2]) << 16) |
										  (static_cast<std::uint32_t>(p[3]) << 24);
				if (sig == 0x06054b50u)
				{
					eocdOff = fileSize - readSpan + i;
					break;
				}
			}
		}
		if (eocdOff < 0)
		{
			meshLoadErr(errMsg, "3DXML: end of central directory not found.");
			return false;
		}
		m_file.clear();
		m_file.seekg(eocdOff + 10);
		std::uint16_t totalEntries = 0;
		std::uint32_t centralSize = 0;
		std::uint32_t centralOffset = 0;
		if (!readLe16(m_file, &totalEntries) || !readLe32(m_file, &centralSize) || !readLe32(m_file, &centralOffset))
		{
			meshLoadErr(errMsg, "3DXML: invalid EOCD.");
			return false;
		}
		(void)centralSize;
		m_file.seekg(centralOffset);
		for (std::uint16_t i = 0; i < totalEntries; ++i)
		{
			std::uint32_t sig = 0;
			if (!readLe32(m_file, &sig) || sig != 0x02014b50u)
			{
				meshLoadErr(errMsg, "3DXML: invalid central directory.");
				return false;
			}
			std::uint16_t verMade = 0;
			std::uint16_t verExtract = 0;
			std::uint16_t gpbf = 0;
			std::uint16_t method = 0;
			std::uint16_t modTime = 0;
			std::uint16_t modDate = 0;
			if (!readLe16(m_file, &verMade) || !readLe16(m_file, &verExtract) || !readLe16(m_file, &gpbf) ||
				!readLe16(m_file, &method) || !readLe16(m_file, &modTime) || !readLe16(m_file, &modDate))
			{
				return false;
			}
			(void)verMade;
			(void)verExtract;
			(void)gpbf;
			(void)modTime;
			(void)modDate;
			std::uint32_t crc = 0;
			std::uint32_t csize = 0;
			std::uint32_t usize = 0;
			if (!readLe32(m_file, &crc) || !readLe32(m_file, &csize) || !readLe32(m_file, &usize))
			{
				return false;
			}
			(void)crc;
			std::uint16_t nameLen = 0;
			std::uint16_t extraLen = 0;
			std::uint16_t commentLen = 0;
			if (!readLe16(m_file, &nameLen) || !readLe16(m_file, &extraLen) || !readLe16(m_file, &commentLen))
			{
				return false;
			}
			std::uint16_t diskStart = 0;
			std::uint16_t intAttr = 0;
			std::uint32_t extAttr = 0;
			std::uint32_t localHdrOff = 0;
			if (!readLe16(m_file, &diskStart) || !readLe16(m_file, &intAttr) || !readLe32(m_file, &extAttr) ||
				!readLe32(m_file, &localHdrOff))
			{
				return false;
			}
			(void)diskStart;
			(void)intAttr;
			(void)extAttr;
			std::string name(nameLen, '\0');
			m_file.read(name.data(), nameLen);
			if (!m_file)
			{
				meshLoadErr(errMsg, "3DXML: truncated file name.");
				return false;
			}
			m_file.seekg(extraLen + commentLen, std::ios::cur);
			Entry e;
			e.name = name;
			e.method = method;
			e.compSize = csize;
			e.uncompSize = usize;
			e.localHdrOff = localHdrOff;
			m_entries[e.name] = e;
			const auto slash = e.name.find_last_of("/\\");
			if (slash != std::string::npos)
			{
				m_entries[e.name.substr(slash + 1)] = e;
			}
		}
		return true;
	}

	bool readEntry(const std::string& name, std::string* outText, std::string* errMsg)
	{
		auto it = m_entries.find(name);
		if (it == m_entries.end())
		{
			meshLoadErr(errMsg, ("3DXML entry not found: " + name).c_str());
			return false;
		}
		const Entry& e = it->second;
		m_file.clear();
		m_file.seekg(e.localHdrOff);
		std::uint32_t lsig = 0;
		if (!readLe32(m_file, &lsig) || lsig != 0x04034b50u)
		{
			meshLoadErr(errMsg, "3DXML: bad local file header.");
			return false;
		}
		std::uint16_t ver = 0;
		std::uint16_t gpbf = 0;
		std::uint16_t method = 0;
		std::uint16_t mt = 0;
		std::uint16_t md = 0;
		if (!readLe16(m_file, &ver) || !readLe16(m_file, &gpbf) || !readLe16(m_file, &method) ||
			!readLe16(m_file, &mt) || !readLe16(m_file, &md))
		{
			return false;
		}
		(void)ver;
		(void)gpbf;
		(void)mt;
		(void)md;
		std::uint32_t crc = 0;
		std::uint32_t csize = 0;
		std::uint32_t usize = 0;
		if (!readLe32(m_file, &crc) || !readLe32(m_file, &csize) || !readLe32(m_file, &usize))
		{
			return false;
		}
		(void)crc;
		if (csize == 0)
		{
			csize = e.compSize;
		}
		if (usize == 0)
		{
			usize = e.uncompSize;
		}
		std::uint16_t nameLen = 0;
		std::uint16_t extraLen = 0;
		if (!readLe16(m_file, &nameLen) || !readLe16(m_file, &extraLen))
		{
			return false;
		}
		m_file.seekg(nameLen + extraLen, std::ios::cur);
		std::vector<char> payload(csize);
		m_file.read(payload.data(), static_cast<std::streamsize>(csize));
		if (!m_file && m_file.gcount() != static_cast<std::streamsize>(csize))
		{
			meshLoadErr(errMsg, "3DXML: truncated zip payload.");
			return false;
		}
		std::vector<char> raw;
		if (method == 0)
		{
			raw = std::move(payload);
		}
		else if (method == 8)
		{
			if (!inflateRaw(payload, usize, &raw, errMsg))
			{
				return false;
			}
		}
		else
		{
			meshLoadErr(errMsg, "3DXML: unsupported zip compression method.");
			return false;
		}
		*outText = std::string(raw.begin(), raw.end());
		return true;
	}

private:
	struct Entry
	{
		std::string name;
		std::uint16_t method = 0;
		std::uint32_t compSize = 0;
		std::uint32_t uncompSize = 0;
		std::uint32_t localHdrOff = 0;
	};
	std::ifstream m_file;
	std::map<std::string, Entry> m_entries;
};

template <typename T>
std::vector<std::vector<T>> parseGroupedNumbers(const char* attribute)
{
	std::vector<std::vector<T>> result;
	if (!attribute)
	{
		return result;
	}
	std::stringstream ss(attribute);
	std::string group;
	while (std::getline(ss, group, ','))
	{
		std::vector<T> numbers;
		std::stringstream gs(group);
		T number{};
		while (gs >> number)
		{
			numbers.push_back(number);
		}
		if (!numbers.empty())
		{
			result.push_back(std::move(numbers));
		}
	}
	return result;
}

void parseFaceAttribute(XMLElement* face, const char* attrName, std::vector<std::vector<int>>* out)
{
	const char* attrValue = face->Attribute(attrName);
	if (!attrValue)
	{
		return;
	}
	auto groups = parseGroupedNumbers<int>(attrValue);
	out->insert(out->end(), groups.begin(), groups.end());
}

void parseFaces(XMLElement* facesElement, DxmlFacePrim* out)
{
	if (!facesElement || !out)
	{
		return;
	}
	for (XMLElement* face = facesElement->FirstChildElement("Face"); face != nullptr;
		 face = face->NextSiblingElement("Face"))
	{
		parseFaceAttribute(face, "strips", &out->strips);
		parseFaceAttribute(face, "Face strips", &out->strips);
		parseFaceAttribute(face, "triangles", &out->triangles);
		parseFaceAttribute(face, "Face triangles", &out->triangles);
		parseFaceAttribute(face, "fans", &out->fans);
		parseFaceAttribute(face, "Face fans", &out->fans);
	}
}

void parsePositions(XMLElement* vertexBufferElement, DxmlPosition* out)
{
	if (!vertexBufferElement || !out)
	{
		return;
	}
	XMLElement* positionsElement = vertexBufferElement->FirstChildElement("Positions");
	XMLElement* normalsElement = vertexBufferElement->FirstChildElement("Normals");
	if (!positionsElement || !positionsElement->GetText())
	{
		return;
	}
	out->vertices = parseGroupedNumbers<float>(positionsElement->GetText());
	if (normalsElement && normalsElement->GetText())
	{
		out->normals = parseGroupedNumbers<float>(normalsElement->GetText());
	}
}

void processRepElement(XMLElement* element, DxmlRepBundle* bundle, bool readFaceInfo)
{
	if (!element || !bundle)
	{
		return;
	}
	const char* xsiType = element->Attribute("xsi:type");
	if (!xsiType)
	{
		for (XMLElement* child = element->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
		{
			processRepElement(child, bundle, readFaceInfo);
		}
		return;
	}
	if (std::strcmp(xsiType, "BagRepType") == 0)
	{
		for (XMLElement* child = element->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
		{
			processRepElement(child, bundle, readFaceInfo);
		}
		return;
	}
	if (std::strcmp(xsiType, "PolygonalRepType") != 0)
	{
		return;
	}

	DxmlFacePrim facePrim;
	DxmlPosition position;
	int solid = -1;
	const char* solidAttr = element->Attribute("solid");
	if (solidAttr)
	{
		solid = std::atoi(solidAttr);
	}
	else if (readFaceInfo)
	{
		solid = 0;
	}

	if (XMLElement* faces = element->FirstChildElement("Faces"))
	{
		parseFaces(faces, &facePrim);
	}
	if (XMLElement* vertexBuffer = element->FirstChildElement("VertexBuffer"))
	{
		parsePositions(vertexBuffer, &position);
	}
	if (position.vertices.empty() || (facePrim.strips.empty() && facePrim.triangles.empty() && facePrim.fans.empty()))
	{
		return;
	}
	if (solid < 0)
	{
		solid = 1;
	}
	bundle->positions.push_back(std::move(position));
	bundle->faces.push_back(std::move(facePrim));
	bundle->solidTypes.push_back(solid);
}

bool parse3DRepXml(const std::string& xml, bool readFaceInfo, DxmlRepBundle* out, std::string* errMsg)
{
	XMLDocument doc;
	if (doc.Parse(xml.c_str(), xml.size()) != tinyxml2::XML_SUCCESS)
	{
		meshLoadErr(errMsg, "Failed to parse 3DRep XML.");
		return false;
	}
	XMLElement* root = doc.RootElement();
	if (!root)
	{
		meshLoadErr(errMsg, "3DRep root missing.");
		return false;
	}
	for (XMLElement* child = root->FirstChildElement(); child != nullptr; child = child->NextSiblingElement())
	{
		processRepElement(child, out, readFaceInfo);
	}
	return !out->positions.empty();
}

std::string urnToFileName(const std::string& input)
{
	const auto pos = input.rfind(':');
	if (pos == std::string::npos)
	{
		return input;
	}
	return input.substr(pos + 1);
}

void applyMatrix12(const std::vector<float>& m12, float x, float y, float z, float& ox, float& oy, float& oz)
{
	if (m12.size() != 12U)
	{
		ox = x;
		oy = y;
		oz = z;
		return;
	}
	ox = m12[0] * x + m12[3] * y + m12[6] * z + m12[9];
	oy = m12[1] * x + m12[4] * y + m12[7] * z + m12[10];
	oz = m12[2] * x + m12[5] * y + m12[8] * z + m12[11];
}

void pushTriFromIndices(std::vector<float>& soup, const std::vector<std::vector<float>>& verts, int i0, int i1, int i2,
						bool reverse, const std::vector<float>& m12)
{
	if (i0 < 0 || i1 < 0 || i2 < 0)
	{
		return;
	}
	if (static_cast<std::size_t>(i0) >= verts.size() || static_cast<std::size_t>(i1) >= verts.size() ||
		static_cast<std::size_t>(i2) >= verts.size())
	{
		return;
	}
	const auto& a = verts[static_cast<std::size_t>(i0)];
	const auto& b = verts[static_cast<std::size_t>(i1)];
	const auto& c = verts[static_cast<std::size_t>(i2)];
	if (a.size() < 3U || b.size() < 3U || c.size() < 3U)
	{
		return;
	}
	float ax = 0;
	float ay = 0;
	float az = 0;
	float bx = 0;
	float by = 0;
	float bz = 0;
	float cx = 0;
	float cy = 0;
	float cz = 0;
	applyMatrix12(m12, a[0], a[1], a[2], ax, ay, az);
	applyMatrix12(m12, b[0], b[1], b[2], bx, by, bz);
	applyMatrix12(m12, c[0], c[1], c[2], cx, cy, cz);
	if (reverse)
	{
		meshPushTri(soup, cx, cy, cz, bx, by, bz, ax, ay, az);
	}
	else
	{
		meshPushTri(soup, ax, ay, az, bx, by, bz, cx, cy, cz);
	}
}

/// RelativeMatrix 含反射时整体绕序翻转
bool matrixFlipsWinding(const std::vector<float>& m12)
{
	if (m12.size() != 12U)
	{
		return false;
	}
	const float det = m12[0] * (m12[4] * m12[8] - m12[5] * m12[7]) - m12[3] * (m12[1] * m12[8] - m12[2] * m12[7]) +
					  m12[6] * (m12[1] * m12[5] - m12[2] * m12[4]);
	return det < 0.0f;
}

void tessellateRep(const DxmlRepBundle& rep, const std::vector<float>& m12, std::vector<float>& soup)
{
	const std::size_t n = rep.positions.size();
	if (n != rep.faces.size() || n != rep.solidTypes.size())
	{
		return;
	}
	const bool flipByMatrix = matrixFlipsWinding(m12);
	for (std::size_t i = 0; i < n; ++i)
	{
		const auto& verts = rep.positions[i].vertices;
		const auto& face = rep.faces[i];
		// solid=0 表示文件内已按反向绕序标注
		const bool reverseSolid = (rep.solidTypes[i] == 0) ^ flipByMatrix;
		for (const auto& strip : face.strips)
		{
			const int countFace = static_cast<int>(strip.size()) - 2;
			// strip 相邻三角须交替绕序，否则每隔一面法向反
			for (int f = 0; f < countFace; ++f)
			{
				const bool reverse = reverseSolid ^ ((f % 2) != 0);
				pushTriFromIndices(soup, verts, strip[static_cast<std::size_t>(f)],
								   strip[static_cast<std::size_t>(f + 1)], strip[static_cast<std::size_t>(f + 2)],
								   reverse, m12);
			}
		}
		for (const auto& tri : face.triangles)
		{
			const int countFace = static_cast<int>(tri.size()) / 3;
			for (int f = 0; f < countFace; ++f)
			{
				const int base = f * 3;
				pushTriFromIndices(soup, verts, tri[static_cast<std::size_t>(base)],
								   tri[static_cast<std::size_t>(base + 1)], tri[static_cast<std::size_t>(base + 2)],
								   reverseSolid, m12);
			}
		}
		for (const auto& fan : face.fans)
		{
			const int countFace = static_cast<int>(fan.size()) - 2;
			for (int f = 0; f < countFace; ++f)
			{
				pushTriFromIndices(soup, verts, fan[0], fan[static_cast<std::size_t>(f + 1)],
								   fan[static_cast<std::size_t>(f + 2)], reverseSolid, m12);
			}
		}
	}
}

std::vector<float> identityMatrix12()
{
	return {1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f};
}

std::unique_ptr<DxmlTreeNode> cloneTreeNode(const DxmlTreeNode& src)
{
	auto dst = std::make_unique<DxmlTreeNode>();
	dst->id = src.id;
	dst->name = src.name;
	dst->type = src.type;
	dst->files = src.files;
	dst->matrixValues = src.matrixValues;
	dst->filesMatrixValues = src.filesMatrixValues;
	for (const auto& child : src.children)
	{
		if (child)
		{
			dst->children.push_back(cloneTreeNode(*child));
		}
	}
	return dst;
}

DxmlTreeNode* findNodeChild(const std::string& id, DxmlTreeNode* node)
{
	if (!node)
	{
		return nullptr;
	}
	for (auto& child : node->children)
	{
		if (child->id == id)
		{
			return child.get();
		}
		if (DxmlTreeNode* found = findNodeChild(id, child.get()))
		{
			return found;
		}
	}
	return nullptr;
}

DxmlTreeNode* findNodeWithId(const std::string& id, std::map<std::string, std::unique_ptr<DxmlTreeNode>>& nodes)
{
	auto it = nodes.find(id);
	if (it != nodes.end())
	{
		return it->second.get();
	}
	for (auto& kv : nodes)
	{
		if (DxmlTreeNode* found = findNodeChild(id, kv.second.get()))
		{
			return found;
		}
	}
	return nullptr;
}

void gatherNodeFiles(DxmlTreeNode* src, DxmlTreeNode* dst)
{
	for (auto& child : src->children)
	{
		dst->files.insert(dst->files.end(), child->files.begin(), child->files.end());
		dst->filesMatrixValues.insert(dst->filesMatrixValues.end(), child->filesMatrixValues.begin(),
									  child->filesMatrixValues.end());
		gatherNodeFiles(child.get(), dst);
	}
}

bool loadRepFile(DxmlZipArchive& zip, const std::string& file, bool readFaceInfo, DxmlRepBundle* out,
				 std::string* errMsg)
{
	std::string xml;
	if (!zip.readEntry(file, &xml, errMsg))
	{
		return false;
	}
	return parse3DRepXml(xml, readFaceInfo, out, errMsg);
}

bool buildProductsFromStructure(DxmlZipArchive& zip, std::vector<DxmlProduct>* products, std::string* rootName,
								bool* isAssembly, std::string* errMsg)
{
	std::string manifestXml;
	if (!zip.readEntry("Manifest.xml", &manifestXml, errMsg))
	{
		return false;
	}
	XMLDocument manifestDoc;
	if (manifestDoc.Parse(manifestXml.c_str(), manifestXml.size()) != tinyxml2::XML_SUCCESS)
	{
		meshLoadErr(errMsg, "Failed to parse Manifest.xml.");
		return false;
	}
	XMLElement* mroot = manifestDoc.RootElement();
	if (!mroot || !mroot->FirstChildElement() || !mroot->FirstChildElement()->GetText())
	{
		meshLoadErr(errMsg, "Manifest.xml missing product structure path.");
		return false;
	}
	const std::string structurePath = mroot->FirstChildElement()->GetText();

	std::string structureXml;
	if (!zip.readEntry(structurePath, &structureXml, errMsg))
	{
		return false;
	}
	XMLDocument productDoc;
	if (productDoc.Parse(structureXml.c_str(), structureXml.size()) != tinyxml2::XML_SUCCESS)
	{
		meshLoadErr(errMsg, "Failed to parse ProductStructure XML.");
		return false;
	}
	XMLElement* root = productDoc.RootElement();
	if (!root)
	{
		meshLoadErr(errMsg, "ProductStructure root missing.");
		return false;
	}

	bool readFaceInfo = false;
	if (XMLElement* header = root->FirstChildElement("Header"))
	{
		if (XMLElement* generator = header->FirstChildElement("Generator"))
		{
			const char* g = generator->GetText();
			if (g && std::strstr(g, "SOLIDWORKS") != nullptr)
			{
				readFaceInfo = true;
			}
		}
	}

	XMLElement* ps = root->FirstChildElement("ProductStructure");
	if (!ps)
	{
		meshLoadErr(errMsg, "ProductStructure element missing.");
		return false;
	}
	const char* rootIdAttr = ps->Attribute("root");
	const std::string rootId = rootIdAttr ? rootIdAttr : "1";

	std::map<std::string, std::unique_ptr<DxmlTreeNode>> nodes;

	for (XMLElement* el = ps->FirstChildElement("Reference3D"); el != nullptr;
		 el = el->NextSiblingElement("Reference3D"))
	{
		const char* id = el->Attribute("id");
		if (!id)
		{
			continue;
		}
		std::string name;
		if (XMLElement* vn = el->FirstChildElement("V_Name"))
		{
			if (vn->GetText())
			{
				name = vn->GetText();
			}
		}
		if (name.empty())
		{
			if (const char* n = el->Attribute("name"))
			{
				name = n;
			}
		}
		auto node = std::make_unique<DxmlTreeNode>();
		node->id = id;
		node->name = name;
		node->type = "Reference3D";
		if (nodes.find(id) == nodes.end())
		{
			nodes[id] = std::move(node);
		}
	}

	for (XMLElement* el = ps->FirstChildElement("ReferenceRep"); el != nullptr;
		 el = el->NextSiblingElement("ReferenceRep"))
	{
		const char* id = el->Attribute("id");
		if (!id)
		{
			continue;
		}
		const char* namePtr = el->Attribute("name");
		auto node = std::make_unique<DxmlTreeNode>();
		node->id = id;
		node->name = namePtr ? namePtr : id;
		node->type = "ReferenceRep";
		if (const char* af = el->Attribute("associatedFile"))
		{
			node->files.push_back(urnToFileName(af));
		}
		if (nodes.find(id) == nodes.end())
		{
			nodes[id] = std::move(node);
		}
	}

	for (XMLElement* el = ps->FirstChildElement("InstanceRep"); el != nullptr;
		 el = el->NextSiblingElement("InstanceRep"))
	{
		XMLElement* by = el->FirstChildElement("IsAggregatedBy");
		XMLElement* of = el->FirstChildElement("IsInstanceOf");
		if (!by || !of || !by->GetText() || !of->GetText())
		{
			continue;
		}
		const std::string parentId = by->GetText();
		const std::string childId = of->GetText();
		auto pit = nodes.find(parentId);
		auto cit = nodes.find(childId);
		if (pit == nodes.end() || cit == nodes.end())
		{
			continue;
		}
		pit->second->files.insert(pit->second->files.end(), cit->second->files.begin(), cit->second->files.end());
		nodes.erase(cit);
	}

	for (XMLElement* el = ps->FirstChildElement("Instance3D"); el != nullptr; el = el->NextSiblingElement("Instance3D"))
	{
		XMLElement* by = el->FirstChildElement("IsAggregatedBy");
		XMLElement* of = el->FirstChildElement("IsInstanceOf");
		XMLElement* rel = el->FirstChildElement("RelativeMatrix");
		if (!by || !of || !by->GetText() || !of->GetText())
		{
			continue;
		}
		const std::string parentId = by->GetText();
		const std::string childId = of->GetText();
		std::vector<float> matrixValues = identityMatrix12();
		if (rel && rel->GetText())
		{
			matrixValues.clear();
			std::istringstream iss(rel->GetText());
			float v = 0.f;
			while (iss >> v)
			{
				matrixValues.push_back(v);
			}
			if (matrixValues.size() != 12U)
			{
				matrixValues = identityMatrix12();
			}
		}

		DxmlTreeNode* childSrc = findNodeWithId(childId, nodes);
		DxmlTreeNode* parent = findNodeWithId(parentId, nodes);
		if (!childSrc || !parent)
		{
			continue;
		}
		auto newChild = cloneTreeNode(*childSrc);
		newChild->matrixValues = matrixValues;
		newChild->filesMatrixValues.clear();
		for (std::size_t i = 0; i < newChild->files.size(); ++i)
		{
			newChild->filesMatrixValues.push_back(matrixValues);
		}
		parent->children.push_back(std::move(newChild));
		auto topIt = nodes.find(childId);
		if (topIt != nodes.end())
		{
			nodes.erase(topIt);
		}
	}

	DxmlTreeNode* rootNode = findNodeWithId(rootId, nodes);
	if (!rootNode)
	{
		if (!nodes.empty())
		{
			rootNode = nodes.begin()->second.get();
		}
	}
	if (!rootNode)
	{
		meshLoadErr(errMsg, "3DXML root node not found.");
		return false;
	}
	*rootName = rootNode->name;
	for (auto& child : rootNode->children)
	{
		gatherNodeFiles(child.get(), child.get());
	}

	*isAssembly = !rootNode->children.empty();
	if (rootNode->children.empty())
	{
		for (const auto& file : rootNode->files)
		{
			DxmlProduct product;
			product.name = rootNode->name.empty() ? file : rootNode->name;
			product.matrices.push_back(identityMatrix12());
			DxmlRepBundle bundle;
			std::string localErr;
			if (!loadRepFile(zip, file, readFaceInfo, &bundle, &localErr))
			{
				RunLogger::warn(std::string("[MeshBackendData] 3DXML skip rep: ") + localErr);
				continue;
			}
			product.reps.push_back(std::move(bundle));
			products->push_back(std::move(product));
		}
	}
	else
	{
		for (auto& node : rootNode->children)
		{
			DxmlProduct product;
			product.name = node->name;
			product.matrices = node->filesMatrixValues;
			while (product.matrices.size() < node->files.size())
			{
				product.matrices.push_back(node->matrixValues.empty() ? identityMatrix12() : node->matrixValues);
			}
			for (std::size_t fi = 0; fi < node->files.size(); ++fi)
			{
				DxmlRepBundle bundle;
				std::string localErr;
				if (!loadRepFile(zip, node->files[fi], readFaceInfo, &bundle, &localErr))
				{
					RunLogger::warn(std::string("[MeshBackendData] 3DXML skip rep: ") + localErr);
					continue;
				}
				product.reps.push_back(std::move(bundle));
			}
			if (!product.reps.empty())
			{
				if (product.matrices.size() > product.reps.size())
				{
					product.matrices.resize(product.reps.size());
				}
				while (product.matrices.size() < product.reps.size())
				{
					product.matrices.push_back(identityMatrix12());
				}
				products->push_back(std::move(product));
			}
		}
	}
	return !products->empty();
}

bool productsToHierarchy(const std::vector<DxmlProduct>& products, const std::string& rootName,
						 std::vector<MeshHierarchyPart>& outParts)
{
	outParts.clear();
	int partIndex = 0;
	for (const auto& product : products)
	{
		std::vector<float> soup;
		const std::size_t n = product.reps.size();
		for (std::size_t i = 0; i < n; ++i)
		{
			const std::vector<float>& m12 = (i < product.matrices.size()) ? product.matrices[i] : identityMatrix12();
			tessellateRep(product.reps[i], m12, soup);
		}
		if (soup.empty())
		{
			continue;
		}
		MeshHierarchyPart part;
		part.partPath = "3dxml_part_" + std::to_string(partIndex++);
		part.parentPartPath.clear();
		part.displayName = product.name.empty() ? rootName : product.name;
		part.triangleSoup = std::move(soup);
		outParts.push_back(std::move(part));
	}
	return !outParts.empty();
}

bool load3dxmlHierarchyImpl(const std::string& path, std::vector<MeshHierarchyPart>& outParts, std::string* errMsg)
{
	outParts.clear();
	DxmlZipArchive zip;
	if (!zip.open(path, errMsg))
	{
		return false;
	}
	std::vector<DxmlProduct> products;
	std::string rootName;
	bool isAssembly = false;
	if (!buildProductsFromStructure(zip, &products, &rootName, &isAssembly, errMsg))
	{
		return false;
	}
	(void)isAssembly;
	if (!productsToHierarchy(products, rootName, outParts))
	{
		meshLoadErr(errMsg, "3DXML produced empty triangle soup.");
		return false;
	}
	RunLogger::info("[MeshBackendData] 3DXML hierarchy loaded: " + std::to_string(outParts.size()) + " part(s).");
	return true;
}

} // namespace

bool meshLoad3dxmlHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts,
									std::string* errMsg)
{
	return load3dxmlHierarchyImpl(path, outParts, errMsg);
}

bool meshLoad3dxmlSingleFile(const std::string& path, std::vector<float>& soup, std::string* errMsg)
{
	soup.clear();
	std::vector<MeshHierarchyPart> parts;
	if (!meshLoad3dxmlHierarchyFromFile(path, parts, errMsg) || parts.empty())
	{
		return false;
	}
	for (auto& p : parts)
	{
		soup.insert(soup.end(), p.triangleSoup.begin(), p.triangleSoup.end());
	}
	return !soup.empty();
}

} // namespace mesh_backend_load

bool MeshBackendData::load3dxmlHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts,
												 std::string* errMsg)
{
	return mesh_backend_load::meshLoad3dxmlHierarchyFromFile(path, outParts, errMsg);
}
