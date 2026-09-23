/// @file GeometryImportersBuiltins.cpp
/// @brief 内建几何文件 Importer（后缀 → parse）

#include "GeometryFileImporterRegistry.h"

#include "BackendImporters.h"
#include "BrepBackendData.h"
#include "MeshBackendData.h"

#include <filesystem>
#include <memory>

namespace cloudsim::host
{
namespace
{
std::string fileStemUtf8Hint(const std::string& nativePath)
{
	try
	{
		return std::filesystem::path(nativePath).filename().string();
	}
	catch (...)
	{
		return {};
	}
}

class DxfGeometryImporter final : public IGeometryFileImporter
{
public:
	std::vector<std::string> extensions() const override { return {"dxf"}; }

	bool parse(const std::string& nativePath, const ImportParseOptions& /*opt*/, ImportParseResult& out,
			   std::string* errMsg) const override
	{
		out = {};
		out.kind = ImportParseKind::MeshHierarchy;
		out.displayNameHint = fileStemUtf8Hint(nativePath);
		if (!MeshBackendData::loadDxfHierarchyFromFile(nativePath, out.meshParts, errMsg) || out.meshParts.empty())
		{
			return false;
		}
		out.ok = true;
		return true;
	}
};

class DxmlGeometryImporter final : public IGeometryFileImporter
{
public:
	std::vector<std::string> extensions() const override { return {"3dxml"}; }

	bool parse(const std::string& nativePath, const ImportParseOptions& /*opt*/, ImportParseResult& out,
			   std::string* errMsg) const override
	{
		out = {};
		out.kind = ImportParseKind::MeshHierarchy;
		out.displayNameHint = fileStemUtf8Hint(nativePath);
		if (!MeshBackendData::load3dxmlHierarchyFromFile(nativePath, out.meshParts, errMsg) || out.meshParts.empty())
		{
			return false;
		}
		out.ok = true;
		return true;
	}
};

class StepGeometryImporter final : public IGeometryFileImporter
{
public:
	std::vector<std::string> extensions() const override { return {"step", "stp"}; }

	bool parse(const std::string& nativePath, const ImportParseOptions& /*opt*/, ImportParseResult& out,
			   std::string* errMsg) const override
	{
		out = {};
		out.displayNameHint = fileStemUtf8Hint(nativePath);
		std::vector<BrepHierarchyPart> parts;
		geoalgo::ShapeHandle assembly;
		if (!BrepBackendData::loadStepHierarchyFromFile(nativePath, parts, errMsg, &assembly))
		{
			return false;
		}
		if (parts.size() > 1U)
		{
			out.kind = ImportParseKind::BrepHierarchy;
			out.brepParts = std::move(parts);
			out.brepAssembly = assembly;
			out.ok = true;
			return true;
		}
		out.kind = ImportParseKind::BrepSingle;
		if (parts.size() == 1U && !parts.front().shapeRef.isNull())
		{
			out.brepSingle = parts.front().shapeRef;
		}
		else if (!assembly.isNull())
		{
			out.brepSingle = assembly;
		}
		else
		{
			BrepBackendData tmp;
			if (!tmp.loadFromStepFile(nativePath, errMsg) || tmp.shapeRef().isNull())
			{
				return false;
			}
			out.brepSingle = tmp.shapeRef();
		}
		out.ok = !out.brepSingle.isNull();
		return out.ok;
	}
};

class BrepFileGeometryImporter final : public IGeometryFileImporter
{
public:
	std::vector<std::string> extensions() const override { return {"brep"}; }

	bool parse(const std::string& nativePath, const ImportParseOptions& /*opt*/, ImportParseResult& out,
			   std::string* errMsg) const override
	{
		out = {};
		out.kind = ImportParseKind::BrepSingle;
		out.displayNameHint = fileStemUtf8Hint(nativePath);
		BrepBackendData tmp;
		if (!tmp.loadFromBrepFile(nativePath, errMsg) || tmp.shapeRef().isNull())
		{
			return false;
		}
		out.brepSingle = tmp.shapeRef();
		out.ok = true;
		return true;
	}
};

class CgalMeshGeometryImporter final : public IGeometryFileImporter
{
public:
	std::vector<std::string> extensions() const override
	{
		return {"obj", "stl", "ply", "off", "igs", "iges"};
	}

	bool parse(const std::string& nativePath, const ImportParseOptions& opt, ImportParseResult& out,
			   std::string* errMsg) const override
	{
		out = {};
		out.kind = ImportParseKind::MeshSingleSoup;
		out.displayNameHint = fileStemUtf8Hint(nativePath);
		MeshBackendData tmp;
		if (!backend_io::loadMeshFromFile(tmp, nativePath, errMsg, opt.meshImportQuality) ||
			tmp.triangleSoup().empty())
		{
			return false;
		}
		out.meshSoup = tmp.triangleSoup();
		out.ok = true;
		return true;
	}
};

class OsgGeometryImporter final : public IGeometryFileImporter
{
public:
	std::vector<std::string> extensions() const override { return {"dae", "3ds", "fbx"}; }

	bool parse(const std::string& /*nativePath*/, const ImportParseOptions& /*opt*/, ImportParseResult& out,
			   std::string* /*errMsg*/) const override
	{
		out = {};
		out.kind = ImportParseKind::OsgCapture;
		out.ok = true;
		return true;
	}
};

} // namespace

void registerBuiltinGeometryImporters(GeometryFileImporterRegistry& registry)
{
	registry.add(std::make_unique<DxfGeometryImporter>());
	registry.add(std::make_unique<DxmlGeometryImporter>());
	registry.add(std::make_unique<StepGeometryImporter>());
	registry.add(std::make_unique<BrepFileGeometryImporter>());
	registry.add(std::make_unique<CgalMeshGeometryImporter>());
	registry.add(std::make_unique<OsgGeometryImporter>());
}

} // namespace cloudsim::host
