/// @file GeometryFileImporterRegistry.cpp
/// @brief 几何文件导入器后缀注册表

#include "GeometryFileImporterRegistry.h"

#include <cctype>
#include <memory>
#include <unordered_map>
#include <vector>

namespace cloudsim::host
{
namespace
{
std::string lowerExt(std::string ext)
{
	for (char& c : ext)
	{
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}
	return ext;
}
} // namespace

struct GeometryFileImporterRegistry::Impl
{
	std::vector<std::unique_ptr<IGeometryFileImporter>> owners;
	std::unordered_map<std::string, const IGeometryFileImporter*> byExt;
	bool builtinsRegistered = false;
};

GeometryFileImporterRegistry& GeometryFileImporterRegistry::instance()
{
	static GeometryFileImporterRegistry s;
	return s;
}

GeometryFileImporterRegistry::~GeometryFileImporterRegistry()
{
	delete m_impl;
	m_impl = nullptr;
}

void GeometryFileImporterRegistry::add(std::unique_ptr<IGeometryFileImporter> importer)
{
	if (!m_impl)
	{
		m_impl = new Impl();
	}
	if (!importer)
	{
		return;
	}
	const IGeometryFileImporter* raw = importer.get();
	for (const std::string& ext : raw->extensions())
	{
		m_impl->byExt[lowerExt(ext)] = raw;
	}
	m_impl->owners.push_back(std::move(importer));
}

const IGeometryFileImporter* GeometryFileImporterRegistry::find(const std::string& extensionLower) const
{
	const_cast<GeometryFileImporterRegistry*>(this)->ensureBuiltinsRegistered();
	if (!m_impl)
	{
		return nullptr;
	}
	const auto it = m_impl->byExt.find(lowerExt(extensionLower));
	if (it == m_impl->byExt.end())
	{
		return nullptr;
	}
	return it->second;
}

std::vector<std::string> GeometryFileImporterRegistry::allExtensions() const
{
	const_cast<GeometryFileImporterRegistry*>(this)->ensureBuiltinsRegistered();
	std::vector<std::string> out;
	if (!m_impl)
	{
		return out;
	}
	out.reserve(m_impl->byExt.size());
	for (const auto& kv : m_impl->byExt)
	{
		out.push_back(kv.first);
	}
	return out;
}

void GeometryFileImporterRegistry::ensureBuiltinsRegistered()
{
	if (!m_impl)
	{
		m_impl = new Impl();
	}
	if (m_impl->builtinsRegistered)
	{
		return;
	}
	m_impl->builtinsRegistered = true;
	registerBuiltinGeometryImporters(*this);
}

} // namespace cloudsim::host
