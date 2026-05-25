#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendVisualRegistry.h"

#include "MeshBackendData.h"
#include "MeshBackendVisual.h"
#include "PointCloudBackendData.h"
#include "PointCloudBackendVisual.h"

#include <mutex>
#include <unordered_map>

namespace {

std::unordered_map<std::string, BackendVisualRegistry::Factory>& factories()
{
	static std::unordered_map<std::string, BackendVisualRegistry::Factory> m;
	return m;
}

std::once_flag& builtinsOnce()
{
	static std::once_flag f;
	return f;
}

void registerBuiltins()
{
	BackendVisualRegistry::registerType("PointCloudBackendData",
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<PointCloudBackendVisual>(); });
	BackendVisualRegistry::registerType("Model",
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<MeshBackendVisual>(); });
	BackendVisualRegistry::registerType("MeshBackendData",
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<MeshBackendVisual>(); });
}

} // namespace

void BackendVisualRegistry::registerType(const std::string& className, Factory factory)
{
	factories()[className] = std::move(factory);
}

void BackendVisualRegistry::ensureBuiltinsRegistered()
{
	std::call_once(builtinsOnce(), registerBuiltins);
}

std::unique_ptr<IBackendVisual> BackendVisualRegistry::createForClassName(const std::string& className)
{
	ensureBuiltinsRegistered();
	const auto it = factories().find(className);
	if (it == factories().end())
	{
		return nullptr;
	}
	return it->second();
}

bool BackendVisualRegistry::buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions,
	BranchBuildResult& out, std::string* errorMessage)
{
	std::unique_ptr<IBackendVisual> v = createForClassName(data.className());
	if (!v)
	{
		if (errorMessage)
		{
			*errorMessage = "No visual registered for backend class: " + data.className();
		}
		return false;
	}
	return v->buildOuterBranch(data, meshOptions, out, errorMessage);
}

void BackendVisualRegistry::computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
	float& outDiagonal)
{
	std::unique_ptr<IBackendVisual> v = createForClassName(data.className());
	if (!v)
	{
		outCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
		outDiagonal = 1.0f;
		return;
	}
	v->computeModelCenterAndDiagonal(data, outCenter, outDiagonal);
}

osg::ref_ptr<osg::Geode> BackendVisualRegistry::buildPointCloudGeode(const PointCloudBackendData& data,
	std::string* errorMessage)
{
	PointCloudBackendVisual v;
	return v.makeStagingGeode(data, errorMessage);
}

osg::ref_ptr<osg::Node> BackendVisualRegistry::buildMeshDisplayNode(const MeshBackendData& data,
	const MeshVisualOptions& options, std::string* errorMessage)
{
	MeshBackendVisual v;
	return v.makeDisplayNode(data, options, errorMessage);
}
