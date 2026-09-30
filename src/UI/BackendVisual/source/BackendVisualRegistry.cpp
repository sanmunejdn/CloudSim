/// @file BackendVisualRegistry.cpp
/// @brief BackendVisual 注册表进程级 override 槽（跨 TU 单点）

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendTypeIdentity.h"
#include "BackendVisualRegistry.h"
#include "BrepBackendVisual.h"
#include "CustomDeviceBackendVisual.h"
#include "FrameBackendVisual.h"
#include "MeshBackendData.h"
#include "MeshBackendVisual.h"
#include "PointCloudBackendData.h"
#include "PointCloudBackendVisual.h"

#include <cassert>

BackendVisualRegistry*& BackendVisualRegistry::processInstanceSlot()
{
	static BackendVisualRegistry* slot = nullptr;
	return slot;
}

BackendVisualRegistry& BackendVisualRegistry::instance()
{
	BackendVisualRegistry* slot = processInstanceSlot();
	assert(slot != nullptr);
	return *slot;
}

void BackendVisualRegistry::registerTypeImpl(const std::string& className, Factory factory)
{
	m_factories[className] = std::move(factory);
}

void BackendVisualRegistry::registerBuiltins()
{
	registerTypeImpl(
		backend_type::kClassPointCloud,
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<PointCloudBackendVisual>(); });
	registerTypeImpl(
		backend_type::kClassModel,
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<MeshBackendVisual>(); });
	registerTypeImpl(
		backend_type::kClassModelVisualAlias,
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<MeshBackendVisual>(); });
	registerTypeImpl(
		backend_type::kClassBrepModel,
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<BrepBackendVisual>(); });
	registerTypeImpl(
		backend_type::kClassParametricBrep,
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<BrepBackendVisual>(); });
	registerTypeImpl(
		backend_type::kClassFrame,
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<FrameBackendVisual>(); });
	registerTypeImpl(
		backend_type::kClassCustomDevice,
		[]() -> std::unique_ptr<IBackendVisual> { return std::make_unique<CustomDeviceBackendVisual>(); });
}

void BackendVisualRegistry::ensureBuiltinsRegisteredImpl()
{
	std::call_once(m_builtinsOnce, [this]() { registerBuiltins(); });
}

std::unique_ptr<IBackendVisual> BackendVisualRegistry::createForClassNameImpl(const std::string& className) const
{
	const_cast<BackendVisualRegistry*>(this)->ensureBuiltinsRegisteredImpl();
	const auto it = m_factories.find(className);
	if (it == m_factories.end())
	{
		return nullptr;
	}
	return it->second();
}

bool BackendVisualRegistry::buildOuterBranchImpl(const BackendDataBase& data, const MeshVisualOptions& meshOptions,
												 BranchBuildResult& out, std::string* errorMessage) const
{
	std::unique_ptr<IBackendVisual> v = createForClassNameImpl(data.className());
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

void BackendVisualRegistry::computeModelCenterAndDiagonalImpl(const BackendDataBase& data, osg::Vec3f& outCenter,
															  float& outDiagonal) const
{
	std::unique_ptr<IBackendVisual> v = createForClassNameImpl(data.className());
	if (!v)
	{
		outCenter = osg::Vec3f(0.0f, 0.0f, 0.0f);
		outDiagonal = 1.0f;
		return;
	}
	v->computeModelCenterAndDiagonal(data, outCenter, outDiagonal);
}

void BackendVisualRegistry::registerType(const std::string& className, Factory factory)
{
	instance().registerTypeImpl(className, std::move(factory));
}

void BackendVisualRegistry::ensureBuiltinsRegistered()
{
	instance().ensureBuiltinsRegisteredImpl();
}

std::unique_ptr<IBackendVisual> BackendVisualRegistry::createForClassName(const std::string& className)
{
	return instance().createForClassNameImpl(className);
}

bool BackendVisualRegistry::buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions,
											 BranchBuildResult& out, std::string* errorMessage)
{
	return instance().buildOuterBranchImpl(data, meshOptions, out, errorMessage);
}

void BackendVisualRegistry::computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
														  float& outDiagonal)
{
	instance().computeModelCenterAndDiagonalImpl(data, outCenter, outDiagonal);
}

osg::ref_ptr<osg::Geode> BackendVisualRegistry::buildPointCloudGeode(const PointCloudBackendData& data,
																	 std::string* errorMessage)
{
	PointCloudBackendVisual v;
	return v.makeStagingGeode(data, errorMessage);
}

osg::ref_ptr<osg::Node> BackendVisualRegistry::buildMeshDisplayNode(const MeshBackendData& data,
																	const MeshVisualOptions& options,
																	std::string* errorMessage)
{
	MeshBackendVisual v;
	return v.makeDisplayNode(data, options, errorMessage);
}
