/// @file HostBackendRegistries.cpp
/// @brief 宿主侧 BackendRegistry / CodecRegistry 访问

#include "HostBackendRegistries.h"

#include "BackendComponentCodecRegistry.h"
#include "BackendRegistry.h"
#include "CloudSimBootstrap.h"
#include "ICloudSimContext.h"

#include <cassert>
#include <memory>

namespace cloudsim::host
{
BackendRegistry& backendRegistry()
{
	cloudsim::core::ICloudSimContext* ctx = cloudsimApplicationContext();
	assert(ctx != nullptr);
	std::shared_ptr<BackendRegistry> svc = ctx->services().getService<BackendRegistry>();
	assert(svc != nullptr);
	return *svc;
}

BackendComponentCodecRegistry& backendComponentCodecRegistry()
{
	cloudsim::core::ICloudSimContext* ctx = cloudsimApplicationContext();
	assert(ctx != nullptr);
	std::shared_ptr<BackendComponentCodecRegistry> svc = ctx->services().getService<BackendComponentCodecRegistry>();
	assert(svc != nullptr);
	return *svc;
}

} // namespace cloudsim::host
