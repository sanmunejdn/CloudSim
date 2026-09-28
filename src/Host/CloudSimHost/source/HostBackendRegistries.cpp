/// @file HostBackendRegistries.cpp
/// @brief 宿主侧 BackendRegistry / CodecRegistry 访问

#include "HostBackendRegistries.h"

#include "BackendComponentCodecRegistry.h"
#include "BackendRegistry.h"
#include "CloudSimBootstrap.h"
#include "ICloudSimContext.h"

#include <memory>

namespace cloudsim::host
{
namespace
{
template <typename T>
T& registryFromContextOrFallback(T& (*fallback)())
{
	if (cloudsim::core::ICloudSimContext* ctx = cloudsimApplicationContext())
	{
		if (std::shared_ptr<T> svc = ctx->services().getService<T>())
		{
			return *svc;
		}
	}
	return fallback();
}
} // namespace

BackendRegistry& backendRegistry()
{
	return registryFromContextOrFallback(&BackendRegistry::instance);
}

BackendComponentCodecRegistry& backendComponentCodecRegistry()
{
	return registryFromContextOrFallback(&BackendComponentCodecRegistry::instance);
}

} // namespace cloudsim::host
