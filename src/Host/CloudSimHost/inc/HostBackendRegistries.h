#ifndef CLOUDSIMHOST_HOSTBACKENDREGISTRIES_H
#define CLOUDSIMHOST_HOSTBACKENDREGISTRIES_H

/// @file HostBackendRegistries.h
/// @brief 宿主侧取 Data 层 Registry：优先 ServiceRegistry，避免散落 cloudsimApplicationContext

#include "cloudsim_host_global.h"

class BackendComponentCodecRegistry;
class BackendRegistry;

namespace cloudsim::host
{
CLOUDSIM_HOST_EXPORT BackendRegistry& backendRegistry();
CLOUDSIM_HOST_EXPORT BackendComponentCodecRegistry& backendComponentCodecRegistry();
} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HOSTBACKENDREGISTRIES_H
