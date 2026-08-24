#ifndef CLOUDSIMHOST_BACKENDVISUALENSURE_H
#define CLOUDSIMHOST_BACKENDVISUALENSURE_H

/// @file BackendVisualEnsure.h
/// @brief 统一后端对象 OSG 分支 ensure 入口

#include "cloudsim_host_global.h"

#include <QString>
#include <string>

namespace cloudsim::host
{
class DocumentHost;

enum class EnsureVisualPolicy
{
	CreateIfMissing,
	TransformOnly,
	FullRebuild
};

struct EnsureVisualOptions
{
	bool showWireOutline = true;
	bool useSceneLighting = true;
	bool resetViewToHome = false;
	bool urdfLinkMesh = false;
};

struct EnsureVisualResult
{
	bool ok = false;
	bool createdBranch = false;
	QString error;
};

CLOUDSIM_HOST_EXPORT EnsureVisualResult ensureVisual(DocumentHost& host, const std::string& backendId,
													 EnsureVisualPolicy policy,
													 const EnsureVisualOptions& opts = {});

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDVISUALENSURE_H
