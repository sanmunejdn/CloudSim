#ifndef CLOUDSIMHOST_ASSEMBLYMATEAPPLY_H
#define CLOUDSIMHOST_ASSEMBLYMATEAPPLY_H

/// @file AssemblyMateApply.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 装配一次定位：整件面索引、世界系查询、worldMatrix 左乘

#include "cloudsim_host_global.h"

#include "AssemblyMate.h"
#include "BackendFollowMath.h"

#include <QString>
#include <string>

namespace cloudsim::host
{
class DocumentHost;

struct AssemblyMateFaceRef
{
	std::string backendId;
	int faceIndex = -1;
	double pickWorldMm[3]{0.0, 0.0, 0.0};
};

/// 记录点中的整件 B-rep 与面索引，不抽 Solid
CLOUDSIM_HOST_EXPORT bool resolveAssemblyMatePick(DocumentHost& host, const std::string& brepId, int faceIndex,
												  double pickWorldX, double pickWorldY, double pickWorldZ,
												  AssemblyMateFaceRef& out, QString* outError = nullptr);

CLOUDSIM_HOST_EXPORT bool snapshotBackendWorldMatrix(DocumentHost& host, const std::string& backendId,
													 BackendMat4& outWorld);

CLOUDSIM_HOST_EXPORT bool restoreBackendWorldMatrix(DocumentHost& host, const std::string& backendId,
													const BackendMat4& world, QString* outError = nullptr);

/// 从快照左乘增量并同步 OSG；commit 时发 PoseCommitted 并跟 Follow
CLOUDSIM_HOST_EXPORT bool applyAssemblyMate(DocumentHost& host, const AssemblyMateFaceRef& grounded,
											const AssemblyMateFaceRef& moving, const geoalgo::AssemblyMateParams& params,
											const BackendMat4* movingWorldSnapshot, bool commit,
											QString* outError = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_ASSEMBLYMATEAPPLY_H
