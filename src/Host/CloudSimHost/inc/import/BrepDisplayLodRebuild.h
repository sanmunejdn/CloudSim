#ifndef CLOUDSIMHOST_BREPDISPLAYLODREBUILD_H
#define CLOUDSIMHOST_BREPDISPLAYLODREBUILD_H

/// @file BrepDisplayLodRebuild.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 按档重剖 B-rep 显示网格并刷新场景

#include "cloudsim_host_global.h"

#include <QString>
#include <string>

namespace cloudsim::host
{
class DocumentHost;

/**
 * 写入 displayLod、清 artifacts 缓存、按档重剖 Phase1，并 Geometry 刷新视口
 * @param lod 0=Coarse，1=Medium，2=Fine
 */
CLOUDSIM_HOST_EXPORT bool rebuildBrepDisplayLod(DocumentHost& host, const std::string& backendId, int lod,
												QString* outError = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BREPDISPLAYLODREBUILD_H
