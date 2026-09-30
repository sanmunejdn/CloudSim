#ifndef CLOUDSIMHOST_MESHCAPTUREDPART_H
#define CLOUDSIMHOST_MESHCAPTUREDPART_H

/// @file MeshCapturedPart.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 层级 mesh 捕获部件（供 IViewportSceneOps / 导入路径共享，不依赖 Controller）

#include <QString>
#include <vector>

struct MeshCapturedPart
{
	QString partPath;
	QString parentPartPath;
	QString displayName;
	std::vector<float> triangleSoup;
};

#endif // CLOUDSIMHOST_MESHCAPTUREDPART_H
