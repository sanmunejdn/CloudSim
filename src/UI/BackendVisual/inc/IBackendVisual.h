#ifndef BACKENDVISUAL_IBACKENDVISUAL_H
#define BACKENDVISUAL_IBACKENDVISUAL_H

/// @file IBackendVisual.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 仅网格构建使用，点云忽略

#include "backendvisual_global.h"

#include <memory>
#include <string>

#include <osg/MatrixTransform>
#include <osg/Vec3f>
#include <osg/ref_ptr>

class BackendDataBase;

namespace geoalgo
{
struct BrepImportArtifacts;
}

/// 仅网格构建使用，点云忽略
struct MeshVisualOptions
{
	bool showWireOutline = true;
	bool useSceneLighting = false;
};

struct BranchBuildResult
{
	/// 外层 MT 存完整局部刚体矩阵，避免 PAT TRS 分解损失
	osg::ref_ptr<osg::MatrixTransform> outer;
	osg::Vec3f modelCenter{};
	float diagonal = 1.0f;
	std::shared_ptr<geoalgo::BrepImportArtifacts> brepArtifacts;
};

/// 按后端类型构建场景分支并提取 gizmo/拾取度量
class BACKENDVISUAL_EXPORT IBackendVisual
{
public:
	virtual ~IBackendVisual() = default;

	virtual std::string typeKey() const = 0;

	virtual bool buildOuterBranch(const BackendDataBase& data, const MeshVisualOptions& meshOptions,
								  BranchBuildResult& out, std::string* errorMessage) = 0;

	virtual void computeModelCenterAndDiagonal(const BackendDataBase& data, osg::Vec3f& outCenter,
											   float& outDiagonal) const = 0;
};

#endif // BACKENDVISUAL_IBACKENDVISUAL_H
