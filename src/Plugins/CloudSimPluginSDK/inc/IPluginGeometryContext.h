#ifndef CLOUDSIMPLUGINSDK_IPLUGINGEOMETRYCONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINGEOMETRYCONTEXT_H

/// @file IPluginGeometryContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 几何窄接口（IPluginHostContext 按域拆分）

/// vtable 仅末尾追加，勿插入中间以免破坏旧插件 ABI

#include "cloudsim_plugin_sdk_global.h"

#include "PluginBackendMeta.h"
#include "PluginPrimitiveTypes.h"

#include <QByteArray>
#include <QString>
#include <string>
#include <vector>

class IPluginGeometryHost;

/// 几何上下文：图元/soup 建模、布尔、几何算法宿主、视口截图
class IPluginGeometryContext
{
public:
	virtual ~IPluginGeometryContext() = default;

	/// 接口版本（0x00010000 = 1.0.0）
	virtual unsigned int contextVersion() const = 0;

	/// 宿主建 MeshBackendData+OSG（同 AI create-mesh）；outBackendId 可选返回注册 id
	virtual bool createPrimitiveMesh(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
									 const PluginMeshCreateOptions& options, QString* outError,
									 QString* outBackendId = nullptr) = 0;

	/// 向 BackendRegistry 注册插件后端类型
	virtual bool registerBackendType(const PluginBackendMeta& meta, QString* outError) = 0;

	/// 三角 soup 注册 mesh（每三角 9 float，mm）
	virtual bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
									  QString* outError) = 0;

	/// 两网格布尔（世界坐标 soup）；outResultBackendId 为结果 mesh id
	virtual bool booleanMesh(PluginMeshBooleanOp op, const std::string& targetBackendId,
							 const std::string& toolBackendId, const PluginBooleanMeshOptions& options,
							 std::string* outResultBackendId, QString* outError) = 0;

	virtual bool buildPrimitiveMeshSoup(const PluginPrimitiveMeshParams& params,
										const PluginPrimitiveMeshQuality& quality,
										const PluginMeshCreateOptions& placement, std::vector<float>& outWorldSoup,
										QString* outError) = 0;

	virtual bool booleanMeshSoups(PluginMeshBooleanOp op, const std::vector<float>& targetWorldSoup,
								  const std::vector<float>& toolWorldSoup, const PluginBooleanMeshOptions& options,
								  std::string* outResultBackendId, QString* outError) = 0;

	virtual bool booleanPrimitiveMeshes(PluginMeshBooleanOp op, const PluginPrimitiveMeshParams& targetParams,
										const PluginPrimitiveMeshQuality& targetQuality,
										const PluginMeshCreateOptions& targetPlacement,
										const PluginPrimitiveMeshParams& toolParams,
										const PluginPrimitiveMeshQuality& toolQuality,
										const PluginMeshCreateOptions& toolPlacement,
										const PluginBooleanMeshOptions& options, std::string* outResultBackendId,
										QString* outError) = 0;

	/// OCC/CGAL 几何算法宿主；宿主版本不足时可为 null
	virtual IPluginGeometryHost* geometryHost() = 0;
	virtual const IPluginGeometryHost* geometryHost() const = 0;

	/// 活动文档 3D 视口 PNG 截图（供 geometry.recognize 等多模态域）
	virtual bool captureActiveViewportPng(QByteArray& outPng, QString* outError = nullptr) = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINGEOMETRYCONTEXT_H
