#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginGeometryTypes.h"

class IPluginDocument;

/// 几何算法宿主 API（1.5.0+）；插件经 IPluginHostContext::geometryHost() 获取
class IPluginGeometryHost
{
public:
	virtual ~IPluginGeometryHost() = default;

	virtual void discretizeStepToMesh(
		IPluginDocument* doc,
		const std::string& stepPathUtf8,
		const PluginMeshDiscretizeParams& params,
		const PluginMeshCreateOptions& options,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeBackendToMesh(
		IPluginDocument* doc,
		const std::string& stepPathUtf8,
		const PluginMeshDiscretizeParams& params,
		const PluginMeshCreateOptions& options,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeBackendFaceToMesh(
		IPluginDocument* doc,
		const PluginGeometryStepRef& faceRef,
		const PluginMeshDiscretizeParams& params,
		const PluginMeshCreateOptions& options,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeWireToTubeMesh(
		IPluginDocument* doc,
		const std::vector<float>& polylineXyz,
		const PluginMeshDiscretizeParams& params,
		const PluginMeshCreateOptions& options,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeWireToRibbonMesh(
		IPluginDocument* doc,
		const std::vector<float>& polylineXyz,
		const PluginMeshDiscretizeParams& params,
		const PluginMeshCreateOptions& options,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizePolylineToMesh(
		IPluginDocument* doc,
		const std::vector<float>& polylineXyz,
		const PluginMeshDiscretizeParams& params,
		const PluginMeshCreateOptions& options,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void discretizeBackendEdgesToPolylines(
		IPluginDocument* doc,
		const std::string& stepPathUtf8,
		const PluginMeshDiscretizeParams& params,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void intersectEdges(
		IPluginDocument* doc,
		const PluginGeometryStepRef& edge1,
		const PluginGeometryStepRef& edge2,
		const PluginGeometryIntersectionParams& params,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void intersectEdgeFace(
		IPluginDocument* doc,
		const PluginGeometryStepRef& edgeRef,
		const PluginGeometryStepRef& faceRef,
		const PluginGeometryIntersectionParams& params,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void intersectFaces(
		IPluginDocument* doc,
		const PluginGeometryStepRef& face1,
		const PluginGeometryStepRef& face2,
		const PluginGeometryIntersectionParams& params,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void intersectBackends(
		IPluginDocument* doc,
		const std::string& targetStepPathUtf8,
		const std::string& toolStepPathUtf8,
		const PluginGeometryIntersectionParams& params,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void brepBooleanToMesh(
		IPluginDocument* doc,
		const std::string& targetStepPathUtf8,
		const std::string& toolStepPathUtf8,
		const PluginGeometryBrepBooleanParams& params,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void fuseWiresToPolyline(
		IPluginDocument* doc,
		const std::string& stepPathUtf8,
		const std::vector<int>& edgeIndices,
		const PluginGeometryIntersectionParams& params,
		PluginGeometryFinishedFn onFinished) = 0;

	virtual void sewFacesToMesh(
		IPluginDocument* doc,
		const std::string& stepPathUtf8,
		const std::vector<int>& faceIndices,
		const PluginMeshDiscretizeParams& meshParams,
		const PluginMeshCreateOptions& options,
		PluginGeometryFinishedFn onFinished) = 0;

	/// 枚举活动文档中可参与 STEP/BRep 运算的后端对象
	virtual bool listComputableBackends(
		IPluginDocument* doc,
		std::vector<PluginGeometryBackendEntry>& outBackends,
		QString* outError = nullptr) = 0;

	/// 进入一次视图拾取并返回 STEP edge/face 引用
	virtual void pickStepElementFromViewport(
		IPluginDocument* doc,
		const PluginGeometryElementPickRequest& request,
		PluginGeometryElementPickedFn onFinished) = 0;
};
