#ifndef CLOUDSIMPLUGINHOST_PLUGINGEOMETRYHOSTIMPL_H
#define CLOUDSIMPLUGINHOST_PLUGINGEOMETRYHOSTIMPL_H

/// @file PluginGeometryHostImpl.h
/// @brief PluginGeometryHostImpl 接口

#include "IPluginGeometryHost.h"

class PluginHostContext;

class PluginGeometryHostImpl : public IPluginGeometryHost
{
public:
	explicit PluginGeometryHostImpl(PluginHostContext* hostContext);

	void discretizeStepToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
							  const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
							  PluginGeometryFinishedFn onFinished) override;

	void discretizeBackendToMesh(IPluginDocument* doc, const std::string& stepPathUtf8,
								 const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
								 PluginGeometryFinishedFn onFinished) override;

	void discretizeBackendFaceToMesh(IPluginDocument* doc, const PluginGeometryStepRef& faceRef,
									 const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
									 PluginGeometryFinishedFn onFinished) override;

	void discretizeWireToTubeMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
								  const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
								  PluginGeometryFinishedFn onFinished) override;

	void discretizeWireToRibbonMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
									const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
									PluginGeometryFinishedFn onFinished) override;

	void discretizePolylineToMesh(IPluginDocument* doc, const std::vector<float>& polylineXyz,
								  const PluginMeshDiscretizeParams& params, const PluginMeshCreateOptions& options,
								  PluginGeometryFinishedFn onFinished) override;

	void discretizeBackendEdgesToPolylines(IPluginDocument* doc, const std::string& stepPathUtf8,
										   const PluginMeshDiscretizeParams& params,
										   PluginGeometryFinishedFn onFinished) override;

	void intersectEdges(IPluginDocument* doc, const PluginGeometryStepRef& edge1, const PluginGeometryStepRef& edge2,
						const PluginGeometryIntersectionParams& params, PluginGeometryFinishedFn onFinished) override;

	void intersectEdgeFace(IPluginDocument* doc, const PluginGeometryStepRef& edgeRef,
						   const PluginGeometryStepRef& faceRef, const PluginGeometryIntersectionParams& params,
						   PluginGeometryFinishedFn onFinished) override;

	void intersectFaces(IPluginDocument* doc, const PluginGeometryStepRef& face1, const PluginGeometryStepRef& face2,
						const PluginGeometryIntersectionParams& params, PluginGeometryFinishedFn onFinished) override;

	void intersectBackends(IPluginDocument* doc, const std::string& targetStepPathUtf8,
						   const std::string& toolStepPathUtf8, const PluginGeometryIntersectionParams& params,
						   PluginGeometryFinishedFn onFinished) override;

	void brepBooleanToMesh(IPluginDocument* doc, const std::string& targetStepPathUtf8,
						   const std::string& toolStepPathUtf8, const PluginGeometryBrepBooleanParams& params,
						   PluginGeometryFinishedFn onFinished) override;

	void fuseWiresToPolyline(IPluginDocument* doc, const std::string& stepPathUtf8, const std::vector<int>& edgeIndices,
							 const PluginGeometryIntersectionParams& params,
							 PluginGeometryFinishedFn onFinished) override;

	void sewFacesToMesh(IPluginDocument* doc, const std::string& stepPathUtf8, const std::vector<int>& faceIndices,
						const PluginMeshDiscretizeParams& meshParams, const PluginMeshCreateOptions& options,
						PluginGeometryFinishedFn onFinished) override;

	bool listComputableBackends(IPluginDocument* doc, std::vector<PluginGeometryBackendEntry>& outBackends,
								QString* outError = nullptr) override;

	void pickStepElementFromViewport(IPluginDocument* doc, const PluginGeometryElementPickRequest& request,
									 PluginGeometryElementPickedFn onFinished) override;

private:
	PluginHostContext* m_host = nullptr;
};

#endif // CLOUDSIMPLUGINHOST_PLUGINGEOMETRYHOSTIMPL_H
