#ifndef CLOUDSIMHOST_BACKENDFILEIMPORT_H
#define CLOUDSIMHOST_BACKENDFILEIMPORT_H

/// @file BackendFileImport.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 单网格文件导入

#include "cloudsim_host_global.h"

#include "CoreTypes.h"

#include <memory>

class BackendDataBase;
class BrepBackendData;
class CustomDeviceBackendData;
class FrameBackendData;
class MeshBackendData;
class PointCloudBackendData;

namespace cloudsim::host
{
class DocumentHost;

/// 单网格文件导入
CLOUDSIM_HOST_EXPORT core::ObjectId importMeshFile(DocumentHost& host, const QString& filePath,
												   const core::ImportOptionsDto& options, QString* outError = nullptr);

/// 点云文件导入
CLOUDSIM_HOST_EXPORT core::ObjectId importPointCloudFile(DocumentHost& host, const QString& filePath,
														 const core::ImportOptionsDto& options,
														 QString* outError = nullptr);

/// 接管已构造 Backend
/// @param linkOsgSceneParent false 时 OSG 扁平（DXF 世界坐标分件）
CLOUDSIM_HOST_EXPORT bool registerAdoptedBackendObject(DocumentHost& host,
													   const std::shared_ptr<BackendDataBase>& object,
													   const QString& sourcePath, const QString& catalogTypeName,
													   const QString& parentId, QString* outError = nullptr,
													   bool linkOsgSceneParent = true);

/// 注册 mesh 并加载
CLOUDSIM_HOST_EXPORT bool registerAdoptedMeshAndLoadScene(DocumentHost& host,
														  const std::shared_ptr<MeshBackendData>& mesh,
														  const QString& sourcePath, const QString& catalogTypeName,
														  const QString& parentId, bool resetViewToHome,
														  QString* outError = nullptr, bool linkOsgSceneParent = true);

CLOUDSIM_HOST_EXPORT bool
registerAdoptedBrepAndLoadScene(DocumentHost& host, const std::shared_ptr<BrepBackendData>& brep,
								const QString& sourcePath, const QString& catalogTypeName, const QString& parentId,
								const bool resetViewToHome, QString* outError = nullptr,
								const bool linkOsgSceneParent = true, const bool loadScene = true);

/// 注册坐标系并加载场景轴
CLOUDSIM_HOST_EXPORT bool registerAdoptedFrameAndLoadScene(DocumentHost& host,
														   const std::shared_ptr<FrameBackendData>& frame,
														   const QString& catalogTypeName, const QString& parentId,
														   bool resetViewToHome, QString* outError = nullptr);

/// 注册自定义设备根并加载示意轴
CLOUDSIM_HOST_EXPORT bool
registerAdoptedCustomDeviceAndLoadScene(DocumentHost& host, const std::shared_ptr<CustomDeviceBackendData>& device,
										const QString& catalogTypeName, const QString& parentId, bool resetViewToHome,
										QString* outError = nullptr);

/// 将子后端挂到自定义设备（Data 父边 + OSG + Follow）
CLOUDSIM_HOST_EXPORT bool attachBackendChildToCustomDevice(DocumentHost& host, const std::string& deviceId,
														   const std::string& childId, QString* outError = nullptr);

/// 将子后端挂到任意父后端（Data 单父 + OSG + hierarchy Follow）
CLOUDSIM_HOST_EXPORT bool attachBackendChildToParent(DocumentHost& host, const std::string& parentId,
													 const std::string& childId, QString* outError = nullptr);

/// 导出自定义设备为 ROS 包（package.xml + urdf + meshes/cad）；成功时 outUrdfPath 为 .urdf 绝对路径
CLOUDSIM_HOST_EXPORT bool exportCustomDeviceUrdfPackage(DocumentHost& host, const std::string& deviceId,
														const QString& packageParentDir, QString* outUrdfPath = nullptr,
														QString* outPackageRoot = nullptr, QString* outError = nullptr);

/// 注册点云并加载
CLOUDSIM_HOST_EXPORT bool
registerAdoptedPointCloudAndLoadScene(DocumentHost& host, const std::shared_ptr<PointCloudBackendData>& pointCloud,
									  const QString& sourcePath, const QString& catalogTypeName, bool resetViewToHome,
									  QString* outError = nullptr);

/// 工程 id 重注册
CLOUDSIM_HOST_EXPORT QString rekeyBackendObject(DocumentHost& host, const QString& fromId, const QString& toId,
												QString* outError = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDFILEIMPORT_H
