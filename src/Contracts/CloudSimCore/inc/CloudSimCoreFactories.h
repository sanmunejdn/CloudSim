#ifndef CLOUDSIMCORE_CLOUDSIMCOREFACTORIES_H
#define CLOUDSIMCORE_CLOUDSIMCOREFACTORIES_H

/// @file CloudSimCoreFactories.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 后端 DLL 工厂入口

namespace cloudsim::core
{
class IDataService;
class IRobotService;
class IRenderViewFactory;

} // namespace cloudsim::core

/// 后端 DLL 工厂入口
extern "C"
{
	cloudsim::core::IDataService* cloudsimCreateDataService(unsigned int apiVersion);
	cloudsim::core::IRobotService* cloudsimCreateRobotService(unsigned int apiVersion);
	cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion);

	/// 配套释放接口：跨 DLL delete 仅在双方共享 CRT（/MD）时安全；
	/// Host 若 /MT 编译必须走此入口释放，禁止直接 delete
	void cloudsimDestroyDataService(cloudsim::core::IDataService* svc);
	void cloudsimDestroyRobotService(cloudsim::core::IRobotService* svc);
	void cloudsimDestroyRenderViewFactory(cloudsim::core::IRenderViewFactory* factory);
}

#endif // CLOUDSIMCORE_CLOUDSIMCOREFACTORIES_H
