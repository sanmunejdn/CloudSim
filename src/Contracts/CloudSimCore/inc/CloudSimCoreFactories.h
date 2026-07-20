#ifndef CLOUDSIMCORE_CLOUDSIMCOREFACTORIES_H
#define CLOUDSIMCORE_CLOUDSIMCOREFACTORIES_H

/// @file CloudSimCoreFactories.h
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
}

#endif // CLOUDSIMCORE_CLOUDSIMCOREFACTORIES_H
