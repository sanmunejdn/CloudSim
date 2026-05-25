#pragma once

namespace cloudsim::core {

class IDataService;
class IRobotService;
class IRenderViewFactory;

} // namespace cloudsim::core

/// 后端 DLL 工厂入口
extern "C" {
cloudsim::core::IDataService* cloudsimCreateDataService(unsigned int apiVersion);
cloudsim::core::IRobotService* cloudsimCreateRobotService(unsigned int apiVersion);
cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion);
}
