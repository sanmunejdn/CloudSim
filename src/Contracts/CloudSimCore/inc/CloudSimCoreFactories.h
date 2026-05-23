#pragma once

namespace cloudsim::core {

class IDataService;
class IRobotService;
class IRenderViewFactory;

} // namespace cloudsim::core

/// Backend DLL entry points (implemented in Data / RobotScene / OsgWidgetCore).
extern "C" {
cloudsim::core::IDataService* cloudsimCreateDataService(unsigned int apiVersion);
cloudsim::core::IRobotService* cloudsimCreateRobotService(unsigned int apiVersion);
cloudsim::core::IRenderViewFactory* cloudsimCreateRenderViewFactory(unsigned int apiVersion);
}
