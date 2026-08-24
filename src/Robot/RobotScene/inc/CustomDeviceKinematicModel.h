#ifndef ROBOTSCENE_CUSTOMDEVICEKINEMATICMODEL_H
#define ROBOTSCENE_CUSTOMDEVICEKINEMATICMODEL_H

#include "CustomDeviceBackendData.h"
#include "robot_scene_global.h"

#include "IKinematicModel.h"
#include "KinematicGraph.h"

#include <array>
#include <memory>
#include <unordered_map>

class BackendDataManager;
class IRobotBackendPoseSink;

namespace CustomDeviceKinematicModel
{
class ROBOT_SCENE_API Model : public kinematic_core::IKinematicModel
{
public:
	explicit Model(CustomDeviceBackendData& device);

	const kinematic_core::KinematicGraph& graph() const override { return m_graph; }
	const CustomDeviceBackendData& device() const { return m_device; }
	int dofCount() const override { return m_graph.dofCount(); }
	std::vector<kinematic_core::AxisDescriptor> axisDescriptors() const override;
	bool forward(const double* q, std::size_t qCount, std::vector<std::array<double, 16>>& linkWorld) const override;

	bool applyToSink(CustomDeviceBackendData& device, BackendDataManager* mgr, IRobotBackendPoseSink* sink,
					 const std::vector<double>& q, const double w0[16]) const;

	bool rebuildGraph();

private:
	CustomDeviceBackendData& m_device;
	kinematic_core::KinematicGraph m_graph;
	int m_rootLinkIdx = 0;
};

ROBOT_SCENE_API std::shared_ptr<Model> create(CustomDeviceBackendData& device);
ROBOT_SCENE_API bool forwardLinkWorldById(const CustomDeviceBackendData& device, BackendDataManager* mgr,
											const std::vector<double>& q,
											std::unordered_map<std::string, std::array<double, 16>>& worldByLink);

} // namespace CustomDeviceKinematicModel

#endif // ROBOTSCENE_CUSTOMDEVICEKINEMATICMODEL_H
