#ifndef ROBOTURDF_URDFROBOTKINEMATICMODEL_H
#define ROBOTURDF_URDFROBOTKINEMATICMODEL_H

#include "robot_urdf_global.h"

#include "IKinematicModel.h"
#include "KinematicGraph.h"

#include <QString>
#include <QStringList>
#include <QVector>
#include <memory>

namespace UrdfRobotKinematicModel
{
class ROBOT_URDF_API Model : public kinematic_core::IKinematicModel
{
public:
	explicit Model(QString urdfPath);

	const kinematic_core::KinematicGraph& graph() const override { return m_graph; }
	int dofCount() const override { return m_graph.dofCount(); }
	std::vector<kinematic_core::AxisDescriptor> axisDescriptors() const override;
	bool forward(const double* q, std::size_t qCount, std::vector<std::array<double, 16>>& linkWorld) const override;

	bool rebuildGraph(QString* errorMessage = nullptr);

private:
	QString m_urdfPath;
	kinematic_core::KinematicGraph m_graph;
	QStringList m_jointNames;
	QVector<double> m_lowerRad;
	QVector<double> m_upperRad;
};

ROBOT_URDF_API std::shared_ptr<Model> create(const QString& urdfPath);

} // namespace UrdfRobotKinematicModel

#endif // ROBOTURDF_URDFROBOTKINEMATICMODEL_H
