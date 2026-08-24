#ifndef ROBOTSCENE_COMPOSITEKINEMATICMODEL_H
#define ROBOTSCENE_COMPOSITEKINEMATICMODEL_H

#include "robot_scene_global.h"

#include "IKinematicModel.h"
#include "RobotKinematicApplyContext.h"

#include <QVector>
#include <memory>
#include <vector>

namespace CompositeKinematicModel
{
/// 顺序拼接多段 q： [segment0..., segment1...]
class ROBOT_SCENE_API Model : public kinematic_core::IKinematicModel
{
public:
	void addSegment(std::shared_ptr<kinematic_core::IKinematicModel> segment);

	Model() = default;
	Model(const Model&) = delete;
	Model& operator=(const Model&) = delete;

	const kinematic_core::KinematicGraph& graph() const override;
	int dofCount() const override;
	std::vector<kinematic_core::AxisDescriptor> axisDescriptors() const override;
	bool forward(const double* q, std::size_t qCount, std::vector<std::array<double, 16>>& linkWorld) const override;

	int armDofCount() const;
	int externalDofCount() const;

	/// fullQ = [armQ | extQ]；缺 ext 段时从 doc 读取
	bool applyToSink(const RobotKinematicApplyContext::Context& ctx, const std::vector<double>& armQ,
					 const std::vector<double>* externalFullQ, QVector<double>& aggregatedAnglesRad) const;

	/// 仅臂段（兼容旧调用）
	bool applyArmToSink(const RobotKinematicApplyContext::Context& ctx, const std::vector<double>& localArmQ,
						QVector<double>& aggregatedAnglesRad) const;

private:
	std::vector<std::shared_ptr<kinematic_core::IKinematicModel>> m_segments;
	mutable kinematic_core::KinematicGraph m_cachedGraph;
};

} // namespace CompositeKinematicModel

#endif // ROBOTSCENE_COMPOSITEKINEMATICMODEL_H
