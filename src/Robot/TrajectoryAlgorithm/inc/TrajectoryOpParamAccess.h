#ifndef TRAJECTORYALGORITHM_TRAJECTORYOPPARAMACCESS_H
#define TRAJECTORYALGORITHM_TRAJECTORYOPPARAMACCESS_H

/// @file TrajectoryOpParamAccess.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TrajectoryOpParamAccess 接口

#include "trajectory_algorithm_global.h"

#include "ITrajectoryOp.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{
class TRAJECTORY_ALGORITHM_API TrajectoryOpParamAccess
{
public:
	static bool read(const RobotInstruction::TrajectoryOpDescriptor& op, const TrajectoryOpParamField& field,
					 TrajectoryParamValue& out);

	static bool write(RobotInstruction::TrajectoryOpDescriptor& op, const TrajectoryOpParamField& field,
					  const TrajectoryParamValue& in);

	static void applyDefaults(RobotInstruction::TrajectoryOpDescriptor& op, const ITrajectoryOp& algo);

	static const TrajectoryOpParamField* findField(const std::vector<TrajectoryOpParamField>& fields,
												   const std::string& key);

	static std::vector<TrajectoryOpParamField> allFieldsForOp(const ITrajectoryOp& algo);
};

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHM_TRAJECTORYOPPARAMACCESS_H
