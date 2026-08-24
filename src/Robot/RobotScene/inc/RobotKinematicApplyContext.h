#ifndef ROBOTSCENE_ROBOTKINEMATICAPPLYCONTEXT_H
#define ROBOTSCENE_ROBOTKINEMATICAPPLYCONTEXT_H

class IRobotBackendPoseSink;
class IRobotSimulationDocument;

namespace RobotKinematicApplyContext
{
struct Context
{
	IRobotSimulationDocument* doc = nullptr;
	IRobotBackendPoseSink* sink = nullptr;
	int instanceIndex = -1;
};
} // namespace RobotKinematicApplyContext

#endif // ROBOTSCENE_ROBOTKINEMATICAPPLYCONTEXT_H
