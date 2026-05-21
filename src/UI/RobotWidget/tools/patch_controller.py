from pathlib import Path
import re

p = Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\RobotWidget\source\RobotSimulationController.cpp")
t = p.read_text(encoding="utf-8", errors="replace")

if "RobotInstructionPlanningHelpers.h" not in t:
    t = t.replace(
        '#include "RobotSimulationMath.h"',
        '#include "RobotSimulationMath.h"\n#include "RobotInstructionPlanningHelpers.h"',
    )
if "#include <QSet>" not in t:
    t = t.replace("#include <QFile>", "#include <QFile>\n#include <QSet>\n#include <QSignalBlocker>")

t = t.replace(
    "void RobotSimulationController::onSimulationAddInstructionRequested(RobotInstruction::Type type)\n"
    "void RobotSimulationController::onSimulationAddInstructionRequested(RobotInstruction::Type type)",
    "void RobotSimulationController::onSimulationAddInstructionRequested(RobotInstruction::Type type)",
)

t = t.replace(
    "RobotInstruction::FeasibleMotionAxisConfigurationOptions MainWindow::feasibleMotionAxisConfigurationOptionsForInstruction",
    "RobotInstruction::FeasibleMotionAxisConfigurationOptions RobotSimulationController::feasibleMotionAxisConfigurationOptionsForInstruction",
)

t = t.replace("m_useChinese", "m_host->useChinese()")
t = t.replace("updateInstructionPropertyPanel(", "m_host->refreshInstructionPropertyPanel(")
t = t.replace("\tm_activeInstructionForProperty = instruction;\n", "")

t = t.replace("MotionPoseBackup", "RobotInstructionPlanning::MotionPoseBackup")
t = t.replace("backupInstructionPose(", "RobotInstructionPlanning::backupInstructionPose(")
t = t.replace("restoreInstructionPose(", "RobotInstructionPlanning::restoreInstructionPose(")
t = t.replace("prepareMotionInstructionForPlanning(", "RobotInstructionPlanning::prepareMotionInstructionForPlanning(")

fns = [
    "defaultTcpLinkNameForUrdf",
    "targetRigidTransformFromUrdfFlangeFk",
    "targetInBaseFromUrdfFlangeFk",
    "robotBaseWorldMatrixForInstance",
    "linkMeshBackendIdForInstance",
    "toolMat4ForFrames",
    "toolTcpInBaseFromFk",
    "captureTcpFromSceneFlangeBackend",
    "osgMatrixFromRobotRigidFrame",
    "osgMatrixFromBackendMat4",
    "matrixFromNodeWorld",
    "encodeMatrix4Csv",
    "decodeMatrix4Csv",
    "buildDhRowsFromUrdf",
]
for fn in fns:
    t = t.replace(fn + "(", "RobotSimulationMath::" + fn + "(")
while "RobotSimulationMath::RobotSimulationMath::" in t:
    t = t.replace("RobotSimulationMath::RobotSimulationMath::", "RobotSimulationMath::")
while "RobotInstructionPlanning::RobotInstructionPlanning::" in t:
    t = t.replace(
        "RobotInstructionPlanning::RobotInstructionPlanning::",
        "RobotInstructionPlanning::",
    )

t = t.replace(
    "engine::eulerDegToQuat(\n\t\tosg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z)))",
    "engine::eulerDegToQuat(r.x, r.y, r.z)",
)
t = t.replace("m_playbackTimer.stop()", "m_playbackTimer->stop()")
t = t.replace("m_playbackTimer.start()", "m_playbackTimer->start()")
t = t.replace(
    "const DocumentPage* doc,\n\tOsgWidget* osg,",
    "IRobotDocumentHost* doc,\n\tIRobotOsgViewHost* osg,",
)

if "namespace InstructionPoseDiagState" not in t:
    stub = """
namespace InstructionPoseDiagState {
void requestRefresh() {}
bool shouldLog(const std::string&) { return false; }
}

"""
    t = t.replace("namespace\n{\nosg::Matrixd tcpLocalFromPoseFields", stub + "namespace\n{\nosg::Matrixd tcpLocalFromPoseFields", 1)

t = re.sub(
    r"\n\tif \(m_activeInstructionForProperty && m_activeInstructionForProperty->hasPoseProperty\(\)[\s\S]*?\n\t\}\n",
    "\n",
    t,
    count=1,
)
t = t.replace("if (/*diag*/nullptr", "if (false && nullptr")
t = t.replace("/*diag*/nullptr", "nullptr")

# applySuggestedAxisPreset still in MainWindow - add host delegate or keep calling via incomplete path
# grep applySuggestedAxisPresetFromSeedIfNeeded in controller - needs MainWindow method via host

p.write_text(t, encoding="utf-8")
print("done", p.stat().st_size)
