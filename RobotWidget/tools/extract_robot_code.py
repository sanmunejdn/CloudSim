# -*- coding: utf-8 -*-
import re
from pathlib import Path

root = Path(__file__).resolve().parents[2]
legacy = root.parent / "legacy_backup" / "vs_route_20260507_physical" / "Widget" / "source" / "MainWindow.cpp"
mw = legacy if legacy.is_file() else root / "Widget" / "source" / "MainWindow.cpp"
print("source", mw)
lines = mw.read_text(encoding="utf-8").splitlines(keepends=True)

math_body = "".join(lines[60:548])
math_body = math_body.replace("DocumentPage*", "IRobotDocumentHost*")
math_body = math_body.replace("OsgWidget*", "IRobotOsgViewHost*")
math_body = re.sub(r"doc->sceneFacade\(\)\.poseSink\(\)", "doc->poseSink()", math_body)

math_hdr = """#include "RobotSimulationMath.h"
#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotInstructionProgram.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionTransform.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotTeachIk.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"
#include <Adapters.h>
#include <RigidTransform.h>
#include <ToolKinematics.h>
#include <QFile>
#include <QRegularExpression>
#include <osg/Matrixd>
#include <osg/Quat>
#include <algorithm>
#include <cmath>

namespace RobotSimulationMath {
namespace {
"""
math_ftr = """
} // namespace
} // namespace RobotSimulationMath
"""
(root / "RobotWidget" / "source" / "RobotSimulationMath.cpp").write_text(
    math_hdr + math_body + math_ftr, encoding="utf-8", newline="\n"
)

# Controller body kept from existing RobotSimulationController.cpp (post-trim extraction)
body_path = root / "RobotWidget" / "source" / "RobotSimulationController.cpp"
body = body_path.read_text(encoding="utf-8", errors="replace")
# Strip existing header/init if re-run
marker = "void RobotSimulationController::syncRobotKinematicsAfterPoseEdit"
if marker in body:
    body = body[body.index(marker) :]
elif "void RobotSimulationController::stopRobotSimulation" in body:
    body = body[body.index("void RobotSimulationController::stopRobotSimulation") :]
repl = [
    ("void MainWindow::", "void RobotSimulationController::"),
    ("bool MainWindow::", "bool RobotSimulationController::"),
    ("QHash<QString, bool> MainWindow::", "QHash<QString, bool> RobotSimulationController::"),
    ("QVector<double> MainWindow::", "QVector<double> RobotSimulationController::"),
    ("DocumentPage* doc = currentPage()", "IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr"),
    ("currentPage()", "m_host->document()"),
    ("OsgWidget* osg = currentOsgWidget()", "IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr"),
    ("OsgWidget* osg = doc ? doc->osgWidget() : nullptr", "IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr"),
    ("doc->sceneFacade().poseSink()", "doc->poseSink()"),
    ("m_simulationCommandPage", "m_host->simulationCommandPage()"),
    ("m_robotAxisControlPage", "m_host->robotAxisControlPage()"),
    ("m_robotFrameSettingsPage", "m_host->robotFrameSettingsPage()"),
    ("m_robotProgramExecutor", "m_programExecutor"),
    ("m_robotInstructionController", "m_instructionController"),
    ("m_robotSimTimer", "m_playbackTimer"),
    ("m_simulationStartAction", "m_host->simulationStartAction()"),
    ("m_runInfoPage", "m_host->runInfoPage()"),
    ("invalidateFeasibleAxisConfigurationCache()", "m_host->invalidateInstructionPropertyCache()"),
    ("currentOsgWidget()", "m_host->osgView()"),
    ("backendvisual_math::eulerDegToQuat", "engine::eulerDegToQuat"),
    ("OsgWidget::RobotFrameOverlayUpdate", "RobotOsgUi::RobotFrameOverlayUpdate"),
    ("OsgWidget::InstructionPoseAxis", "RobotOsgUi::InstructionPoseAxis"),
    (
        "MainWindowSelectionService::clearBackendObjectSelection(*this, true)",
        "m_host->clearBackendObjectSelection(true)",
    ),
]
for a, b in repl:
    body = body.replace(a, b)

ctrl_hdr = """#include "RobotSimulationController.h"
#include "RobotSimulationDockWidget.h"
#include "RobotSimulationMath.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "SimulationCommandWidget.h"
#include "RobotAxisControlWidget.h"
#include "RobotFrameSettingsWidget.h"
#include "RobotProgramExport.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RobotSceneKinematics.h"
#include "RobotTeachIk.h"
#include "UrdfRobotLoader.h"
#include "RunLogger.h"
#include "RobotOsgUiTypes.h"
#include "../OsgWidgetCore/inc/ObjectGizmoFrame.h"
#include <Adapters.h>
#include <BackendDataBase.h>
#include <BackendDataManager.h>
#include <QMessageBox>
#include <QFile>
#include <osg/Matrixd>

using namespace RobotSimulation;

"""
ctrl_init = (root / "RobotWidget" / "source" / "RobotSimulationController_init.cpp").read_text(encoding="utf-8")
(root / "RobotWidget" / "source" / "RobotSimulationController.cpp").write_text(
    ctrl_hdr + ctrl_init + body, encoding="utf-8", newline="\n"
)
print("ok")
