# -*- coding: utf-8 -*-
from pathlib import Path

lines = Path(
    r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\src\UI\Widget\source\MainWindowPropertyPanel.cpp"
).read_text(encoding="utf-8").splitlines(keepends=True)

def slice1based(start, end):
    return "".join(lines[start - 1 : end])

chunk = slice1based(33, 140) + slice1based(644, 1093)

chunk = chunk.replace(
    "void MainWindow::applySuggestedAxisPresetFromSeedIfNeeded(\n\tconst std::shared_ptr<RobotInstruction::Base>& instruction,",
    "void InstructionPropertyPanel::applySuggestedAxisPresetFromSeedIfNeeded(\n\tIRobotInstructionPropertyUiHost& host,\n\tconst std::shared_ptr<RobotInstruction::Base>& instruction,",
)
chunk = chunk.replace(
    "void MainWindow::updateInstructionPropertyPanel(\n\tconst std::shared_ptr<RobotInstruction::Base>& instruction,",
    "void InstructionPropertyPanel::update(\n\tIRobotInstructionPropertyUiHost& host,\n\tconst std::shared_ptr<RobotInstruction::Base>& instruction,",
)
chunk = chunk.replace("BackendMat4 instructionTcpForDisplay(const MainWindow& mw,", "BackendMat4 instructionTcpForDisplay(IRobotInstructionPropertyUiHost& host,")
chunk = chunk.replace("void applyInstructionPropertyViaService(MainWindow& mw,", "void applyInstructionPropertyViaService(IRobotInstructionPropertyUiHost& host,")
chunk = chunk.replace("instructionTcpForDisplay(*this,", "instructionTcpForDisplay(host,")
chunk = chunk.replace("applyInstructionPropertyViaService(*this,", "applyInstructionPropertyViaService(host,")
chunk = chunk.replace(
    "InstructionPropertyPanel::applySuggestedAxisPresetFromSeedIfNeeded(host,",
    "applySuggestedAxisPresetFromSeedIfNeeded(host,",
)

repls = [
    ("mw.currentPage()", "host.currentPage()"),
    ("mw.currentSimulationRobotInstanceIndex()", "host.currentSimulationRobotInstanceIndex()"),
    ("m_propertyBrowser", "host.propertyBrowser()"),
    ("m_variantManager", "host.variantManager()"),
    ("m_updatingPropertyBrowser", "host.updatingPropertyBrowserFlag()"),
    ("m_propertyEnumTokens", "host.propertyEnumTokens()"),
    ("m_activeInstructionForProperty", "host.activeInstructionForProperty()"),
    ("currentPage()", "host.currentPage()"),
    ("simulationCommandPage()", "host.simulationCommandPage()"),
    ("appendPropertyBrowserRow(", "host.appendPropertyBrowserRow("),
    ("propertyDisplayLabelForKey(", "host.propertyDisplayLabelForKey("),
    ("feasibleMotionAxisConfigurationOptionsForInstruction(", "host.feasibleMotionAxisConfigurationOptionsForInstruction("),
    ("m_robotSimulation", "host.robotSimulation()"),
    ("invalidateFeasibleAxisConfigurationCache()", "host.invalidateFeasibleAxisConfigurationCache()"),
    ("m_runInfoPage", "host.runInfoPage()"),
    ("i18n(", "host.i18n("),
]
for old, new in repls:
    chunk = chunk.replace(old, new)

header = """#include "InstructionPropertyPanel.h"

#include "IRobotInstructionPropertyUiHost.h"

#include <algorithm>
#include <memory>

#include <QMetaObject>
#include <QPointer>

#include "CoreTypes.h"
#include "DocumentPage.h"
#include "RobotCoordinateFrames.h"
#include "RobotInstructionPropertySchema.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionController.h"
#include "RobotInstructionPlanningHelpers.h"
#include "SimulationCommandWidget.h"

#include "BackendPropertyRow.h"

#include "qttreepropertybrowser.h"
#include "qtvariantproperty.h"

namespace
{

"""

footer = """
} // namespace

"""

Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\src\UI\RobotWidget\source\InstructionPropertyPanel.cpp").write_text(
    header + chunk + footer, encoding="utf-8"
)
print("ok", len(chunk))
