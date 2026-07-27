/**
 * @file EditOperationInputCommand.h
 * @brief Re-profile an operation: change its primary input (e.g. swap an extrude's sketch region).
 */
#ifndef ONECAD_APP_COMMANDS_EDITOPERATIONINPUTCOMMAND_H
#define ONECAD_APP_COMMANDS_EDITOPERATIONINPUTCOMMAND_H

#include "Command.h"
#include "../document/OperationRecord.h"

#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class EditOperationInputCommand : public Command {
public:
    EditOperationInputCommand(Document* document, std::string opId, OperationInput newInput);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Re-profile Operation"; }

private:
    Document* document_ = nullptr;
    std::string opId_;
    OperationInput newInput_;
    OperationInput oldInput_;
    bool hasOldInput_ = false;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_EDITOPERATIONINPUTCOMMAND_H
