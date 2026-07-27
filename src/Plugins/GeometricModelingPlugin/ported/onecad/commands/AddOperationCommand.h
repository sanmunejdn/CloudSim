/**
 * @file AddOperationCommand.h
 * @brief Command to append an operation to history and regenerate.
 */
#ifndef ONECAD_APP_COMMANDS_ADDOPERATIONCOMMAND_H
#define ONECAD_APP_COMMANDS_ADDOPERATIONCOMMAND_H

#include "Command.h"
#include "../document/OperationRecord.h"

#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class AddOperationCommand : public Command {
public:
    AddOperationCommand(Document* document, OperationRecord record);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Add Operation"; }

private:
    Document* document_;
    OperationRecord record_;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_ADDOPERATIONCOMMAND_H
