/**
 * @file RemoveOperationCommand.h
 * @brief Command to remove an operation from history and regenerate.
 */
#ifndef ONECAD_APP_COMMANDS_REMOVEOPERATIONCOMMAND_H
#define ONECAD_APP_COMMANDS_REMOVEOPERATIONCOMMAND_H

#include "Command.h"
#include "../document/OperationRecord.h"

#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class RemoveOperationCommand : public Command {
public:
    RemoveOperationCommand(Document* document, std::string opId);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Remove Operation"; }

private:
    Document* document_;
    std::string opId_;
    OperationRecord removedRecord_;
    int removedIndex_ = -1;
    bool wasSuppressed_ = false;
    bool hasRecord_ = false;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_REMOVEOPERATIONCOMMAND_H
