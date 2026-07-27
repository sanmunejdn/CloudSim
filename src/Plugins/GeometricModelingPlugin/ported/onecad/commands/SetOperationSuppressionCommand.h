/**
 * @file SetOperationSuppressionCommand.h
 * @brief Command to suppress/unsuppress an operation and regenerate.
 */
#ifndef ONECAD_APP_COMMANDS_SETOPERATIONSUPPRESSIONCOMMAND_H
#define ONECAD_APP_COMMANDS_SETOPERATIONSUPPRESSIONCOMMAND_H

#include "Command.h"
#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class SetOperationSuppressionCommand : public Command {
public:
    SetOperationSuppressionCommand(Document* document,
                                   std::string opId,
                                   bool suppressed);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Toggle Suppression"; }

private:
    Document* document_;
    std::string opId_;
    bool newSuppressed_ = false;
    bool oldSuppressed_ = false;
    bool hasOldState_ = false;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_SETOPERATIONSUPPRESSIONCOMMAND_H
