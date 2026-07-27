/**
 * @file RenameBodyCommand.h
 * @brief Command that renames a body with undo support.
 */
#ifndef ONECAD_APP_COMMANDS_RENAMEBODYCOMMAND_H
#define ONECAD_APP_COMMANDS_RENAMEBODYCOMMAND_H

#include "Command.h"

#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class RenameBodyCommand : public Command {
public:
    RenameBodyCommand(Document* document,
                      const std::string& bodyId,
                      const std::string& newName);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Rename Body"; }

private:
    Document* document_ = nullptr;
    std::string bodyId_;
    std::string newName_;
    std::string oldName_;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_RENAMEBODYCOMMAND_H
