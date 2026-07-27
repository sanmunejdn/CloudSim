/**
 * @file UpdateOperationParamsCommand.h
 * @brief Command to update operation parameters and regenerate.
 */
#ifndef ONECAD_APP_COMMANDS_UPDATEOPERATIONPARAMSCOMMAND_H
#define ONECAD_APP_COMMANDS_UPDATEOPERATIONPARAMSCOMMAND_H

#include "Command.h"
#include "../document/OperationRecord.h"

#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class UpdateOperationParamsCommand : public Command {
public:
    UpdateOperationParamsCommand(Document* document,
                                 std::string opId,
                                 OperationParams params);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Update Operation"; }

private:
    Document* document_;
    std::string opId_;
    OperationParams newParams_;
    OperationParams oldParams_;
    bool hasOldParams_ = false;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_UPDATEOPERATIONPARAMSCOMMAND_H
