/**
 * @file RollbackCommand.h
 * @brief Command to rollback to a specific operation.
 */
#ifndef ONECAD_APP_COMMANDS_ROLLBACKCOMMAND_H
#define ONECAD_APP_COMMANDS_ROLLBACKCOMMAND_H

#include "Command.h"
#include <cstddef>
#include <string>
#include <unordered_map>

namespace onecad {
namespace app {

class Document;

namespace commands {

/**
 * @brief Undoable command to rollback to a specific operation.
 *
 * Maintains suppression behavior for downstream operations and also moves the
 * applied operation cursor so new operations insert after the rollback point.
 * Can be undone to restore the prior suppression map and applied cursor.
 */
class RollbackCommand : public Command {
public:
    RollbackCommand(Document* document, const std::string& targetOpId);

    bool execute() override;
    bool undo() override;

private:
    Document* document_;
    std::string targetOpId_;
    std::unordered_map<std::string, bool> previousSuppression_;
    std::size_t previousAppliedOpCount_ = 0;
    std::size_t targetAppliedOpCount_ = 0;
};

} // namespace commands
} // namespace app
} // namespace onecad

#endif // ONECAD_APP_COMMANDS_ROLLBACKCOMMAND_H
