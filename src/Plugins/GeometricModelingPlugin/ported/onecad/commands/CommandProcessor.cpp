/**
 * @file CommandProcessor.cpp
 */
#include "CommandProcessor.h"

#include <algorithm>

namespace onecad::app::commands {

namespace {

class CommandGroup : public Command {
public:
    CommandGroup(std::string label, std::vector<std::unique_ptr<Command>> commands)
        : label_(std::move(label)),
          commands_(std::move(commands)) {
    }

    bool execute() override {
        size_t executed = 0;
        for (auto& cmd : commands_) {
            if (!cmd->execute()) {
                for (size_t i = executed; i-- > 0;) {
                    commands_[i]->undo();
                }
                return false;
            }
            ++executed;
        }
        return true;
    }

    bool undo() override {
        bool ok = true;
        for (size_t i = commands_.size(); i-- > 0;) {
            ok = commands_[i]->undo() && ok;
        }
        return ok;
    }

    std::string label() const override { return label_; }

private:
    std::string label_;
    std::vector<std::unique_ptr<Command>> commands_;
};

} // namespace

CommandProcessor::CommandProcessor(QObject* parent)
    : QObject(parent) {
}

bool CommandProcessor::execute(std::unique_ptr<Command> command) {
    if (!command) {
        return false;
    }

    emit commandStarted(QString::fromStdString(command->label()));
    const bool executed = command->execute();
    emit commandFinished();
    if (!executed) {
        if (inTransaction_) {
            cancelTransaction();
        }
        return false;
    }

    const bool prevUndo = canUndo();
    const bool prevRedo = canRedo();

    if (inTransaction_) {
        transaction_.push_back(std::move(command));
    } else {
        undoStack_.push_back(std::move(command));
        redoStack_.clear();
        // Bound memory: sketch gestures snapshot whole sketches, so a long
        // session would otherwise grow without limit.
        constexpr std::size_t kMaxUndoDepth = 200;
        if (undoStack_.size() > kMaxUndoDepth) {
            undoStack_.erase(undoStack_.begin());
        }
    }

    emitStateChange(prevUndo, prevRedo);
    return true;
}

void CommandProcessor::undo() {
    if (inTransaction_ || undoStack_.empty()) {
        return;
    }

    const bool prevUndo = canUndo();
    const bool prevRedo = canRedo();

    std::unique_ptr<Command> command = std::move(undoStack_.back());
    undoStack_.pop_back();

    if (command) {
        emit commandStarted(QString::fromStdString(command->label()));
        const bool undone = command->undo();
        emit commandFinished();
        if (undone) {
            redoStack_.push_back(std::move(command));
        } else {
            undoStack_.push_back(std::move(command));
        }
    }

    emitStateChange(prevUndo, prevRedo);
}

void CommandProcessor::redo() {
    if (inTransaction_ || redoStack_.empty()) {
        return;
    }

    const bool prevUndo = canUndo();
    const bool prevRedo = canRedo();

    std::unique_ptr<Command> command = std::move(redoStack_.back());
    redoStack_.pop_back();

    if (command) {
        emit commandStarted(QString::fromStdString(command->label()));
        const bool redone = command->execute();
        emit commandFinished();
        if (redone) {
            undoStack_.push_back(std::move(command));
        } else {
            redoStack_.push_back(std::move(command));
        }
    }

    emitStateChange(prevUndo, prevRedo);
}

bool CommandProcessor::canUndo() const {
    return !undoStack_.empty();
}

bool CommandProcessor::canRedo() const {
    return !redoStack_.empty();
}

void CommandProcessor::clear() {
    const bool prevUndo = canUndo();
    const bool prevRedo = canRedo();

    undoStack_.clear();
    redoStack_.clear();
    transaction_.clear();
    inTransaction_ = false;
    transactionLabel_.clear();

    emitStateChange(prevUndo, prevRedo);
}

void CommandProcessor::beginTransaction(const std::string& label) {
    if (inTransaction_) {
        return;
    }
    inTransaction_ = true;
    transactionLabel_ = label;
    transaction_.clear();
}

void CommandProcessor::endTransaction() {
    if (!inTransaction_) {
        return;
    }

    const bool prevUndo = canUndo();
    const bool prevRedo = canRedo();

    if (transaction_.empty()) {
        inTransaction_ = false;
        transactionLabel_.clear();
        emitStateChange(prevUndo, prevRedo);
        return;
    }

    if (transaction_.size() == 1) {
        undoStack_.push_back(std::move(transaction_.front()));
    } else {
        auto group = std::make_unique<CommandGroup>(transactionLabel_, std::move(transaction_));
        undoStack_.push_back(std::move(group));
    }
    redoStack_.clear();

    inTransaction_ = false;
    transactionLabel_.clear();
    transaction_.clear();

    emitStateChange(prevUndo, prevRedo);
}

void CommandProcessor::cancelTransaction() {
    if (!inTransaction_) {
        return;
    }

    const bool prevUndo = canUndo();
    const bool prevRedo = canRedo();

    for (size_t i = transaction_.size(); i-- > 0;) {
        transaction_[i]->undo();
    }

    transaction_.clear();
    inTransaction_ = false;
    transactionLabel_.clear();

    emitStateChange(prevUndo, prevRedo);
}

void CommandProcessor::emitStateChange(bool prevUndo, bool prevRedo) {
    const bool nowUndo = canUndo();
    const bool nowRedo = canRedo();
    if (prevUndo != nowUndo) {
        emit canUndoChanged(nowUndo);
    }
    if (prevRedo != nowRedo) {
        emit canRedoChanged(nowRedo);
    }
}

} // namespace onecad::app::commands
