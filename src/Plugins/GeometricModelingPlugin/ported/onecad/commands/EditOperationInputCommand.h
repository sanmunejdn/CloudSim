/**
 * @file EditOperationInputCommand.h
 * @brief Re-profile an operation: change its primary input (e.g. swap an extrude's sketch region).
 */
#ifndef GEOMETRICMODELINGPLUGIN_EDITOPERATIONINPUTCOMMAND_H
#define GEOMETRICMODELINGPLUGIN_EDITOPERATIONINPUTCOMMAND_H

#include "../document/OperationRecord.h"
#include "Command.h"

#include <string>

namespace onecad::app
{
class Document;
}

namespace onecad::app::commands
{
class EditOperationInputCommand : public Command
{
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

#endif // GEOMETRICMODELINGPLUGIN_EDITOPERATIONINPUTCOMMAND_H
