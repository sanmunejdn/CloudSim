/**
 * @file Command.h
 * @brief Base command interface for undo/redo.
 */
#ifndef GEOMETRICMODELINGPLUGIN_COMMAND_H
#define GEOMETRICMODELINGPLUGIN_COMMAND_H

#include <string>

namespace onecad::app::commands
{
class Command
{
public:
	virtual ~Command() = default;
	virtual bool execute() = 0;
	virtual bool undo() = 0;
	virtual std::string label() const { return {}; }
};

} // namespace onecad::app::commands

#endif // GEOMETRICMODELINGPLUGIN_COMMAND_H
