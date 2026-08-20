#ifndef GEOMETRICMODELINGPLUGIN_COMMANDSTACK_H
#define GEOMETRICMODELINGPLUGIN_COMMANDSTACK_H

/// @file CommandStack.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 移植自 OneCAD Command / CommandProcessor（MIT）

#include <QObject>
#include <QString>
#include <memory>
#include <string>
#include <vector>

class GeomodelingCommand
{
public:
	virtual ~GeomodelingCommand() = default;
	virtual bool execute() = 0;
	virtual bool undo() = 0;
	virtual std::string label() const { return {}; }
};

class CommandStack : public QObject
{
	Q_OBJECT
public:
	explicit CommandStack(QObject* parent = nullptr);

	bool execute(std::unique_ptr<GeomodelingCommand> command);
	void undo();
	void redo();
	bool canUndo() const;
	bool canRedo() const;
	void clear();

signals:
	void canUndoChanged(bool);
	void canRedoChanged(bool);

private:
	std::vector<std::unique_ptr<GeomodelingCommand>> m_undo;
	std::vector<std::unique_ptr<GeomodelingCommand>> m_redo;
};

#endif
