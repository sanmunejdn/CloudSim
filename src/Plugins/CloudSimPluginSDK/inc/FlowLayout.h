#pragma once

/// @file FlowLayout.h
/// @brief 窄时折行、宽时行内拉伸，避免裁切或右侧留白

#include <QLayout>
#include <QList>
#include <QStyle>
#include <QVector>
#include <QWidget>

class FlowLayout final : public QLayout
{
public:
	explicit FlowLayout(QWidget* parent = nullptr, int hSpacing = 4, int vSpacing = 4)
		: QLayout(parent), m_hSpace(hSpacing), m_vSpace(vSpacing)
	{
		setContentsMargins(0, 0, 0, 0);
	}

	~FlowLayout() override
	{
		while (QLayoutItem* item = takeAt(0))
		{
			delete item;
		}
	}

	void addItem(QLayoutItem* item) override { m_items.append(item); }
	int count() const override { return m_items.size(); }
	QLayoutItem* itemAt(int index) const override
	{
		return (index >= 0 && index < m_items.size()) ? m_items.at(index) : nullptr;
	}
	QLayoutItem* takeAt(int index) override
	{
		return (index >= 0 && index < m_items.size()) ? m_items.takeAt(index) : nullptr;
	}
	Qt::Orientations expandingDirections() const override { return Qt::Horizontal; }
	bool hasHeightForWidth() const override { return true; }
	int heightForWidth(int width) const override { return doLayout(QRect(0, 0, width, 0), true); }
	void setGeometry(const QRect& rect) override
	{
		QLayout::setGeometry(rect);
		doLayout(rect, false);
	}
	QSize sizeHint() const override { return minimumSize(); }
	QSize minimumSize() const override
	{
		QSize size;
		for (QLayoutItem* item : m_items)
		{
			size = size.expandedTo(item->minimumSize());
		}
		const QMargins m = contentsMargins();
		size += QSize(m.left() + m.right(), m.top() + m.bottom());
		return size;
	}

private:
	static bool isExpanding(QLayoutItem* item)
	{
		if (!item)
		{
			return false;
		}
		// Qt5: widget() 非 const
		if (QWidget* w = item->widget())
		{
			const QSizePolicy::Policy hp = w->sizePolicy().horizontalPolicy();
			return hp == QSizePolicy::Expanding || hp == QSizePolicy::MinimumExpanding ||
				   hp == QSizePolicy::Ignored;
		}
		return item->expandingDirections() & Qt::Horizontal;
	}

	int spacingFor(QLayoutItem* a, QLayoutItem* b, bool horizontal) const
	{
		QWidget* wa = a ? a->widget() : nullptr;
		QWidget* wb = b ? b->widget() : nullptr;
		if (wa && wb)
		{
			return qMax(horizontal ? m_hSpace : m_vSpace,
						wa->style()->layoutSpacing(wa->sizePolicy().controlType(), wb->sizePolicy().controlType(),
												   horizontal ? Qt::Horizontal : Qt::Vertical));
		}
		return horizontal ? m_hSpace : m_vSpace;
	}

	int doLayout(const QRect& rect, bool testOnly) const
	{
		int left = 0;
		int top = 0;
		int right = 0;
		int bottom = 0;
		getContentsMargins(&left, &top, &right, &bottom);
		const QRect effective = rect.adjusted(+left, +top, -right, -bottom);
		if (effective.width() <= 0 || m_items.isEmpty())
		{
			return bottom;
		}

		// 先按 sizeHint 分行，再把行内剩余宽度分给 Expanding 控件
		QVector<QVector<QLayoutItem*>> lines;
		QVector<QLayoutItem*> current;
		int lineWidth = 0;
		for (QLayoutItem* item : m_items)
		{
			const int hintW = item->sizeHint().width();
			const int space =
				current.isEmpty() ? 0 : spacingFor(current.last(), item, true);
			const int nextWidth = lineWidth + space + hintW;
			if (!current.isEmpty() && nextWidth > effective.width())
			{
				lines.append(current);
				current.clear();
				lineWidth = 0;
			}
			if (!current.isEmpty())
			{
				lineWidth += spacingFor(current.last(), item, true);
			}
			current.append(item);
			lineWidth += hintW;
		}
		if (!current.isEmpty())
		{
			lines.append(current);
		}

		int y = effective.y();
		for (int li = 0; li < lines.size(); ++li)
		{
			const QVector<QLayoutItem*>& line = lines.at(li);
			int hintSum = 0;
			int spaceSum = 0;
			int expandCount = 0;
			for (int i = 0; i < line.size(); ++i)
			{
				hintSum += line.at(i)->sizeHint().width();
				if (i + 1 < line.size())
				{
					spaceSum += spacingFor(line.at(i), line.at(i + 1), true);
				}
				if (isExpanding(line.at(i)))
				{
					++expandCount;
				}
			}
			int extra = effective.width() - hintSum - spaceSum;
			if (extra < 0)
			{
				extra = 0;
			}
			// 无 Expanding 时均分，避免宽侧栏整行贴左留白
			const int growSlots = expandCount > 0 ? expandCount : line.size();
			const int extraEach = growSlots > 0 ? extra / growSlots : 0;
			int extraRemain = growSlots > 0 ? extra % growSlots : 0;

			int x = effective.x();
			int lineHeight = 0;
			for (int i = 0; i < line.size(); ++i)
			{
				QLayoutItem* item = line.at(i);
				QSize sz = item->sizeHint();
				const bool grow = expandCount > 0 ? isExpanding(item) : true;
				if (grow)
				{
					sz.setWidth(sz.width() + extraEach + (extraRemain > 0 ? 1 : 0));
					if (extraRemain > 0)
					{
						--extraRemain;
					}
				}
				if (!testOnly)
				{
					item->setGeometry(QRect(QPoint(x, y), sz));
				}
				x += sz.width();
				if (i + 1 < line.size())
				{
					x += spacingFor(item, line.at(i + 1), true);
				}
				lineHeight = qMax(lineHeight, sz.height());
			}
			y += lineHeight;
			if (li + 1 < lines.size())
			{
				int spaceY = m_vSpace;
				if (!line.isEmpty() && !lines.at(li + 1).isEmpty())
				{
					spaceY = spacingFor(line.last(), lines.at(li + 1).first(), false);
				}
				y += spaceY;
			}
		}
		return y - rect.y() + bottom;
	}

	QList<QLayoutItem*> m_items;
	int m_hSpace = 4;
	int m_vSpace = 4;
};
