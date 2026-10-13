#ifndef CLOUDSIMHOST_MULTIVIEWSHELL_H
#define CLOUDSIMHOST_MULTIVIEWSHELL_H

/// @file MultiViewShell.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 文档内多视角分屏壳（QSplitter，禁止 Stack 包 GL）

#include "MultiViewTypes.h"
#include "SatelliteOsgView.h"
#include "widget_global.h"

#include <QPointer>
#include <QVector>
#include <QWidget>

class OsgWidget;
class DetachedViewWindow;
class QSplitter;
class QFrame;
class QToolButton;

/// 文档内多视角分屏壳：Primary + 卫星格，活动格高亮
class OSG_WIDGET_API MultiViewShell : public QWidget
{
	Q_OBJECT
public:
	explicit MultiViewShell(OsgWidget* primary, QWidget* parent = nullptr);
	~MultiViewShell() override;

	OsgWidget* primaryWidget() const { return m_primary; }

	void setViewLayout(cloudsim::host::MultiViewLayout layout);
	cloudsim::host::MultiViewLayout viewLayout() const { return m_layout; }

	int activeViewIndex() const { return m_activeIndex; }
	void setActiveViewIndex(int index);

	/// 浮动视角窗；达上限返回 false
	bool openDetachedViewWindow();
	void closeAllDetachedWindows();
	int detachedWindowCount() const;

	void persistSettings(const QString& documentId) const;
	void restoreSettings(const QString& documentId);

	void setUseChinese(bool chinese);
	void refreshChrome();

signals:
	void layoutChanged(cloudsim::host::MultiViewLayout layout);
	void activeViewChanged(int index);

protected:
	bool eventFilter(QObject* watched, QEvent* event) override;

private:
	struct Cell
	{
		QFrame* frame = nullptr;
		QWidget* contentHost = nullptr;
		QToolButton* cornerBtn = nullptr;
		SatelliteOsgView* satellite = nullptr; // 格 0 为 nullptr（Primary）
	};

	void rebuildLayout();
	void ensureSatelliteCount(int count);
	void destroySatellitesBeyond(int keep);
	SatelliteOsgView* createSatellite();
	void placePrimaryInCell(int index);
	void updateActiveChrome();
	void wireCellActivation(int index);
	void showCornerMenu(int cellIndex);
	void applyPresetToCell(int cellIndex, SatelliteOsgView::ViewPreset preset);
	void applyPresetToPrimary(SatelliteOsgView::ViewPreset preset);
	static int slotCountFor(cloudsim::host::MultiViewLayout layout);

	OsgWidget* m_primary = nullptr;
	cloudsim::host::MultiViewLayout m_layout = cloudsim::host::MultiViewLayout::Single;
	int m_activeIndex = 0;
	bool m_useChinese = true;

	QSplitter* m_rootSplitter = nullptr;
	QVector<Cell> m_cells;
	QVector<QPointer<DetachedViewWindow>> m_detached;
	static constexpr int kMaxDetached = 2;
};

#endif // CLOUDSIMHOST_MULTIVIEWSHELL_H
