// ============================================================================
// TrajectoryPlannerDock.h
// 轨迹规划 Dock —— 主窗口左侧停靠面板
// ============================================================================
// 内嵌 QStackedWidget，按当前测量模式切换显示对应 ParamPage。
// 顶部显示模式名标签，底部提供 [校验] [生成脚本] [发送脚本] 按钮组。
//
// 主窗口职责：
//   - 通过 setMode(modeId) 切换页面
//   - 监听 validateRequested / generateRequested / startRequested 信号执行动作
//   - 通过 currentPage() 取当前 ParamPage 调用接口
// ============================================================================

#pragma once
#include <QDockWidget>
#include <QList>

class QStackedWidget;
class QLabel;
class QPushButton;
class ParamPageBase;

class TrajectoryPlannerDock : public QDockWidget
{
    Q_OBJECT
public:
    explicit TrajectoryPlannerDock(QWidget* parent = nullptr);

    // 添加参数页（按 modeId 顺序），dock 持有所有权
    void addPage(ParamPageBase* page);

    // 按 modeId 切换页面（找不到匹配则忽略）
    void setMode(int modeId);

    // 当前激活页（可能为 null）
    ParamPageBase* currentPage() const;

    // 全部页（供主窗口遍历做 QSettings 加载/保存）
    const QList<ParamPageBase*>& pages() const { return m_pages; }

signals:
    void modeSwitched(int modeId);
    void validateRequested();
    void generateRequested();
    void startRequested();

private:
    QStackedWidget*        m_stack;
    QLabel*                m_lblMode;
    QPushButton*           m_btnValidate;
    QPushButton*           m_btnGenerate;
    QPushButton*           m_btnStart;
    QList<ParamPageBase*>  m_pages;
};
