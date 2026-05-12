#include "TrajectoryPlannerDock.h"
#include "ParamPageBase.h"

#include <QStackedWidget>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QWidget>

TrajectoryPlannerDock::TrajectoryPlannerDock(QWidget* parent)
    : QDockWidget(QStringLiteral("轨迹规划"), parent)
{
    setObjectName("dock_TrajectoryPlanner");
    setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    auto* container = new QWidget(this);
    auto* layout    = new QVBoxLayout(container);
    layout->setContentsMargins(6, 6, 6, 6);

    m_lblMode = new QLabel(QStringLiteral("当前模式：（未选择）"), container);
    m_lblMode->setStyleSheet("font-weight: bold;");
    layout->addWidget(m_lblMode);

    auto* line = new QFrame(container);
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);

    m_stack = new QStackedWidget(container);
    layout->addWidget(m_stack, 1);

    auto* btnRow = new QHBoxLayout();
    m_btnValidate = new QPushButton(QStringLiteral("校验"), container);
    m_btnGenerate = new QPushButton(QStringLiteral("生成脚本"), container);
    m_btnStart    = new QPushButton(QStringLiteral("发送脚本"), container);
    m_btnValidate->setToolTip(QStringLiteral("仅做参数 UI 安全校验（频率/极点/赤道/陡坡），不执行任何运动。"));
    m_btnGenerate->setToolTip(QStringLiteral("根据当前参数重新生成脚本并写入预览（不下发）。"));
    m_btnStart->setToolTip(QStringLiteral("校验参数 → 重新生成脚本 → 仅下发 PMAC（不连接测头，不启动采集）。"));
    btnRow->addWidget(m_btnValidate);
    btnRow->addWidget(m_btnGenerate);
    btnRow->addWidget(m_btnStart);
    layout->addLayout(btnRow);

    setWidget(container);
    setMinimumWidth(320);

    connect(m_btnValidate, &QPushButton::clicked, this, &TrajectoryPlannerDock::validateRequested);
    connect(m_btnGenerate, &QPushButton::clicked, this, &TrajectoryPlannerDock::generateRequested);
    connect(m_btnStart,    &QPushButton::clicked, this, &TrajectoryPlannerDock::startRequested);
}

void TrajectoryPlannerDock::addPage(ParamPageBase* page)
{
    if (!page) return;
    m_pages.append(page);
    m_stack->addWidget(page);
}

void TrajectoryPlannerDock::setMode(int modeId)
{
    for (int i = 0; i < m_pages.size(); ++i) {
        if (m_pages[i]->modeId() == modeId) {
            m_stack->setCurrentIndex(i);
            m_lblMode->setText(QStringLiteral("当前模式：%1").arg(m_pages[i]->modeName()));
            emit modeSwitched(modeId);
            return;
        }
    }
}

ParamPageBase* TrajectoryPlannerDock::currentPage() const
{
    int idx = m_stack->currentIndex();
    if (idx < 0 || idx >= m_pages.size()) return nullptr;
    return m_pages[idx];
}
