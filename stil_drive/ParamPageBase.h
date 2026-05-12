// ============================================================================
// ParamPageBase.h
// 轨迹规划参数页 抽象基类
// ============================================================================
// 所有扫描模式（直线 / 螺旋 / 球面纬线 / 非球面母线）参数页面继承自此类，
// 主窗口通过 QStackedWidget 切换不同模式的参数面板，
// 通过 modeId() 选择 DataReducer 分支，buildScript() 调用 PmacScriptGen 生成脚本。
//
// 派生类需实现：
//   validate(errMsg) - UI 层安全校验（如频率 ≤ f_max、避开极点/赤道死区）
//   buildScript()    - 调 PmacScriptGen 对应函数生成 PMAC 脚本字符串
//   summary()        - 返回信息条文本（预计频率 / 时间 / 点数）
//   modeId()         - 返回 DataReducer 模式编号 (0/2/3/4)
//   modeName()       - 返回 UI 显示用的中文名
//   saveSettings/loadSettings - QSettings 持久化
// ============================================================================

#pragma once
#include <QWidget>
#include <QString>
#include <QSettings>

class ParamPageBase : public QWidget
{
    Q_OBJECT
public:
    explicit ParamPageBase(QWidget* parent = nullptr) : QWidget(parent) {}
    ~ParamPageBase() override = default;

    virtual bool    validate(QString& errMsg) = 0;
    virtual QString buildScript() = 0;
    virtual QString summary() = 0;
    virtual int     modeId() const = 0;
    virtual QString modeName() const = 0;

    virtual void saveSettings(QSettings& s) = 0;
    virtual void loadSettings(QSettings& s) = 0;

signals:
    void paramChanged();
};
