// ============================================================================
// SphericalParamPage.h
// 球面纬线扫描参数页（模式 3）
// ============================================================================
// 多纬线扫描调度：
//   页面持有 m_currentLatIndex（0 起算）。每条纬线扫描完成后，主窗口检查
//   hasNext() 并提示用户确认 X/Z 移动量；用户确认后调用 advance() 推进，
//   并由主窗口下发 buildPositioningScriptToCurrent() 生成的定位脚本。
//   再次点击"开始测量"即扫描新纬线（buildScript() 始终对应当前 index）。
// ============================================================================

#pragma once
#include "ParamPageBase.h"
#include "ui_SphericalParamPage.h"

class SphericalParamPage : public ParamPageBase
{
    Q_OBJECT
public:
    explicit SphericalParamPage(QWidget* parent = nullptr);

    bool    validate(QString& errMsg) override;
    QString buildScript() override;            // 单条纬线扫描（当前 index）
    QString summary() override;
    int     modeId() const override { return 3; }
    QString modeName() const override { return QStringLiteral("球面纬线扫描"); }

    void saveSettings(QSettings& s) override;
    void loadSettings(QSettings& s) override;

    // ---- 参数 getter ----
    bool   isConvex() const;
    double sphereRadiusMm() const;
    double latStartDeg() const;
    double latEndDeg() const;
    double latStepDeg() const;
    double densityPerDeg() const;
    double cSpeedDegPerSec() const;
    double overScanDeg() const;

    // ---- 多纬线调度 ----
    int    numLatitudes() const;
    int    currentIndex() const { return m_currentLatIndex; }
    double currentLatitude() const;            // 由 start/step/index 推算
    bool   hasNext() const;                    // 还有下一条
    double peekNextLatitude() const;           // 下一条的纬度值（hasNext()=true 时有效）

    // 生成"从当前纬度移动到下一纬度"的 X/Z 定位脚本
    QString buildPositioningScriptToNext() const;

    // 调度操作
    void advance();   // index++，刷新进度标签
    void resetIndex();// index=0，刷新进度标签

private slots:
    void onAnyChanged();
    void onResetLatClicked();

private:
    void refreshProgressLabel();

    Ui::SphericalParamPageClass ui;
    int m_currentLatIndex = 0;
};
