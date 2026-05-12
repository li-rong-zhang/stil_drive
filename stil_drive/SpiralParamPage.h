// ============================================================================
// SpiralParamPage.h
// 平面螺旋扫描参数页（模式 2）
// ============================================================================

#pragma once
#include "ParamPageBase.h"
#include "ui_SpiralParamPage.h"

class SpiralParamPage : public ParamPageBase
{
    Q_OBJECT
public:
    explicit SpiralParamPage(QWidget* parent = nullptr);

    bool    validate(QString& errMsg) override;
    QString buildScript() override;
    QString summary() override;
    int     modeId() const override { return 2; }
    QString modeName() const override { return QStringLiteral("平面螺旋扫描"); }

    void saveSettings(QSettings& s) override;
    void loadSettings(QSettings& s) override;

    double radiusMm() const;
    double pitchUmPerRev() const;
    double densityDegPerPt() const;
    double angularSpeedDegPerSec() const;
    double overCenterMm() const;

private slots:
    void onAnyChanged();

private:
    Ui::SpiralParamPageClass ui;
};
