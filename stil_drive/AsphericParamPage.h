// ============================================================================
// AsphericParamPage.h
// 非球面母线扫描参数页（模式 4）
// ============================================================================

#pragma once
#include "ParamPageBase.h"
#include "ui_AsphericParamPage.h"

class AsphericParamPage : public ParamPageBase
{
    Q_OBJECT
public:
    explicit AsphericParamPage(QWidget* parent = nullptr);

    bool    validate(QString& errMsg) override;
    QString buildScript() override;
    QString summary() override;
    int     modeId() const override { return 4; }
    QString modeName() const override { return QStringLiteral("非球面母线扫描"); }

    void saveSettings(QSettings& s) override;
    void loadSettings(QSettings& s) override;

    double R_mm() const;
    double K() const;
    double A4() const;  double A6() const;
    double A8() const;  double A10() const;
    double A12() const;
    double startRadiusUm() const;
    double scanRangeUm() const;
    double densityUmPerPt() const;
    double feedrateMmPerSec() const;

private slots:
    void onAnyChanged();

private:
    Ui::AsphericParamPageClass ui;
};
