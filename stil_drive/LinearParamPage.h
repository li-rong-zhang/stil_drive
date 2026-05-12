// ============================================================================
// LinearParamPage.h
// 直线扫描参数页（模式 0）
// ============================================================================

#pragma once
#include "ParamPageBase.h"
#include "ui_LinearParamPage.h"

class LinearParamPage : public ParamPageBase
{
    Q_OBJECT
public:
    explicit LinearParamPage(QWidget* parent = nullptr);

    bool    validate(QString& errMsg) override;
    QString buildScript() override;
    QString summary() override;
    int     modeId() const override { return 0; }
    QString modeName() const override { return QStringLiteral("直线扫描"); }

    void saveSettings(QSettings& s) override;
    void loadSettings(QSettings& s) override;

    double scanLengthMm() const;
    double densityUmPerPt() const;
    double feedrateMmPerSec() const;

private slots:
    void onAnyChanged();

private:
    Ui::LinearParamPageClass ui;
};
