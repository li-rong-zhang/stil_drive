#include "SpiralParamPage.h"
#include "PmacScriptGen.h"

static constexpr double F_MAX_HZ = 2000.0;

SpiralParamPage::SpiralParamPage(QWidget* parent) : ParamPageBase(parent)
{
    ui.setupUi(this);

    auto editChanged = [this]() { onAnyChanged(); };
    connect(ui.lineEdit_Radius,     &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_Pitch,      &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_Density,    &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_AngSpeed,   &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_OverCenter, &QLineEdit::textChanged, this, editChanged);

    onAnyChanged();
}

void SpiralParamPage::onAnyChanged()
{
    ui.lbl_Summary->setText(summary());
    emit paramChanged();
}

double SpiralParamPage::radiusMm() const              { return ui.lineEdit_Radius->text().toDouble(); }
double SpiralParamPage::pitchUmPerRev() const         { return ui.lineEdit_Pitch->text().toDouble(); }
double SpiralParamPage::densityDegPerPt() const       { return ui.lineEdit_Density->text().toDouble(); }
double SpiralParamPage::angularSpeedDegPerSec() const { return ui.lineEdit_AngSpeed->text().toDouble(); }
double SpiralParamPage::overCenterMm() const          { return ui.lineEdit_OverCenter->text().toDouble(); }

bool SpiralParamPage::validate(QString& errMsg)
{
    double R = radiusMm(), P = pitchUmPerRev();
    double dens = densityDegPerPt(), w = angularSpeedDegPerSec();
    if (R <= 0)    { errMsg = QStringLiteral("半径必须 > 0"); return false; }
    if (P <= 0)    { errMsg = QStringLiteral("螺距必须 > 0"); return false; }
    if (dens <= 0) { errMsg = QStringLiteral("触发密度必须 > 0"); return false; }
    if (w <= 0)    { errMsg = QStringLiteral("角速度必须 > 0"); return false; }

    double f = w / dens;  // °/s ÷ °/pt = pt/s
    if (f > F_MAX_HZ) {
        errMsg = QStringLiteral("预计触发频率 %1 Hz 超过测头上限 %2 Hz，请降速或增大密度。")
                    .arg(f, 0, 'f', 1).arg(F_MAX_HZ, 0, 'f', 0);
        return false;
    }
    return true;
}

QString SpiralParamPage::buildScript()
{
    return PmacScriptGen::generateSpiralScript(
        radiusMm(), pitchUmPerRev(), densityDegPerPt(),
        angularSpeedDegPerSec(), overCenterMm());
}

QString SpiralParamPage::summary()
{
    double R = radiusMm(), P = pitchUmPerRev();
    double dens = densityDegPerPt(), w = angularSpeedDegPerSec();
    if (R<=0 || P<=0 || dens<=0 || w<=0)
        return QStringLiteral("—（请填写完整参数）");

    double turns = R * 1000.0 / P;          // 总圈数
    double f = w / dens;                    // 触发频率 Hz
    double t = turns * 360.0 / w;           // 总扫描时间 s
    long   n = (long)((turns * 360.0) / dens);

    QString warn;
    if (f > F_MAX_HZ)  warn += QStringLiteral("<font color='red'> [超频]</font>");
    return QStringLiteral("圈数: %1 | 频率: %2 Hz | 时间: %3 s | 点数: %4%5")
        .arg(turns, 0, 'f', 1).arg(f, 0, 'f', 1).arg(t, 0, 'f', 1).arg(n).arg(warn);
}

void SpiralParamPage::saveSettings(QSettings& s)
{
    s.beginGroup("Spiral");
    s.setValue("radius",     ui.lineEdit_Radius->text());
    s.setValue("pitch",      ui.lineEdit_Pitch->text());
    s.setValue("density",    ui.lineEdit_Density->text());
    s.setValue("angSpeed",   ui.lineEdit_AngSpeed->text());
    s.setValue("overCenter", ui.lineEdit_OverCenter->text());
    s.endGroup();
}

void SpiralParamPage::loadSettings(QSettings& s)
{
    s.beginGroup("Spiral");
    ui.lineEdit_Radius->setText(    s.value("radius",     "10").toString());
    ui.lineEdit_Pitch->setText(     s.value("pitch",      "10").toString());
    ui.lineEdit_Density->setText(   s.value("density",    "0.5").toString());
    ui.lineEdit_AngSpeed->setText(  s.value("angSpeed",   "1000").toString());
    ui.lineEdit_OverCenter->setText(s.value("overCenter", "2.0").toString());
    s.endGroup();
    onAnyChanged();
}
