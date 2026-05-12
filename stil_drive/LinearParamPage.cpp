#include "LinearParamPage.h"
#include "PmacScriptGen.h"

static constexpr double F_MAX_HZ = 2000.0;

LinearParamPage::LinearParamPage(QWidget* parent) : ParamPageBase(parent)
{
    ui.setupUi(this);

    connect(ui.lineEdit_Length,  &QLineEdit::textChanged, this, &LinearParamPage::onAnyChanged);
    connect(ui.lineEdit_Density, &QLineEdit::textChanged, this, &LinearParamPage::onAnyChanged);
    connect(ui.lineEdit_Speed,   &QLineEdit::textChanged, this, &LinearParamPage::onAnyChanged);

    onAnyChanged();
}

void LinearParamPage::onAnyChanged()
{
    ui.lbl_Summary->setText(summary());
    emit paramChanged();
}

double LinearParamPage::scanLengthMm() const     { return ui.lineEdit_Length->text().toDouble(); }
double LinearParamPage::densityUmPerPt() const   { return ui.lineEdit_Density->text().toDouble(); }
double LinearParamPage::feedrateMmPerSec() const { return ui.lineEdit_Speed->text().toDouble(); }

bool LinearParamPage::validate(QString& errMsg)
{
    double L = scanLengthMm(), d = densityUmPerPt(), v = feedrateMmPerSec();
    if (L <= 0)  { errMsg = QStringLiteral("扫描长度必须 > 0"); return false; }
    if (d <= 0)  { errMsg = QStringLiteral("触发密度必须 > 0"); return false; }
    if (v <= 0)  { errMsg = QStringLiteral("速度必须 > 0"); return false; }
    double f = (v * 1000.0) / d;  // Hz = (mm/s × 1000 µm/mm) / (µm/pt)
    if (f > F_MAX_HZ) {
        errMsg = QStringLiteral("预计触发频率 %1 Hz 超过测头上限 %2 Hz，请降速或增大密度。")
                    .arg(f, 0, 'f', 1).arg(F_MAX_HZ, 0, 'f', 0);
        return false;
    }
    return true;
}

QString LinearParamPage::buildScript()
{
    return PmacScriptGen::generateLinearScript(
        scanLengthMm(), densityUmPerPt(), feedrateMmPerSec());
}

QString LinearParamPage::summary()
{
    double L = scanLengthMm(), d = densityUmPerPt(), v = feedrateMmPerSec();
    if (d <= 0 || v <= 0 || L <= 0)
        return QStringLiteral("预计触发频率：— （请填写完整参数）");

    double f = (v * 1000.0) / d;
    double t = L / v;
    long   n = (long)((L * 1000.0) / d);

    QString warn = (f > F_MAX_HZ)
        ? QStringLiteral("<font color='red'> [超频]</font>") : QString();
    return QStringLiteral("预计频率: %1 Hz | 时间: %2 s | 点数: %3%4")
        .arg(f, 0, 'f', 1).arg(t, 0, 'f', 2).arg(n).arg(warn);
}

void LinearParamPage::saveSettings(QSettings& s)
{
    s.beginGroup("Linear");
    s.setValue("length",  ui.lineEdit_Length->text());
    s.setValue("density", ui.lineEdit_Density->text());
    s.setValue("speed",   ui.lineEdit_Speed->text());
    s.endGroup();
}

void LinearParamPage::loadSettings(QSettings& s)
{
    s.beginGroup("Linear");
    ui.lineEdit_Length->setText(s.value("length",  "10").toString());
    ui.lineEdit_Density->setText(s.value("density","1.0").toString());
    ui.lineEdit_Speed->setText(s.value("speed",    "1.0").toString());
    s.endGroup();
    onAnyChanged();
}
