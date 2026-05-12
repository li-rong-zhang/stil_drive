#include "AsphericParamPage.h"
#include "PmacScriptGen.h"
#include "AsphericMath.h"

static constexpr double F_MAX_HZ = 2000.0;

AsphericParamPage::AsphericParamPage(QWidget* parent) : ParamPageBase(parent)
{
    ui.setupUi(this);

    auto editChanged = [this]() { onAnyChanged(); };
    QList<QLineEdit*> edits = {
        ui.lineEdit_R, ui.lineEdit_K,
        ui.lineEdit_A4, ui.lineEdit_A6, ui.lineEdit_A8, ui.lineEdit_A10, ui.lineEdit_A12,
        ui.lineEdit_StartR, ui.lineEdit_Range,
        ui.lineEdit_Density, ui.lineEdit_Feedrate,
    };
    for (auto* e : edits) connect(e, &QLineEdit::textChanged, this, editChanged);

    onAnyChanged();
}

void AsphericParamPage::onAnyChanged()
{
    ui.lbl_Summary->setText(summary());
    emit paramChanged();
}

double AsphericParamPage::R_mm() const             { return ui.lineEdit_R->text().toDouble(); }
double AsphericParamPage::K() const                { return ui.lineEdit_K->text().toDouble(); }
double AsphericParamPage::A4() const               { return ui.lineEdit_A4->text().toDouble(); }
double AsphericParamPage::A6() const               { return ui.lineEdit_A6->text().toDouble(); }
double AsphericParamPage::A8() const               { return ui.lineEdit_A8->text().toDouble(); }
double AsphericParamPage::A10() const              { return ui.lineEdit_A10->text().toDouble(); }
double AsphericParamPage::A12() const              { return ui.lineEdit_A12->text().toDouble(); }
double AsphericParamPage::startRadiusUm() const    { return ui.lineEdit_StartR->text().toDouble(); }
double AsphericParamPage::scanRangeUm() const      { return ui.lineEdit_Range->text().toDouble(); }
double AsphericParamPage::densityUmPerPt() const   { return ui.lineEdit_Density->text().toDouble(); }
double AsphericParamPage::feedrateMmPerSec() const { return ui.lineEdit_Feedrate->text().toDouble(); }

bool AsphericParamPage::validate(QString& errMsg)
{
    if (R_mm() == 0.0)            { errMsg = QStringLiteral("曲率 R 不能为 0"); return false; }
    if (scanRangeUm() <= 0)       { errMsg = QStringLiteral("扫描范围必须 > 0"); return false; }
    if (feedrateMmPerSec() <= 0)  { errMsg = QStringLiteral("进给速度必须 > 0"); return false; }

    // 推荐安全步距：基于斜率求导
    double safeStepUm = AsphericMath::calcSafeStepSize(
        startRadiusUm() / 1000.0, R_mm(), K(), A4(), A6(), A8(), A10(), A12());

    double dens = densityUmPerPt();
    if (dens > 0 && dens > safeStepUm * 1.5) {
        errMsg = QStringLiteral("陡坡风险：当前触发密度 %1 µm/pt 大于推荐安全步距 %2 µm/pt，"
                                "Z 方向跨度可能超出插补精度，建议减小密度。")
                    .arg(dens, 0, 'f', 2).arg(safeStepUm, 0, 'f', 2);
        return false;
    }

    double effDens = (dens > 0) ? dens : safeStepUm;
    double f = (feedrateMmPerSec() * 1000.0) / effDens;
    if (f > F_MAX_HZ) {
        errMsg = QStringLiteral("预计触发频率 %1 Hz 超过 %2 Hz。").arg(f, 0, 'f', 1).arg(F_MAX_HZ);
        return false;
    }
    return true;
}

QString AsphericParamPage::buildScript()
{
    return PmacScriptGen::generateAsphericMeridianScript(
        startRadiusUm(), scanRangeUm(),
        R_mm(), K(), A4(), A6(), A8(), A10(), A12(),
        densityUmPerPt(), feedrateMmPerSec());
}

QString AsphericParamPage::summary()
{
    if (R_mm() <= 0 || scanRangeUm() <= 0 || feedrateMmPerSec() <= 0)
        return QStringLiteral("—（请填写完整参数）");

    double safeStepUm = AsphericMath::calcSafeStepSize(
        startRadiusUm() / 1000.0, R_mm(), K(), A4(), A6(), A8(), A10(), A12());
    double dens = densityUmPerPt();
    double effDens = (dens > 0) ? dens : safeStepUm;
    long   nPts = (long)(scanRangeUm() / effDens) + 1;
    double f = (feedrateMmPerSec() * 1000.0) / effDens;
    double t = (scanRangeUm() / 1000.0) / feedrateMmPerSec();

    QString warn;
    if (f > F_MAX_HZ) warn = QStringLiteral("<font color='red'> [超频]</font>");

    return QStringLiteral("推荐步距: %1 µm | 实步距: %2 µm | 频率: %3 Hz | 时间: %4 s | 点数: %5%6")
        .arg(safeStepUm, 0, 'f', 2)
        .arg(effDens, 0, 'f', 2)
        .arg(f, 0, 'f', 1).arg(t, 0, 'f', 2).arg(nPts).arg(warn);
}

void AsphericParamPage::saveSettings(QSettings& s)
{
    s.beginGroup("Aspheric");
    s.setValue("R",       ui.lineEdit_R->text());
    s.setValue("K",       ui.lineEdit_K->text());
    s.setValue("A4",      ui.lineEdit_A4->text());
    s.setValue("A6",      ui.lineEdit_A6->text());
    s.setValue("A8",      ui.lineEdit_A8->text());
    s.setValue("A10",     ui.lineEdit_A10->text());
    s.setValue("A12",     ui.lineEdit_A12->text());
    s.setValue("startR",  ui.lineEdit_StartR->text());
    s.setValue("range",   ui.lineEdit_Range->text());
    s.setValue("density", ui.lineEdit_Density->text());
    s.setValue("feed",    ui.lineEdit_Feedrate->text());
    s.endGroup();
}

void AsphericParamPage::loadSettings(QSettings& s)
{
    s.beginGroup("Aspheric");
    ui.lineEdit_R->setText(       s.value("R",       "100").toString());
    ui.lineEdit_K->setText(       s.value("K",       "0").toString());
    ui.lineEdit_A4->setText(      s.value("A4",      "0").toString());
    ui.lineEdit_A6->setText(      s.value("A6",      "0").toString());
    ui.lineEdit_A8->setText(      s.value("A8",      "0").toString());
    ui.lineEdit_A10->setText(     s.value("A10",     "0").toString());
    ui.lineEdit_A12->setText(     s.value("A12",     "0").toString());
    ui.lineEdit_StartR->setText(  s.value("startR",  "10000").toString());
    ui.lineEdit_Range->setText(   s.value("range",   "50000").toString());
    ui.lineEdit_Density->setText( s.value("density", "0").toString());
    ui.lineEdit_Feedrate->setText(s.value("feed",    "1.0").toString());
    s.endGroup();
    onAnyChanged();
}
