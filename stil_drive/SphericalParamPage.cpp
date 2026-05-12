#include "SphericalParamPage.h"
#include "PmacScriptGen.h"
#include <cmath>

static constexpr double F_MAX_HZ        = 2000.0;
static constexpr double EQUATOR_DEAD    = 50.0;  // |纬度| < 50° 拒绝（赤道死区）
static constexpr double POLE_DEG        = 90.0;  // 拒绝极点

SphericalParamPage::SphericalParamPage(QWidget* parent) : ParamPageBase(parent)
{
    ui.setupUi(this);

    auto editChanged = [this]() {
        // 改变 start/end/step 后纬线列表变化，重置当前 index 防越界
        if (m_currentLatIndex >= numLatitudes()) m_currentLatIndex = 0;
        onAnyChanged();
    };
    connect(ui.rb_Convex, &QRadioButton::toggled, this, editChanged);
    connect(ui.lineEdit_SphereR,   &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_LatStart,  &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_LatEnd,    &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_LatStep,   &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_Density,   &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_CSpeed,    &QLineEdit::textChanged, this, editChanged);
    connect(ui.lineEdit_OverScan,  &QLineEdit::textChanged, this, editChanged);
    connect(ui.btn_ResetLat,       &QPushButton::clicked,   this, &SphericalParamPage::onResetLatClicked);

    onAnyChanged();
}

void SphericalParamPage::onAnyChanged()
{
    refreshProgressLabel();
    ui.lbl_Summary->setText(summary());
    emit paramChanged();
}

void SphericalParamPage::onResetLatClicked()
{
    resetIndex();
}

// ---- getter ----
bool   SphericalParamPage::isConvex() const         { return ui.rb_Convex->isChecked(); }
double SphericalParamPage::sphereRadiusMm() const   { return ui.lineEdit_SphereR->text().toDouble(); }
double SphericalParamPage::latStartDeg() const      { return ui.lineEdit_LatStart->text().toDouble(); }
double SphericalParamPage::latEndDeg() const        { return ui.lineEdit_LatEnd->text().toDouble(); }
double SphericalParamPage::latStepDeg() const       { return ui.lineEdit_LatStep->text().toDouble(); }
double SphericalParamPage::densityPerDeg() const    { return ui.lineEdit_Density->text().toDouble(); }
double SphericalParamPage::cSpeedDegPerSec() const  { return ui.lineEdit_CSpeed->text().toDouble(); }
double SphericalParamPage::overScanDeg() const      { return ui.lineEdit_OverScan->text().toDouble(); }

// ---- 多纬线调度 ----
int SphericalParamPage::numLatitudes() const
{
    double s = latStepDeg();
    if (s <= 0) return 0;
    int n = (int)(std::fabs(latStartDeg() - latEndDeg()) / s) + 1;
    return std::max(1, n);
}

double SphericalParamPage::currentLatitude() const
{
    double dir = (latEndDeg() >= latStartDeg()) ? 1.0 : -1.0;
    return latStartDeg() + dir * m_currentLatIndex * latStepDeg();
}

bool SphericalParamPage::hasNext() const
{
    return m_currentLatIndex + 1 < numLatitudes();
}

double SphericalParamPage::peekNextLatitude() const
{
    double dir = (latEndDeg() >= latStartDeg()) ? 1.0 : -1.0;
    return latStartDeg() + dir * (m_currentLatIndex + 1) * latStepDeg();
}

QString SphericalParamPage::buildPositioningScriptToNext() const
{
    if (!hasNext()) return QString();
    return PmacScriptGen::generateSphericalPositioningScript(
        sphereRadiusMm(), currentLatitude(), peekNextLatitude(), isConvex());
}

void SphericalParamPage::advance()
{
    if (m_currentLatIndex + 1 < numLatitudes()) {
        ++m_currentLatIndex;
        refreshProgressLabel();
    }
}

void SphericalParamPage::resetIndex()
{
    m_currentLatIndex = 0;
    refreshProgressLabel();
    ui.lbl_Summary->setText(summary());
    emit paramChanged();
}

void SphericalParamPage::refreshProgressLabel()
{
    int n = numLatitudes();
    if (n <= 0) {
        ui.lbl_Progress->setText(QStringLiteral("进度：— / —（请填写参数）"));
        return;
    }
    ui.lbl_Progress->setText(QStringLiteral("进度：%1 / %2（当前纬度 φ = %3°）")
        .arg(m_currentLatIndex + 1).arg(n)
        .arg(currentLatitude(), 0, 'f', 2));
}

// ---- validate / build ----
bool SphericalParamPage::validate(QString& errMsg)
{
    double R = sphereRadiusMm();
    double a = latStartDeg(), b = latEndDeg(), s = latStepDeg();
    double dens = densityPerDeg(), w = cSpeedDegPerSec();
    if (R <= 0)    { errMsg = QStringLiteral("球面半径必须 > 0"); return false; }
    if (s <= 0)    { errMsg = QStringLiteral("纬度间隔必须 > 0"); return false; }
    if (dens <= 0) { errMsg = QStringLiteral("触发密度必须 > 0"); return false; }
    if (w <= 0)    { errMsg = QStringLiteral("C 转速必须 > 0"); return false; }

    // 极点 / 死区检查（针对扫描范围内每条纬线）
    double maxLat = std::max(std::fabs(a), std::fabs(b));
    if (std::fabs(maxLat - POLE_DEG) < 1e-6 || maxLat >= POLE_DEG) {
        errMsg = QStringLiteral("极点拓扑回避：拒绝 %1° 处的纬线扫描。").arg(POLE_DEG);
        return false;
    }
    double minLat = std::min(std::fabs(a), std::fabs(b));
    if (minLat < EQUATOR_DEAD) {
        errMsg = QStringLiteral("倾角死区：拒绝 |纬度| < %1° 范围（测头最大可测倾角约 40°）。")
                    .arg(EQUATOR_DEAD);
        return false;
    }
    // 频率防溢出
    double f = w * dens;
    if (f > F_MAX_HZ) {
        errMsg = QStringLiteral("预计触发频率 %1 Hz 超过 %2 Hz。").arg(f, 0, 'f', 1).arg(F_MAX_HZ);
        return false;
    }
    return true;
}

QString SphericalParamPage::buildScript()
{
    return PmacScriptGen::generateSphericalLatScript(
        sphereRadiusMm(), currentLatitude(),
        densityPerDeg(), cSpeedDegPerSec(), overScanDeg());
}

QString SphericalParamPage::summary()
{
    double a = latStartDeg(), b = latEndDeg(), s = latStepDeg();
    double dens = densityPerDeg(), w = cSpeedDegPerSec();
    if (s <= 0 || dens <= 0 || w <= 0)
        return QStringLiteral("—（请填写完整参数）");

    int    nLines = numLatitudes();
    double f = w * dens;
    double tPerLine = (360.0 + 2.0 * overScanDeg()) / w;
    double tTotal   = tPerLine * nLines;
    long   nPts     = (long)(360.0 * dens) * nLines;

    QString warn;
    if (f > F_MAX_HZ)            warn = QStringLiteral("<font color='red'> [超频]</font>");
    else if (std::min(std::fabs(a),std::fabs(b)) < EQUATOR_DEAD)
        warn = QStringLiteral("<font color='red'> [赤道死区]</font>");

    return QStringLiteral("条数: %1 | 单条耗时: %2 s | 总时: %3 s | 总点数: %4%5")
        .arg(nLines).arg(tPerLine, 0, 'f', 1).arg(tTotal, 0, 'f', 1).arg(nPts).arg(warn);
}

// ---- QSettings ----
void SphericalParamPage::saveSettings(QSettings& s)
{
    s.beginGroup("Spherical");
    s.setValue("convex",   ui.rb_Convex->isChecked());
    s.setValue("R",        ui.lineEdit_SphereR->text());
    s.setValue("latStart", ui.lineEdit_LatStart->text());
    s.setValue("latEnd",   ui.lineEdit_LatEnd->text());
    s.setValue("latStep",  ui.lineEdit_LatStep->text());
    s.setValue("density",  ui.lineEdit_Density->text());
    s.setValue("cSpeed",   ui.lineEdit_CSpeed->text());
    s.setValue("overScan", ui.lineEdit_OverScan->text());
    s.endGroup();
}

void SphericalParamPage::loadSettings(QSettings& s)
{
    s.beginGroup("Spherical");
    bool convex = s.value("convex", true).toBool();
    ui.rb_Convex->setChecked(convex);
    ui.rb_Concave->setChecked(!convex);
    ui.lineEdit_SphereR->setText( s.value("R",        "50").toString());
    ui.lineEdit_LatStart->setText(s.value("latStart", "80").toString());
    ui.lineEdit_LatEnd->setText(  s.value("latEnd",   "50").toString());
    ui.lineEdit_LatStep->setText( s.value("latStep",  "5").toString());
    ui.lineEdit_Density->setText( s.value("density",  "10").toString());
    ui.lineEdit_CSpeed->setText(  s.value("cSpeed",   "100").toString());
    ui.lineEdit_OverScan->setText(s.value("overScan", "5").toString());
    s.endGroup();
    m_currentLatIndex = 0;
    onAnyChanged();
}
