#pragma once
#include <QString>

// PMAC 扫描模式
enum class ScanMode {
    LinearFollowX  = 0,  // 直线跟随 X 轴
    SpiralFollowC  = 2,  // 螺旋线跟随 C 轴
    SphericalLat   = 3,  // 球面纬线扫描
    AsphericMeridian = 4 // 非球面母线扫描
};

class PmacScriptGen
{
public:
    // 模式 0: 直线跟随 X 轴
    static QString generateLinearScript(int targetGate = 1, int targetChan = 0);

    // 模式 2: 螺旋线跟随 C 轴
    static QString generateSpiralScript(double pitchUm,
        int targetGate = 1, int targetChan = 0);

    // 模式 3: 球面纬线扫描
    static QString generateSphericalLatScript(int targetGate = 1, int targetChan = 0);

    // 模式 4: 非球面母线扫描 (Z 轴跟随)
    static QString generateAsphericMeridianScript(
        double startRadiusUm, double scanRangeUm,
        double R_mm, double k,
        double A4, double A6, double A8, double A10, double A12,
        bool followZ,
        int targetGate = 1, int targetChan = 0);

    // 急停脚本
    static QString emergencyStop();
};
