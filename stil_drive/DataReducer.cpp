// ============================================================================
// DataReducer.cpp
// 扫描数据坐标还原模块
// ============================================================================
// 根据当前扫描模式（X轴直线 / C+X螺旋 / C纬度 / 非球面母线），
// 将接收到的"高度+强度"脉冲序列还原为真实的空间坐标 (X, Y, Z)，
// 并实时写入 CSV 文件供后期分析。
//
// 坐标体系说明：
//   - 直线模式：     X = 脉冲序号 × 1 µm，Y = 0
//   - 螺旋模式：     极坐标 (r, θ) → 直角坐标 (X, Y)
//   - 纬度模式：     X = R·cos(θ)，Y = R·sin(θ)，θ = 脉冲序号 × 0.1°
//   - 非球面母线模式：X = 半径 (µm)，Y = 0，理论 Z 由 AsphericMath 计算
// ============================================================================

#define _USE_MATH_DEFINES
#include <cmath>

#include "DataReducer.h"
#include "AsphericMath.h"

// 用 std::acos(-1.0) 在运行时计算 π，结果即为 double 能表示的最精确 π 值
// 与 M_PI 宏（3.14159265358979323846）的二进制表示完全相同，但写法更接近数学定义
static const double kPi = std::acos(-1.0);

// ============================================================================
// reduceMode0：X 轴直线跟随扫描
// 每收到一个触发脉冲对应 X 轴移动 1 µm
// ============================================================================
ReductionResult DataReducer::reduceMode0(
    const QVector<double>& alts,
    const QVector<double>&,
    qint64 startPoint,
    QTextStream* csvStream)
{
    ReductionResult r;
    r.unit = "um";
    r.windowWidth = 1000.0;
    int dataSize = alts.size();
    r.xData.resize(dataSize);

    double pulse_pitch_um = 1.0;  // 触发间隔 1 µm
    double start_X = 0.0;

    for (int i = 0; i < dataSize; i++) {
        // 累计脉冲数 × 步距 = 当前 X 坐标
        double simulated_X = start_X + (startPoint + i) * pulse_pitch_um;
        r.xData[i] = simulated_X;

        if (csvStream) {
            *csvStream << simulated_X << ",0.0," << alts[i] << "\n";
        }
        r.currentX = simulated_X;
    }
    return r;
}

// ============================================================================
// reduceMode2：C+X 轴螺旋线扫描
// C 轴按配置的触发密度触发，X 轴同步径向进给
// 扫描路径：从外向内螺旋逼近圆心
// ============================================================================
ReductionResult DataReducer::reduceMode2(
    const QVector<double>& alts,
    const QVector<double>& ints,
    qint64 startPoint,
    double startRadiusUm,
    double pitchPerRevUm,
    double densityDegPerPt,
    double overCenterMm,
    QTextStream* csvStream)
{
    ReductionResult r;
    r.unit = "度";
    r.windowWidth = 360.0;
    int dataSize = alts.size();
    r.xData.resize(dataSize);

    // 使用实际配置的触发密度（°/pt）
    double pulse_angle_deg = densityDegPerPt;
    // 过圆心偏移量（µm）
    double r_over_um = overCenterMm * 1000.0;

    // 计算到达圆心所需总角度（用于自动停止判断）
    double total_angle_needed = ((startRadiusUm + r_over_um) / pitchPerRevUm) * 360.0;
    double current_total_angle = startPoint * pulse_angle_deg;

    if (current_total_angle >= total_angle_needed) {
        // 扫描已达圆心，返回终止状态
        r.currentX = current_total_angle;
        return r;
    }

    for (int i = 0; i < dataSize; i++) {
        double current_angle_deg = (startPoint + i) * pulse_angle_deg;
        double revolutions = current_angle_deg / 360.0;
        // 半径随圈数递减（螺旋逼近圆心）
        double current_radius = startRadiusUm - (revolutions * pitchPerRevUm);

        // 极坐标 → 直角坐标
        double angle_rad = current_angle_deg * kPi / 180.0;
        double simulated_X = current_radius * std::cos(angle_rad);
        double simulated_Y = current_radius * std::sin(angle_rad);

        r.xData[i] = current_angle_deg;  // 横坐标显示角度便于观察波形

        if (csvStream) {
            *csvStream << simulated_X << "," << simulated_Y << "," << alts[i] << "," << ints[i] << "\n";
        }
        r.currentX = current_angle_deg;
    }
    return r;
}

// ============================================================================
// reduceMode3：C 轴球面纬度圆弧扫描
// C 轴单纯旋转，X 轴固定，用于测量单条纬度圆弧轮廓
// 扫描范围 370°（360° 主体 + 10° 重叠区）
// ============================================================================
ReductionResult DataReducer::reduceMode3(
    const QVector<double>& alts,
    const QVector<double>& ints,
    qint64 startPoint,
    QTextStream* csvStream)
{
    ReductionResult r;
    r.unit = "度";
    r.windowWidth = 370.0;  // 370° 视窗，容纳完整扫描
    int dataSize = alts.size();
    r.xData.resize(dataSize);

    // 底层 PMAC CompAdd=40960，对应 0.1°
    double pulse_angle_deg = 0.1;

    for (int i = 0; i < dataSize; i++) {
        double current_angle_deg = (startPoint + i) * pulse_angle_deg;
        r.xData[i] = current_angle_deg;

        if (csvStream) {
            *csvStream << current_angle_deg << ",0.0," << alts[i] << "," << ints[i] << "\n";
        }
        r.currentX = current_angle_deg;
    }
    return r;
}

// ============================================================================
// reduceMode4：非球面母线扫描
// X 轴按安全步距径向移动，实测 Z 与理论非球面 Z 实时对比
// 若勾选"跟随 Z 轴"，则 Z 轴同步跟踪非球面轨迹
// ============================================================================
ReductionResult DataReducer::reduceMode4(
    const QVector<double>& alts,
    const QVector<double>& ints,
    qint64 startPoint,
    double startRadiusUm,
    double scanRangeUm,
    double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12,
    QTextStream* csvStream)
{
    ReductionResult r;
    r.unit = "µm";
    r.windowWidth = scanRangeUm;
    int dataSize = alts.size();
    r.xData.resize(dataSize);

    if (R_mm <= 0) R_mm = 100.0;

    // 根据斜率计算安全步距（避免在陡峭区域漏检）
    double step_um = AsphericMath::calcSafeStepSize(
        startRadiusUm / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

    for (int i = 0; i < dataSize; i++) {
        double current_r_um = startRadiusUm + (startPoint + i) * step_um;
        double current_r_mm = current_r_um / 1000.0;

        // 计算理论非球面 Z 值
        double z_theory = AsphericMath::calcAsphericZ(current_r_mm, R_mm, k, A4, A6, A8, A10, A12);
        // 实测偏差 = 实测高度 - 理论高度
        double z_deviation = alts[i] - z_theory;

        r.xData[i] = current_r_um;

        if (csvStream) {
            // CSV 增加理论 Z 和偏差列，便于后期面形误差分析
            *csvStream << current_r_um << ",0.0," << alts[i] << "," << z_theory << "," << z_deviation << "," << ints[i] << "\n";
        }
        r.currentX = current_r_um;
    }
    return r;
}
