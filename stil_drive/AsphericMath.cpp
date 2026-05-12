// ============================================================================
// AsphericMath.cpp
// 非球面光学曲面数学工具库
// ============================================================================
// 包含圆锥系数曲面（Conic）方程和高次项（Aspheric）方程的纯数学计算，
// 不涉及任何 UI、硬件或 Qt 依赖，可独立复用。
//
// 公式说明：
//   Z(r) = Z_conic(r) + Z_high(r)
//   Z_conic = c*r^2 / (1 + sqrt(1 - (1+k)*c^2*r^2))     [单位：mm → 转为 µm]
//   Z_high = A4*r^4 + A6*r^6 + A8*r^8 + A10*r^10 + A12*r^12  [单位：µm]
//   c = 1/R 为曲率，k 为圆锥系数（k=0 时退化为球面）
// ============================================================================

#include "AsphericMath.h"
#include <cmath>

// ============================================================================
// calcAsphericZ
// 计算给定半径处的非球面 Z 值（理论曲面高度）
// 参数：
//   r_mm      当前点半径 (mm)
//   R_mm      顶点曲率半径 (mm)
//   k         圆锥系数
//   A4~A12    4/6/8/10/12 次非球面系数 (µm)
// 返回：
//   Z 值 (µm)
// ============================================================================
double AsphericMath::calcAsphericZ(double r_mm, double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12)
{
    if (R_mm == 0.0) return 0;

    double c = 1.0 / R_mm;           // 曲率
    double r2 = r_mm * r_mm;
    double c2 = c * c;

    // --- 圆锥面部分（计算 Z_conic）---
    double sqrt_term = 1.0 - (1.0 + k) * c2 * r2;
    double z_conic = 0.0;

    if (sqrt_term > 1e-10) {
        // 稳定计算公式，避免奇点
        z_conic = (c * r2) / (1.0 + std::sqrt(sqrt_term));
    }

    // --- 高次项部分（直接加和，单位 µm）---
    double z_high = A4 * std::pow(r_mm, 4)
        + A6 * std::pow(r_mm, 6)
        + A8 * std::pow(r_mm, 8)
        + A10 * std::pow(r_mm, 10)
        + A12 * std::pow(r_mm, 12);

    // 圆锥面部分单位为 mm，乘以 1000 转为 µm
    return z_conic * 1000.0 + z_high;
}

// ============================================================================
// calcSafeStepSize
// 根据当前半径处的斜率约束，自动计算安全的径向扫描步距
// 原理：|dZ/dr| 越大，说明曲面越陡，必须用更小的步距避免漏检
// 参数：
//   r_mm            当前半径 (mm)
//   R_mm, k, A4~A12 同上
//   max_z_change_um 每步最大允许 Z 变化量（µm），默认 100 µm
// 返回：
//   安全步距 (µm)，范围限制在 [0.5, 50] µm
// ============================================================================
double AsphericMath::calcSafeStepSize(double r_mm, double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12,
    double max_z_change_um)
{
    if (R_mm == 0.0) return 10.0;

    double c = 1.0 / R_mm;
    double c2 = c * c;
    double r2 = r_mm * r_mm;
    double r3 = r2 * r_mm;
    double r5 = r3 * r2;
    double r7 = r5 * r2;
    double r9 = r7 * r2;

    // --- 计算 dZ_conic/dr ---
    double sqrt_term = 1.0 - (1.0 + k) * c2 * r2;
    double dZ_conic_dr = 0.0;

    if (sqrt_term > 1e-10) {
        double S = std::sqrt(sqrt_term);
        // Z_conic = c*r^2 / (1+S)
        // d/dr[...] 解析导数
        dZ_conic_dr = (2.0 * c * r_mm * (1.0 + S) - c * r3 * (1.0 + k) * c2 * r_mm / S)
            / ((1.0 + S) * (1.0 + S));
    }

    // --- 计算 dZ_high/dr ---
    double dZ_high_dr = 4.0 * A4 * r3
        + 6.0 * A6 * r5
        + 8.0 * A8 * r7
        + 10.0 * A10 * r9
        + 12.0 * A12 * r2 * r9;

    // 总斜率（单位：µm/mm = µm/µm * 1000）
    double total_slope = std::abs(dZ_conic_dr * 1000.0 + dZ_high_dr);
    if (total_slope < 0.001) total_slope = 0.001;  // 防止除零

    // 安全步距 = 最大允许 Z 变化 / 斜率
    double safe_step_um = max_z_change_um / total_slope;

    // 限制合理范围
    if (safe_step_um < 0.5)  safe_step_um = 0.5;   // 最小步距
    if (safe_step_um > 50.0) safe_step_um = 50.0; // 最大步距

    return safe_step_um;
}
