// ============================================================================
// AsphericMath.h
// 非球面光学曲面数学工具库
// ============================================================================
// 纯数学函数库，不依赖 Qt 或任何 UI 框架，可直接移植到其他项目。
//
// 公式体系（非球面方程）：
//   Z(r) = c*r^2/(1+sqrt(1-(1+k)*c^2*r^2)) + Σ A2n*r^(2n)  [n=2..6]
//   其中 c = 1/R（曲率），k（圆锥系数），A4~A12（高次项系数）
// ============================================================================

#pragma once

namespace AsphericMath {
    // 计算非球面 Z 值
    // r_mm: 当前半径(mm), R_mm: 曲率半径(mm)
    double calcAsphericZ(double r_mm, double R_mm, double k,
        double A4, double A6, double A8, double A10, double A12);

    // 根据斜率约束计算安全步距
    // max_z_change_um: 每步最大 Z 变化量(um)
    double calcSafeStepSize(double r_mm, double R_mm, double k,
        double A4, double A6, double A8, double A10, double A12,
        double max_z_change_um = 100.0);
}
