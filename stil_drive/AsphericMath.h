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
