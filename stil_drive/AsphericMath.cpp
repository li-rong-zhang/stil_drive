#include "AsphericMath.h"
#include <cmath>

#define _USE_MATH_DEFINES

double AsphericMath::calcAsphericZ(double r_mm, double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12)
{
    if (R_mm <= 0) return 0;

    double c = 1.0 / R_mm;
    double r2 = r_mm * r_mm;
    double c2 = c * c;

    double sqrt_term = 1.0 - (1.0 + k) * c2 * r2;
    double z_conic = 0.0;

    if (sqrt_term > 1e-10) {
        z_conic = (c * r2) / (1.0 + std::sqrt(sqrt_term));
    }

    double z_high = A4 * std::pow(r_mm, 4)
        + A6 * std::pow(r_mm, 6)
        + A8 * std::pow(r_mm, 8)
        + A10 * std::pow(r_mm, 10)
        + A12 * std::pow(r_mm, 12);

    return z_conic * 1000.0 + z_high;
}

double AsphericMath::calcSafeStepSize(double r_mm, double R_mm, double k,
    double A4, double A6, double A8, double A10, double A12,
    double max_z_change_um)
{
    if (R_mm <= 0) return 10.0;

    double c = 1.0 / R_mm;
    double c2 = c * c;
    double r2 = r_mm * r_mm;
    double r3 = r2 * r_mm;
    double r5 = r3 * r2;
    double r7 = r5 * r2;
    double r9 = r7 * r2;

    double sqrt_term = 1.0 - (1.0 + k) * c2 * r2;
    double dZ_conic_dr = 0.0;

    if (sqrt_term > 1e-10) {
        double S = std::sqrt(sqrt_term);
        dZ_conic_dr = (2.0 * c * r_mm * (1.0 + S) - c * r3 * (1.0 + k) * c2 * r_mm / S)
            / ((1.0 + S) * (1.0 + S));
    }

    double dZ_high_dr = 4.0 * A4 * r3
        + 6.0 * A6 * r5
        + 8.0 * A8 * r7
        + 10.0 * A10 * r9
        + 12.0 * A12 * r2 * r9;

    double total_slope = std::abs((dZ_conic_dr * 1000.0 + dZ_high_dr));
    if (total_slope < 0.001) total_slope = 0.001;

    double safe_step_um = max_z_change_um / total_slope;
    if (safe_step_um < 0.5) safe_step_um = 0.5;
    if (safe_step_um > 50.0) safe_step_um = 50.0;

    return safe_step_um;
}
