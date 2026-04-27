#define _USE_MATH_DEFINES
#include <cmath>

#include "DataReducer.h"
#include "AsphericMath.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

    double pulse_pitch_um = 1.0;
    double start_X = 0.0;

    for (int i = 0; i < dataSize; i++) {
        double simulated_X = start_X + (startPoint + i) * pulse_pitch_um;
        r.xData[i] = simulated_X;

        if (csvStream) {
            *csvStream << simulated_X << ",0.0," << alts[i] << "\n";
        }
        r.currentX = simulated_X;
    }
    return r;
}

ReductionResult DataReducer::reduceMode2(
    const QVector<double>& alts,
    const QVector<double>& ints,
    qint64 startPoint,
    double startRadiusUm,
    double pitchPerRevUm,
    QTextStream* csvStream)
{
    ReductionResult r;
    r.unit = "度";
    r.windowWidth = 360.0;
    int dataSize = alts.size();
    r.xData.resize(dataSize);

    double pulse_angle_deg = 0.5;
    double r_over_um = 1000.0;

    double total_angle_needed = ((startRadiusUm + r_over_um) / pitchPerRevUm) * 360.0;
    double current_total_angle = startPoint * pulse_angle_deg;

    if (current_total_angle >= total_angle_needed) {
        r.currentX = current_total_angle;
        return r;
    }

    for (int i = 0; i < dataSize; i++) {
        double current_angle_deg = (startPoint + i) * pulse_angle_deg;
        double revolutions = current_angle_deg / 360.0;
        double current_radius = startRadiusUm - (revolutions * pitchPerRevUm);

        double angle_rad = current_angle_deg * M_PI / 180.0;
        double simulated_X = current_radius * std::cos(angle_rad);
        double simulated_Y = current_radius * std::sin(angle_rad);

        r.xData[i] = current_angle_deg;

        if (csvStream) {
            *csvStream << simulated_X << "," << simulated_Y << "," << alts[i] << "," << ints[i] << "\n";
        }
        r.currentX = current_angle_deg;
    }
    return r;
}

ReductionResult DataReducer::reduceMode3(
    const QVector<double>& alts,
    const QVector<double>& ints,
    qint64 startPoint,
    QTextStream* csvStream)
{
    ReductionResult r;
    r.unit = "度";
    r.windowWidth = 370.0;
    int dataSize = alts.size();
    r.xData.resize(dataSize);

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

    double step_um = AsphericMath::calcSafeStepSize(
        startRadiusUm / 1000.0, R_mm, k, A4, A6, A8, A10, A12);

    for (int i = 0; i < dataSize; i++) {
        double current_r_um = startRadiusUm + (startPoint + i) * step_um;
        double current_r_mm = current_r_um / 1000.0;
        double z_theory = AsphericMath::calcAsphericZ(current_r_mm, R_mm, k, A4, A6, A8, A10, A12);
        double z_deviation = alts[i] - z_theory;

        r.xData[i] = current_r_um;

        if (csvStream) {
            *csvStream << current_r_um << ",0.0," << alts[i] << "," << z_theory << "," << z_deviation << "," << ints[i] << "\n";
        }
        r.currentX = current_r_um;
    }
    return r;
}
