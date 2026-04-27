#pragma once
#include <QString>
#include <QVector>
#include <QTextStream>

struct ReductionResult {
    QVector<double> xData;
    double currentX = 0.0;
    double windowWidth = 1000.0;
    QString unit;
};

class DataReducer
{
public:
    static ReductionResult reduceMode0(
        const QVector<double>& alts,
        const QVector<double>& ints,
        qint64 startPoint,
        QTextStream* csvStream);

    static ReductionResult reduceMode2(
        const QVector<double>& alts,
        const QVector<double>& ints,
        qint64 startPoint,
        double startRadiusUm,
        double pitchPerRevUm,
        QTextStream* csvStream);

    static ReductionResult reduceMode3(
        const QVector<double>& alts,
        const QVector<double>& ints,
        qint64 startPoint,
        QTextStream* csvStream);

    static ReductionResult reduceMode4(
        const QVector<double>& alts,
        const QVector<double>& ints,
        qint64 startPoint,
        double startRadiusUm,
        double scanRangeUm,
        double R_mm, double k,
        double A4, double A6, double A8, double A10, double A12,
        QTextStream* csvStream);
};
