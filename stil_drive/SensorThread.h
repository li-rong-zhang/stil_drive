#pragma once
#include <QThread>
#include <QVector>
#include <windows.h>

#ifndef DLL_CHR_API
#define DLL_CHR_API __declspec(dllimport)
#endif

#include "MchrDefine.h"
#include "MchrType.h"
#include "MchrError.h"
#include "Mchr.h"

class SensorThread : public QThread
{
    Q_OBJECT

public:
    explicit SensorThread(MCHR_ID sensorID, QObject* parent = nullptr);
    ~SensorThread();

    void stopAcquisition();

signals:
    void dataReady(QVector<double> altitudes, QVector<double> intensities);
    void errorOccurred(QString errorMsg);

protected:
    void run() override;

private:
    MCHR_ID m_sensorID;
    bool m_isRunning;
};
