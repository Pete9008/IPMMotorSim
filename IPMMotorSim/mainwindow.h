/*
 * This file is part of the IPMMotorSim project
 *
 * Copyright (C) 2022 Pete9008 <openinverter.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "datagraph.h"
#include "motormodel.h"



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

private:
    void runFor(int num_steps);
    void calcFluxLinkage(void);

    DataGraph motorGraph;
    DataGraph simulationGraph;
    DataGraph controllerGraph;
    DataGraph debugGraph;
    MotorModel *motor;
    double m_time;
    uint32_t m_old_time;
    uint32_t m_old_ms_time;

    double m_wheelSize;
    double m_vehicleWeight;
    double m_gearRatio;
    double m_drag;
    double m_Lq;
    double m_Ld;
    double m_Rs;
    double m_Poles;
    double m_fluxLinkage;
    double m_fluxLinkageDelta;
    double m_syncdelay;
    double m_samplingPoint;

    double m_timestep;
    double m_Vdc;


public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_vehicleWeight_editingFinished();

    void on_pbRunFor_clicked();

    void on_pbStep_clicked();

    void on_wheelSize_editingFinished();

    void on_gearRatio_editingFinished();

    void on_Vdc_editingFinished();

    void on_Lq_editingFinished();

    void on_Ld_editingFinished();

    void on_FluxLinkage_editingFinished();

    void on_Drag_editingFinished();

    void on_LoopFreq_editingFinished();

    void on_pbRunFor10s_clicked();

    void on_pbRunFor1s_clicked();

    void on_pbRunFor100ms_clicked();

    void on_pbRunFor10ms_clicked();

    void on_pbRestart_clicked();

    void on_Rs_editingFinished();

    void on_Poles_editingFinished();

    void on_BaseFreq_editingFinished();

    void on_BaseVolts_editingFinished();

    void on_torqueDemand_editingFinished();

    void on_throttleCurrent_editingFinished();

    void on_opMode_editingFinished();

    void on_direction_editingFinished();

    void on_IqManual_editingFinished();

    void on_IdManual_editingFinished();

    void on_CurrentKp_editingFinished();

    void on_CurrentKi_editingFinished();

    void on_FluxLinkageDelta_editingFinished();

    void on_SyncAdv_editingFinished();

    void on_LqMinusLd_editingFinished();

    void on_SyncDelay_editingFinished();

    void on_FWStart_editingFinished();

    void on_FreqMax_editingFinished();

    void on_FWKp_editingFinished();

    void on_FWKi_editingFinished();

    void on_VLimKp_editingFinished();

    void on_VLimKi_editingFinished();

    void on_SamplingPoint_editingFinished();

private:
    Ui::MainWindow *ui;
    void closeEvent(QCloseEvent *bar);
};

#endif // MAINWINDOW_H
