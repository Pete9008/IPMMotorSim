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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QtMath>
#include "pwmgeneration.h"
#include "foc.h"
#include "params.h"
#include "inc_encoder.h"

#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#include "anain.h"

//PWM graph
#define IA 1
#define IB 2
#define IC 3
#define IQ 4
#define ID 5

//Simulation graph
#define M_RPM 1
#define M_MOTOR_POS 2
#define M_CONT_POS 3

//Controller graph
#define VA 1
#define VB 2
#define VC 3
#define VQ 4
#define VD 5
#define C_IQ 6
#define C_ID 7
#define C_IFW 8
#define C_IVLIM 9

#define TWO_PI_CONT 65536

//c++ test stubs globals
extern volatile uint16_t g_input_angle;
extern volatile double g_il1_input;
extern volatile double g_il2_input;

// C test stubs globals
extern volatile bool disablePWM;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ANA_IN_CONFIGURE(ANA_IN_LIST);

    m_wheelSize = ui->wheelSize->text().toDouble();
    m_vehicleWeight = ui->vehicleWeight->text().toDouble();
    m_gearRatio = ui->gearRatio->text().toDouble();
    m_drag = ui->Drag->text().toDouble();
    m_Lq = ui->Lq->text().toDouble()/1000; //entered in mH
    m_Ld = ui->Ld->text().toDouble()/1000; //entered in mH
    m_Rs = ui->Rs->text().toDouble();
    m_Poles = ui->Poles->text().toDouble();
    m_fluxLinkage = ui->FluxLinkage->text().toDouble()/1000; //entered in mWeber
    m_fluxLinkageDelta = ui->FluxLinkageDelta->text().toDouble();
    m_syncdelay = ui->SyncDelay->text().toDouble()/1000000; //entered in uS
    m_samplingPoint = ui->SamplingPoint->text().toDouble()/100.0; //entered in %

    m_timestep = 1.0 / ui->LoopFreq->text().toDouble();
    m_Vdc = ui->Vdc->text().toDouble();

    calcFluxLinkage();

    motor = new MotorModel(m_wheelSize,m_gearRatio,m_drag,m_vehicleWeight,m_Lq,m_Ld,m_Rs,m_Poles,m_fluxLinkage,m_timestep,m_fluxLinkageDelta,m_syncdelay,m_samplingPoint);
    m_time = 0;
    m_old_time = 0;
    m_old_ms_time = 0;

    motorGraph.setWindowTitle("Motor Currents");
    motorGraph.addSeries("Ia (A)", IA);
    motorGraph.addSeries("Ib (A)", IB);
    motorGraph.addSeries("Ic (A)", IC);
    motorGraph.addSeries("Iq (A)", IQ);
    motorGraph.setColour(Qt::blue, IQ);
    motorGraph.addSeries("Id (A)", ID);
    motorGraph.setColour(Qt::red, ID);
    motorGraph.show();

    simulationGraph.setWindowTitle("Simulation Data");
    simulationGraph.addSeries("Motor Position (degrees)", M_MOTOR_POS);
    simulationGraph.setOpacity(0.25, M_MOTOR_POS);
    simulationGraph.addSeries("Controller Position (degrees)", M_CONT_POS);
    simulationGraph.setOpacity(0.25, M_CONT_POS);
    simulationGraph.addSeries("Motor Elec Speed (Hz)", M_RPM);
    simulationGraph.setColour(Qt::blue, M_RPM);
    simulationGraph.show();

    controllerGraph.setWindowTitle("Controller Voltages");
    controllerGraph.addSeries("Va (V)", VA);
    controllerGraph.addSeries("Vb (V)", VB);
    controllerGraph.addSeries("Vc (V)", VC);
    controllerGraph.addSeries("Vq (V)", VQ);
    controllerGraph.setColour(Qt::blue, VQ);
    controllerGraph.addSeries("Vd (V)", VD);
    controllerGraph.setColour(Qt::red, VD);
    controllerGraph.show();

    debugGraph.setWindowTitle("Controller Currents");
    debugGraph.addSeries("Iq (A)", C_IQ);
    debugGraph.setColour(Qt::blue, C_IQ);
    debugGraph.addSeries("Id (A)", C_ID);
    debugGraph.setColour(Qt::red, C_ID);
    debugGraph.addSeries("Ifw (A)", C_IFW);
    debugGraph.addSeries("Throttle Reduction (%)", C_IVLIM);
    debugGraph.show();    

    //following block copied from OpenInverter - probably not needed
    Param:SetInt(Param::version, 4); //backward compatibility

    if (Param::GetInt(Param::snsm) < 12)
        Param::SetInt(Param::snsm, Param::GetInt(Param::snsm) + 10); //upgrade parameter
    if (Param::Get(Param::offthrotregen) > 0)
        Param::Set(Param::offthrotregen, -Param::Get(Param::offthrotregen));


    Param::Change(Param::PARAM_LAST);
    Param::Change(Param::nodeid);

    //Param::Set(Param::lqminusld, FP_FROMFLT(ui->LqMinusLd->text().toFloat()));
    ui->LqMinusLd->setText(QString::number(Param::GetFloat(Param::lqminusld), 'f', 1));
    //Param::Set(Param::fluxlinkage, FP_FROMFLT(ui->FluxLinkage->text().toFloat()));
    ui->FluxLinkage->setText(QString::number(Param::GetInt(Param::fluxlinkage)));
    //Param::SetInt(Param::syncadv, ui->SyncAdv->text().toInt());
    ui->SyncAdv->setText(QString::number(Param::GetInt(Param::syncadv)));
    //Param::Set(Param::ffwstart, FP_FROMFLT(ui->FWStart->text().toFloat()));
    ui->FWStart->setText(QString::number(Param::GetFloat(Param::ffwstart), 'f', 1));
    //Param::Set(Param::fmax, FP_FROMFLT(ui->FreqMax->text().toFloat()));
    ui->FreqMax->setText(QString::number(Param::GetFloat(Param::fmax), 'f', 1));

    //Param::Set(Param::curkp, FP_FROMINT(ui->CurrentKp->text().toInt()));
    ui->CurrentKp->setText(QString::number(Param::GetInt(Param::curkp)));
    //Param::Set(Param::curki, FP_FROMINT(ui->CurrentKi->text().toInt()));
    ui->CurrentKi->setText(QString::number(Param::GetInt(Param::curki)));
    //Param::Set(Param::fwkp, FP_FROMINT(ui->FWKp->text().toInt()));
    ui->FWKp->setText(QString::number(Param::GetInt(Param::fwkp)));
    //Param::Set(Param::fwki, FP_FROMINT(ui->FWKi->text().toInt()));
    ui->FWKi->setText(QString::number(Param::GetInt(Param::fwki)));
    //Param::Set(Param::vlimkp, FP_FROMINT(ui->VLimKp->text().toInt()));
    ui->VLimKp->setText(QString::number(Param::GetInt(Param::vlimkp)));
    //Param::Set(Param::vlimki, FP_FROMINT(ui->VLimKi->text().toInt()));
    ui->VLimKi->setText(QString::number(Param::GetInt(Param::vlimki)));

    //Param::Set(Param::manualid, FP_FROMFLT(ui->IdManual->text().toFloat()));
    ui->IdManual->setText(QString::number(Param::GetFloat(Param::manualid), 'f', 1));
    //Param::Set(Param::manualiq, FP_FROMFLT(ui->IqManual->text().toFloat()));
    ui->IqManual->setText(QString::number(Param::GetFloat(Param::manualiq), 'f', 1));

    //Param::SetInt(Param::syncadv, ui->SyncAdv->text().toInt());
    ui->SyncAdv->setText(QString::number(Param::GetInt(Param::syncadv)));

    //Param::Set(Param::throtcur, FP_FROMFLT(ui->throttleCurrent->text().toFloat()));
    ui->throttleCurrent->setText(QString::number(Param::GetFloat(Param::throtcur), 'f', 1));

    PwmGeneration::SetOpmode(ui->opMode->text().toInt());
    Param::SetInt(Param::dir, ui->direction->text().toInt());

    //Param::Set(Param::polepairs, FP_FROMINT(ui->Poles->text().toInt()));
    ui->Poles->setText(QString::number(Param::GetInt(Param::polepairs)));
    //Param::Set(Param::throtcur, FP_FROMINT(ui->throttleCurrent->text().toInt()));
    ui->throttleCurrent->setText(QString::number(Param::GetInt(Param::throtcur)));

    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());

    //run for 1sec to complete motor init
    runFor(8789);
    on_pbRestart_clicked();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    motorGraph.hide();
    simulationGraph.hide();
    controllerGraph.hide();
    debugGraph.hide();
    QWidget::closeEvent(event);
}

void MainWindow::runFor(int num_steps)
{
    double Va = 0;
    double Vb = 0;
    double Vc = 0;
    double oldVa = 0;
    double oldVb = 0;
    double oldVc = 0;

    if(num_steps<0)
        return;

    QList<QPointF> listIa, listIb, listIc, listIq, listId;
    QList<QPointF> listMFreq, listMPos, listContMPos;
    QList<QPointF> listCVa, listCVb, listCVc, listCVq, listCVd, listCIq, listCId, listCifw, listCivlim;

    for(int i = 0;i<num_steps; i++)
    {
        //routines that need calling every 10ms
        if((uint32_t)(m_time*100) != m_old_time)
        {
            m_old_time = (uint32_t)(m_time*100);
            Encoder::UpdateRotorFrequency(100);
        }

        //routines that need calling every ms
        if((uint32_t)(m_time*1000) != m_old_ms_time)
        {
            m_old_ms_time = (uint32_t)(m_time*100);
#ifdef HAS_FW_CALL_FROM_MS_TASK
            PwmGeneration::runFWController();
#endif
        }        

        g_input_angle = (uint16_t)((motor->getElecPosition()*TWO_PI_CONT)/360.0);
        if(disablePWM)
        {
            g_il1_input = 0;
            g_il2_input = 0;
        }
        else
        {
            g_il1_input = (Param::GetFloat(Param::il1gain)*motor->getIaSamp());
            g_il2_input = (Param::GetFloat(Param::il2gain)*motor->getIbSamp());
        }

        PwmGeneration::Run();

        if(disablePWM)
        {
            Va = 0;
            Va = 0;
            Va = 0;
        }
        else
        {
            Va = (m_Vdc/65536) * (FOC::DutyCycles[0]-32768);
            Vb = (m_Vdc/65536) * (FOC::DutyCycles[1]-32768);
            Vc = (m_Vdc/65536) * (FOC::DutyCycles[2]-32768);
        }

        //one period delay to simulate slow timer reload in target hardware
        if(ui->ExtraCycleDelay->isChecked())
            motor->Step(oldVa,oldVb,oldVc);
        else
            motor->Step(Va,Vb,Vc);
        oldVa = Va;
        oldVb = Vb;
        oldVb = Vb;


        //motor->Step(0,0,0);
        listIa.append(QPointF(m_time, motor->getIaSamp()));
        listIb.append(QPointF(m_time, motor->getIbSamp()));
        listIc.append(QPointF(m_time, motor->getIcSamp()));
        listIq.append(QPointF(m_time, motor->getIq()));
        listId.append(QPointF(m_time, motor->getId()));

        listMFreq.append(QPointF(m_time, (motor->getMotorFreq()*m_Poles)));
        listMPos.append(QPointF(m_time, motor->getMotorPosition()));
        listContMPos.append(QPointF(m_time, (360.0 * PwmGeneration::GetAngle())/TWO_PI_CONT));

        listCVa.append(QPointF(m_time, Va));
        listCVb.append(QPointF(m_time, Vb));
        listCVc.append(QPointF(m_time, Vc));
        listCVq.append(QPointF(m_time, (m_Vdc/65536) * Param::GetFloat(Param::uq)));
        listCVd.append(QPointF(m_time, (m_Vdc/65536) * Param::GetFloat(Param::ud)));

        listCIq.append(QPointF(m_time, Param::GetFloat(Param::iq)));
        listCId.append(QPointF(m_time, Param::GetFloat(Param::id)));

        listCifw.append(QPointF(m_time, Param::GetFloat(Param::ifw)));
        listCivlim.append(QPointF(m_time, Param::GetFloat(Param::ivlim)));

        m_time += m_timestep;
    }

    motorGraph.addDataPoints(listIa, IA);
    motorGraph.addDataPoints(listIb, IB);
    motorGraph.addDataPoints(listIc, IC);
    motorGraph.addDataPoints(listIq, IQ);
    motorGraph.addDataPoints(listId, ID);
    simulationGraph.addDataPoints(listMFreq, M_RPM);
    simulationGraph.addDataPoints(listMPos, M_MOTOR_POS);
    simulationGraph.addDataPoints(listContMPos, M_CONT_POS);

    controllerGraph.addDataPoints(listCVa, VA);
    controllerGraph.addDataPoints(listCVb, VB);
    controllerGraph.addDataPoints(listCVc, VC);
    controllerGraph.addDataPoints(listCVq, VQ);
    controllerGraph.addDataPoints(listCVd, VD);

    debugGraph.addDataPoints(listCIq, C_IQ);
    debugGraph.addDataPoints(listCId, C_ID);
    debugGraph.addDataPoints(listCifw, C_IFW);
    debugGraph.addDataPoints(listCivlim, C_IVLIM);

    motorGraph.updateGraph();
    simulationGraph.updateGraph();
    controllerGraph.updateGraph();
    debugGraph.updateGraph();
}

void MainWindow::on_vehicleWeight_editingFinished()
{
    m_vehicleWeight = ui->vehicleWeight->text().toDouble();
    motor->setVehicleMass(m_vehicleWeight);
}

void MainWindow::on_wheelSize_editingFinished()
{
    m_wheelSize = ui->wheelSize->text().toDouble();
    motor->setWheelSize(m_wheelSize);
}

void MainWindow::on_gearRatio_editingFinished()
{
    m_gearRatio = ui->gearRatio->text().toDouble();
    motor->setGboxRatio(m_gearRatio);
}

void MainWindow::on_Vdc_editingFinished()
{
    m_Vdc = ui->Vdc->text().toDouble();
}

void MainWindow::on_Lq_editingFinished()
{
    m_Lq = ui->Lq->text().toDouble()/1000;
    motor->setLq(m_Lq);
}

void MainWindow::on_Ld_editingFinished()
{
    m_Ld = ui->Ld->text().toDouble()/1000;
    motor->setLd(m_Ld);
}

void MainWindow::on_Rs_editingFinished()
{
    m_Rs = ui->Rs->text().toDouble();
    motor->setRs(m_Rs);
}

void MainWindow::on_Poles_editingFinished()
{
    m_Poles = ui->Poles->text().toDouble();
    Param::Set(Param::polepairs, FP_FROMINT(ui->Poles->text().toInt()));
    motor->setPoles(m_Poles);
    calcFluxLinkage();
}

void MainWindow::on_FluxLinkage_editingFinished()
{
    m_fluxLinkage = ui->FluxLinkage->text().toDouble()/1000;
    Param::Set(Param::fluxlinkage, FP_FROMFLT(ui->FluxLinkage->text().toFloat()));
    motor->setFluxLinkage(m_fluxLinkage);
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat()); //make sure is recalculated
}

void MainWindow::on_Drag_editingFinished()
{
    m_drag = ui->Drag->text().toDouble();
    motor->setDrag(m_drag);
}

void MainWindow::on_LoopFreq_editingFinished()
{
    m_timestep = 1.0 / ui->LoopFreq->text().toDouble();
}

void MainWindow::on_pbRunFor_clicked()
{
    double runTime = ui->runTime->text().toDouble();

    if((runTime<m_timestep) || (runTime>60))
        return;

    runFor(int(runTime/m_timestep));
}

void MainWindow::on_pbRunFor10s_clicked()
{
    runFor(int(10.0/m_timestep));
}

void MainWindow::on_pbRunFor1s_clicked()
{
    runFor(int(1.0/m_timestep));
}

void MainWindow::on_pbRunFor100ms_clicked()
{
    runFor(int(0.1/m_timestep));
}

void MainWindow::on_pbRunFor10ms_clicked()
{
    runFor(int(0.01/m_timestep));
}

void MainWindow::on_pbStep_clicked()
{
    runFor(1);
}

void MainWindow::on_pbRestart_clicked()
{
    m_time = 0;
    motor->Restart();
    motorGraph.clearData();
    simulationGraph.clearData();
    controllerGraph.clearData();
    debugGraph.clearData();
}

void MainWindow::calcFluxLinkage(void)
{
    double fluxLink = ui->BaseVolts->text().toDouble() /(2.0 * M_PI * m_Poles * (ui->BaseFreq->text().toDouble()/60));
    ui->CalcFluxLink->setText(QString::number(fluxLink,'g',2));
}

void MainWindow::on_BaseFreq_editingFinished()
{
    calcFluxLinkage();
}

void MainWindow::on_BaseVolts_editingFinished()
{
    calcFluxLinkage();
}

void MainWindow::on_torqueDemand_editingFinished()
{
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());
}

void MainWindow::on_throttleCurrent_editingFinished()
{
    Param::Set(Param::throtcur, FP_FROMFLT(ui->throttleCurrent->text().toFloat()));
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat()); //make sure is recalculated
}

void MainWindow::on_opMode_editingFinished()
{
    PwmGeneration::SetOpmode(ui->opMode->text().toInt());
}

void MainWindow::on_direction_editingFinished()
{
    Param::SetInt(Param::dir, ui->direction->text().toInt());
}

void MainWindow::on_IqManual_editingFinished()
{
    Param::Set(Param::manualiq, FP_FROMFLT(ui->IqManual->text().toFloat()));
}

void MainWindow::on_IdManual_editingFinished()
{
    Param::Set(Param::manualid, FP_FROMFLT(ui->IdManual->text().toFloat()));
}

void MainWindow::on_CurrentKp_editingFinished()
{
    Param::Set(Param::curkp, FP_FROMINT(ui->CurrentKp->text().toInt()));
}

void MainWindow::on_CurrentKi_editingFinished()
{
    Param::Set(Param::curki, FP_FROMINT(ui->CurrentKi->text().toInt()));
}

void MainWindow::on_FluxLinkageDelta_editingFinished()
{
    m_fluxLinkageDelta = ui->FluxLinkageDelta->text().toDouble();
    motor->setFluxLinkageDelta(m_fluxLinkageDelta);
}

void MainWindow::on_SyncAdv_editingFinished()
{
    Param::SetInt(Param::syncadv, ui->SyncAdv->text().toInt());
}

void MainWindow::on_LqMinusLd_editingFinished()
{
    Param::Set(Param::lqminusld, FP_FROMFLT(ui->LqMinusLd->text().toFloat()));
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat()); //make sure MTPA is recalculated
}

void MainWindow::on_SyncDelay_editingFinished()
{
    m_syncdelay = ui->SyncDelay->text().toDouble()/1000000; //entered in uS
    motor->setSyncDelay(m_syncdelay);
}

void MainWindow::on_FWStart_editingFinished()
{
    Param::Set(Param::ffwstart, FP_FROMFLT(ui->FWStart->text().toFloat()));
}

void MainWindow::on_FreqMax_editingFinished()
{
    Param::Set(Param::fmax, FP_FROMFLT(ui->FreqMax->text().toFloat()));
}

void MainWindow::on_FWKp_editingFinished()
{
    Param::Set(Param::fwkp, FP_FROMINT(ui->FWKp->text().toInt()));
}

void MainWindow::on_FWKi_editingFinished()
{
    Param::Set(Param::fwki, FP_FROMINT(ui->FWKi->text().toInt()));
}

void MainWindow::on_VLimKp_editingFinished()
{
    Param::Set(Param::vlimkp, FP_FROMINT(ui->VLimKp->text().toInt()));
}

void MainWindow::on_VLimKi_editingFinished()
{
    Param::Set(Param::vlimki, FP_FROMINT(ui->VLimKi->text().toInt()));
}

void MainWindow::on_SamplingPoint_editingFinished()
{
    m_samplingPoint = ui->SamplingPoint->text().toDouble()/100.0; //entered in %
    motor->setSamplingPoint(m_samplingPoint);
}
