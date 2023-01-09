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
#include <QRandomGenerator>
#include <QSettings>
#include "pwmgeneration.h"
#include "foc.h"
#include "params.h"
#include "inc_encoder.h"
#include "teststubs.h"
#include "my_math.h"

#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#include "anain.h"

//Current graph
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

//Voltage graph
#define VVD 1
#define VVQ 2
#define VVQ_BEMF 3
#define VVQ_DT_ID 4
#define VVD_DT_IQ 5
#define VVQ_DT_RQ 6
#define VVD_DT_RD 7
#define VVLD 8
#define VVLQ 9

//Power/Torque graph
#define POWER 6
#define TORQUE 7

//Op point graph
#define IDIQAMPS 2

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

    QSettings settings("OpenInverter", "IPMMotorSim");
    restoreGeometry(settings.value("mainwin/geometry").toByteArray());
    restoreState(settings.value("mainwin/windowState").toByteArray());

    if(settings.contains(ui->vehicleWeight->objectName())) ui->vehicleWeight->setText(settings.value(ui->vehicleWeight->objectName(),QString()).toString());
    if(settings.contains(ui->wheelSize->objectName())) ui->wheelSize->setText(settings.value(ui->wheelSize->objectName(),QString()).toString());
    if(settings.contains(ui->gearRatio->objectName())) ui->gearRatio->setText(settings.value(ui->gearRatio->objectName(),QString()).toString());
    if(settings.contains(ui->Vdc->objectName())) ui->Vdc->setText(settings.value(ui->Vdc->objectName(),QString()).toString());
    if(settings.contains(ui->Lq->objectName())) ui->Lq->setText(settings.value(ui->Lq->objectName(),QString()).toString());
    if(settings.contains(ui->Ld->objectName())) ui->Ld->setText(settings.value(ui->Ld->objectName(),QString()).toString());
    if(settings.contains(ui->Rs->objectName())) ui->Rs->setText(settings.value(ui->Rs->objectName(),QString()).toString());
    if(settings.contains(ui->SyncDelay->objectName())) ui->SyncDelay->setText(settings.value(ui->SyncDelay->objectName(),QString()).toString());
    if(settings.contains(ui->LoopFreq->objectName())) ui->LoopFreq->setText(settings.value(ui->LoopFreq->objectName(),QString()).toString());
    if(settings.contains(ui->SamplingPoint->objectName())) ui->SamplingPoint->setText(settings.value(ui->SamplingPoint->objectName(),QString()).toString());
    if(settings.contains(ui->ExtraCycleDelay->objectName())) ui->ExtraCycleDelay->setChecked(settings.value(ui->ExtraCycleDelay->objectName()).toBool());
    if(settings.contains(ui->AddNoise->objectName())) ui->AddNoise->setChecked(settings.value(ui->AddNoise->objectName()).toBool());
    if(settings.contains(ui->NoiseAmp->objectName())) ui->NoiseAmp->setText(settings.value(ui->NoiseAmp->objectName(),QString()).toString());
    if(settings.contains(ui->runTime->objectName())) ui->runTime->setText(settings.value(ui->runTime->objectName(),QString()).toString());
    if(settings.contains(ui->RoadGradient->objectName())) ui->RoadGradient->setText(settings.value(ui->RoadGradient->objectName(),QString()).toString());
    if(settings.contains(ui->ThrotRamps->objectName())) ui->ThrotRamps->setChecked(settings.value(ui->ThrotRamps->objectName()).toBool());

    motorGraph = new DataGraph("motor", this);
    simulationGraph = new DataGraph("sim", this);
    controllerGraph = new DataGraph("cont", this);
    debugGraph = new DataGraph("debug", this);
    voltageGraph = new DataGraph("voltage", this);
    idigGraph = new IdIqGraph("idig", this);
    powerGraph = new DataGraph("power", this);

    motorGraph->hide();//not sure why needed but otherwise always up?

    ANA_IN_CONFIGURE(ANA_IN_LIST);

    //set any parameters that can upset simulation to safe values
    Param::SetInt(Param::syncofs,0); //simulator assumes perfect alignment
    Param::SetInt(Param::pinswap,0); //shouldn't be a problem but may be in the future
    Param::SetInt(Param::respolepairs,Param::GetInt(Param::polepairs)); //force resolver pole pairs to match motor

    ui->LqMinusLd->setText(QString::number(Param::GetFloat(Param::lqminusld), 'f', 1));
    ui->FluxLinkage->setText(QString::number(Param::GetInt(Param::fluxlinkage)));
    ui->SyncAdv->setText(QString::number(Param::GetInt(Param::syncadv)));
    ui->FreqMax->setText(QString::number(Param::GetFloat(Param::fmax), 'f', 1));
    ui->Poles->setText(QString::number(Param::GetFloat(Param::polepairs), 'f', 1));
    ui->CurrentKp->setText(QString::number(Param::GetInt(Param::curkp)));
    ui->CurrentKi->setText(QString::number(Param::GetInt(Param::curki)));
    ui->VLimMargin->setText(QString::number(Param::GetInt(Param::vlimmargin)));
    ui->VLimFlt->setText(QString::number(Param::GetInt(Param::vlimflt)));
    ui->FWCurrMax->setText(QString::number(Param::GetInt(Param::fwcurmax)));
    ui->IdManual->setText(QString::number(Param::GetFloat(Param::manualid), 'f', 1));
    ui->IqManual->setText(QString::number(Param::GetFloat(Param::manualiq), 'f', 1));
    ui->SyncAdv->setText(QString::number(Param::GetInt(Param::syncadv)));
    ui->throttleCurrent->setText(QString::number(Param::GetFloat(Param::throtcur), 'f', 1));

    m_wheelSize = ui->wheelSize->text().toDouble();
    m_vehicleWeight = ui->vehicleWeight->text().toDouble();
    m_gearRatio = ui->gearRatio->text().toDouble();
    m_Lq = ui->Lq->text().toDouble()/1000; //entered in mH
    m_Ld = ui->Ld->text().toDouble()/1000; //entered in mH
    m_Rs = ui->Rs->text().toDouble();
    m_Poles = ui->Poles->text().toDouble();
    m_fluxLinkage = ui->FluxLinkage->text().toDouble()/1000; //entered in mWeber
    m_syncdelay = ui->SyncDelay->text().toDouble()/1000000; //entered in uS
    m_samplingPoint = ui->SamplingPoint->text().toDouble()/100.0; //entered in %
    m_roadGradient = ui->RoadGradient->text().toDouble()/100.0; //entered in %
    m_runTime = ui->runTime->text().toDouble();

    m_timestep = 1.0 / ui->LoopFreq->text().toDouble();
    m_Vdc = ui->Vdc->text().toDouble();
    Param::SetFloat(Param::udc, m_Vdc);

    motor = new MotorModel(m_wheelSize,m_gearRatio,m_roadGradient,m_vehicleWeight,m_Lq,m_Ld,m_Rs,m_Poles,m_fluxLinkage,m_timestep,m_syncdelay,m_samplingPoint);

    m_time = 0;
    m_old_time = 0;
    m_old_ms_time = 0;

    m_oldVa = 0;
    m_oldVb = 0;
    m_oldVc = 0;

    m_lastTorqueDemand = 0;

    motorGraph->setWindowTitle("Motor Currents");
    motorGraph->setAxisText("", "Amps (A)", "");
    motorGraph->addSeries("Ia (A)", left, IA);
    motorGraph->addSeries("Ib (A)", left, IB);
    motorGraph->addSeries("Ic (A)", left, IC);
    motorGraph->addSeries("Iq (A)", left, IQ);
    motorGraph->setColour(Qt::blue, IQ);
    motorGraph->addSeries("Id (A)", left, ID);
    motorGraph->setColour(Qt::red, ID);
    if(settings.contains(ui->cb_MotCurr->objectName())) ui->cb_MotCurr->setChecked(settings.value(ui->cb_MotCurr->objectName()).toBool());
    if(settings.contains(ui->cb_PhaseCurrs->objectName())) ui->cb_PhaseCurrs->setChecked(settings.value(ui->cb_PhaseCurrs->objectName()).toBool());

    simulationGraph->setWindowTitle("Simulation Data");
    simulationGraph->setAxisText("", "Angle (Degrees)", "Speed (Hz)");
    simulationGraph->addSeries("Motor Position (degrees)", left, M_MOTOR_POS);
    simulationGraph->setOpacity(0.25, M_MOTOR_POS);
    simulationGraph->addSeries("Controller Position (degrees)", left, M_CONT_POS);
    simulationGraph->setOpacity(0.25, M_CONT_POS);
    simulationGraph->addSeries("Motor Elec Speed (Hz)", right, M_RPM);
    simulationGraph->setColour(Qt::blue, M_RPM);
    if(settings.contains(ui->cb_Simulation->objectName())) ui->cb_Simulation->setChecked(settings.value(ui->cb_Simulation->objectName()).toBool());
    if(settings.contains(ui->cb_MotorPos->objectName())) ui->cb_MotorPos->setChecked(settings.value(ui->cb_MotorPos->objectName()).toBool());

    controllerGraph->setWindowTitle("Controller Voltages");
    controllerGraph->setAxisText("", "Volts (V)", "");
    controllerGraph->addSeries("Va (V)", left, VA);
    controllerGraph->addSeries("Vb (V)", left, VB);
    controllerGraph->addSeries("Vc (V)", left, VC);
    controllerGraph->addSeries("Vq (V)", left, VQ);
    controllerGraph->setColour(Qt::blue, VQ);
    controllerGraph->addSeries("Vd (V)", left, VD);
    controllerGraph->setColour(Qt::red, VD);
    if(settings.contains(ui->cb_ContVolt->objectName())) ui->cb_ContVolt->setChecked(settings.value(ui->cb_ContVolt->objectName()).toBool());
    if(settings.contains(ui->cb_PhaseVolts->objectName())) ui->cb_PhaseVolts->setChecked(settings.value(ui->cb_PhaseVolts->objectName()).toBool());

    debugGraph->setWindowTitle("Controller Currents");
    debugGraph->setAxisText("", "Amps (A)", "");
    debugGraph->addSeries("Iq (A)", left, C_IQ);
    debugGraph->setColour(Qt::blue, C_IQ);
    debugGraph->addSeries("Id (A)", left, C_ID);
    debugGraph->setColour(Qt::red, C_ID);
    debugGraph->addSeries("Ifw (A)", left, C_IFW);
    debugGraph->addSeries("Throttle Reduction (%)", right, C_IVLIM);
    if(settings.contains(ui->cb_ContCurr->objectName())) ui->cb_ContCurr->setChecked(settings.value(ui->cb_ContCurr->objectName()).toBool());

    voltageGraph->setWindowTitle("Motor Voltages");
    voltageGraph->setAxisText("", "Volts (V)", "");
    voltageGraph->addSeries("Vd (V)", left, VVD);
    voltageGraph->setColour(Qt::red, VVD);
    voltageGraph->addSeries("Vq (V)", left, VVQ);
    voltageGraph->setColour(Qt::blue, VVQ);
    voltageGraph->addSeries("Vq_BEMF (V)", left, VVQ_BEMF);
    voltageGraph->addSeries("Vq_LdId (V)", left, VVQ_DT_ID);
    voltageGraph->addSeries("Vd_LqIq (V)", left, VVD_DT_IQ);
    voltageGraph->addSeries("Vq_RqIq (V)", left, VVQ_DT_RQ);
    voltageGraph->addSeries("Vd_RdIq (V)", left, VVD_DT_RD);
    voltageGraph->addSeries("VLd (V)", left, VVLD);
    voltageGraph->addSeries("VLq (V)", left, VVLQ);
    if(settings.contains(ui->cb_MotVolt->objectName())) ui->cb_MotVolt->setChecked(settings.value(ui->cb_MotVolt->objectName()).toBool());

    idigGraph->setWindowTitle("Operating Point");
    idigGraph->setAxisText("Id (A)", "Iq (A)", "");
    idigGraph->addSeries("I (A)", left, IDIQAMPS);
    if(settings.contains(ui->cb_OpPoint->objectName())) ui->cb_OpPoint->setChecked(settings.value(ui->cb_OpPoint->objectName()).toBool());    
    if(settings.contains(ui->rb_OP_Amps->objectName()))
    {
        if(settings.value(ui->rb_OP_Amps->objectName()).toBool())
        {
            ui->rb_OP_Amps->setChecked(true);
            idigGraph->setAxisText("Id (A)", "Iq (A)", "");
        }
        else
        {
            ui->rb_OP_Volts->setChecked(true);
            idigGraph->setAxisText("Vd (V)", "Vq (V)", "");
            idigGraph->updateSeries("V (V)", left, IDIQAMPS);
        }
    }

    powerGraph->setWindowTitle("Power/Torque");
    powerGraph->setAxisText("Time (s)", "Power (kW)", "Torque (Nm)");
    powerGraph->addSeries("Power (kW)", left, POWER);
    powerGraph->addSeries("Torque (Nm)", right, TORQUE);
    if(settings.contains(ui->cb_PowTorqTime->objectName())) ui->cb_PowTorqTime->setChecked(settings.value(ui->cb_PowTorqTime->objectName()).toBool());
    if(settings.contains(ui->rb_Speed->objectName()))
    {
        if(settings.value(ui->rb_Speed->objectName()).toBool())
        {
            ui->rb_Speed->setChecked(true);
            powerGraph->setAxisText("Shaft Speed (rpm)", "Power (kW)", "Torque (Nm)");
        }
        else
        {
            ui->rb_Time->setChecked(true);
            powerGraph->setAxisText("Time (s)", "Power (kW)", "Torque (Nm)");
        }
    }

    //following block copied from OpenInverter - probably not needed
    Param::SetInt(Param::version, 4); //backward compatibility

    if (Param::GetInt(Param::snsm) < 12)
        Param::SetInt(Param::snsm, Param::GetInt(Param::snsm) + 10); //upgrade parameter
    if (Param::Get(Param::offthrotregen) > 0)
        Param::Set(Param::offthrotregen, -Param::Get(Param::offthrotregen));


    Param::Change(Param::PARAM_LAST);
    Param::Change(Param::nodeid);

    PwmGeneration::SetOpmode(0);
    PwmGeneration::SetOpmode(ui->opMode->text().toInt());
    Param::SetInt(Param::dir, ui->direction->text().toInt());

    ui->Poles->setText(QString::number(Param::GetInt(Param::polepairs)));
    ui->throttleCurrent->setText(QString::number(Param::GetInt(Param::throtcur)));

    FOC::SetMotorParameters(Param::GetFloat(Param::lqminusld)/1000, Param::GetFloat(Param::fluxlinkage)/1000);

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
    QSettings settings("OpenInverter", "IPMMotorSim");
    settings.setValue("mainwin/geometry", saveGeometry());
    settings.setValue("mainwin/windowState", saveState());

    settings.setValue(ui->vehicleWeight->objectName(), ui->vehicleWeight->text());
    settings.setValue(ui->wheelSize->objectName(), ui->wheelSize->text());
    settings.setValue(ui->gearRatio->objectName(), ui->gearRatio->text());
    settings.setValue(ui->Vdc->objectName(), ui->Vdc->text());
    settings.setValue(ui->Lq->objectName(), ui->Lq->text());
    settings.setValue(ui->Ld->objectName(), ui->Ld->text());
    settings.setValue(ui->Rs->objectName(), ui->Rs->text());
    settings.setValue(ui->SyncDelay->objectName(), ui->SyncDelay->text());
    settings.setValue(ui->LoopFreq->objectName(), ui->LoopFreq->text());
    settings.setValue(ui->SamplingPoint->objectName(), ui->SamplingPoint->text());
    settings.setValue(ui->ExtraCycleDelay->objectName(), ui->ExtraCycleDelay->isChecked());
    settings.setValue(ui->AddNoise->objectName(), ui->AddNoise->isChecked());
    settings.setValue(ui->NoiseAmp->objectName(), ui->NoiseAmp->text());
    settings.setValue(ui->runTime->objectName(), ui->runTime->text());
    settings.setValue(ui->ThrotRamps->objectName(), ui->ThrotRamps->isChecked());
    settings.setValue(ui->RoadGradient->objectName(), ui->RoadGradient->text());

    settings.setValue(ui->cb_ContCurr->objectName(), ui->cb_ContCurr->isChecked());
    settings.setValue(ui->cb_ContVolt->objectName(), ui->cb_ContVolt->isChecked());
    settings.setValue(ui->cb_MotCurr->objectName(), ui->cb_MotCurr->isChecked());
    settings.setValue(ui->cb_MotVolt->objectName(), ui->cb_MotVolt->isChecked());
    settings.setValue(ui->cb_OpPoint->objectName(), ui->cb_OpPoint->isChecked());
    settings.setValue(ui->cb_PowTorqTime->objectName(), ui->cb_PowTorqTime->isChecked());
    settings.setValue(ui->cb_Simulation->objectName(), ui->cb_Simulation->isChecked());
    settings.setValue(ui->rb_Speed->objectName(), ui->rb_Speed->isChecked());

    settings.setValue(ui->cb_PhaseCurrs->objectName(), ui->cb_PhaseCurrs->isChecked());
    settings.setValue(ui->cb_MotorPos->objectName(), ui->cb_MotorPos->isChecked());
    settings.setValue(ui->cb_PhaseVolts->objectName(), ui->cb_PhaseVolts->isChecked());
    settings.setValue(ui->rb_OP_Amps->objectName(), ui->rb_OP_Amps->isChecked());

    motorGraph->saveWinState();
    simulationGraph->saveWinState();
    controllerGraph->saveWinState();
    debugGraph->saveWinState();
    voltageGraph->saveWinState();
    idigGraph->saveWinState();
    powerGraph->saveWinState();
    QWidget::closeEvent(event);
}

void MainWindow::runFor(int num_steps)
{
    double Va = 0;
    double Vb = 0;
    double Vc = 0;

    if(num_steps<0)
        return;

    QList<QPointF> listIa, listIb, listIc, listIq, listId;
    QList<QPointF> listMFreq, listMPos, listContMPos;
    QList<QPointF> listCVa, listCVb, listCVc, listCVq, listCVd, listCIq, listCId, listCifw;//, listCivlim;
    QList<QPointF> listVVd, listVVq, listVVq_bemf, listVVq_dueto_id, listVVd_dueto_iq, listVVq_dueto_Rq, listVVd_dueto_Rd, listVVLd, listVVLq;
    QList<QPointF> listIdIq;
    QList<QPointF> listPower, listTorque;

    //PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());
    for(int i = 0;i<num_steps; i++)
    {
        //routines that need calling every 10ms
        if((uint32_t)(m_time*100) != m_old_time)
        {
            m_old_time = (uint32_t)(m_time*100);
            Encoder::UpdateRotorFrequency(100);

            int requestedTorque = ui->torqueDemand->text().toInt() * 100;
            if(ui->ThrotRamps->isChecked())
            {
                //ramps set at 5% above 0 and 0.5% below
                if(m_lastTorqueDemand != requestedTorque)
                {
                    if(requestedTorque > m_lastTorqueDemand)
                        requestedTorque = RAMPUP(m_lastTorqueDemand, requestedTorque, ((m_lastTorqueDemand>=0)?500:50));
                    else
                        requestedTorque = RAMPDOWN(m_lastTorqueDemand, requestedTorque, ((m_lastTorqueDemand>=0)?500:50));
                    m_lastTorqueDemand = requestedTorque;
                }
                PwmGeneration::SetTorquePercent(((float)(requestedTorque+50))/100);
            }
            else
                PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());
        }

        //routines that need calling every ms
        if((uint32_t)(m_time*1000) != m_old_ms_time)
        {
            m_old_ms_time = (uint32_t)(m_time*100);
            //not used at the moment but left in for future use
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

        if(ui->AddNoise->isChecked())
        {
            double noise = ui->NoiseAmp->text().toDouble();
            g_il1_input += QRandomGenerator::global()->bounded(noise) - (noise/2);
            g_il2_input += QRandomGenerator::global()->bounded(noise) - (noise/2);
        }

        PwmGeneration::Run();

        if(disablePWM) //needed to allow OpeinInverter initialisation to complete
        {
            Va = 0;
            Vb = 0;
            Vc = 0;
        }
        else
        {
            Va = (m_Vdc/65536) * (FOC::DutyCycles[0]-32768);
            Vb = (m_Vdc/65536) * (FOC::DutyCycles[1]-32768);
            Vc = (m_Vdc/65536) * (FOC::DutyCycles[2]-32768);
        }

        //add voltages to plot here so that we see the SVM waveforms
        if(ui->cb_PhaseVolts->isChecked())
        {
            listCVa.append(QPointF(m_time, Va));
            listCVb.append(QPointF(m_time, Vb));
            listCVc.append(QPointF(m_time, Vc));
        }

        //remove space vector modulation
        double offset = Va + Vb + Vc;
        Va = Va - offset/3;
        Vb = Vb - offset/3;
        Vc = Vc - offset/3;

        //one period delay to simulate slow timer reload in target hardware
        if(ui->ExtraCycleDelay->isChecked())
            motor->Step(m_oldVa,m_oldVb,m_oldVc);
        else
            motor->Step(Va,Vb,Vc);
        m_oldVa = Va;
        m_oldVb = Vb;
        m_oldVb = Vb;

        //motor->Step(0,0,0);
        if(ui->cb_PhaseCurrs->isChecked())
        {
            listIa.append(QPointF(m_time, motor->getIaSamp()));
            listIb.append(QPointF(m_time, motor->getIbSamp()));
            listIc.append(QPointF(m_time, motor->getIcSamp()));
        }
        listIq.append(QPointF(m_time, motor->getIq()));
        listId.append(QPointF(m_time, motor->getId()));

        listMFreq.append(QPointF(m_time, (motor->getMotorFreq()*m_Poles)));
        if(ui->cb_MotorPos->isChecked())
        {
            listMPos.append(QPointF(m_time, motor->getMotorPosition()));
            listContMPos.append(QPointF(m_time, (360.0 * PwmGeneration::GetAngle())/TWO_PI_CONT));
        }

          //inlcude here to see sinusoidal waveforms that motor sees
//        if(ui->cb_PhaseVolts->isChecked())
//        {
//            listCVa.append(QPointF(m_time, Va));
//            listCVb.append(QPointF(m_time, Vb));
//            listCVc.append(QPointF(m_time, Vc));
//        }
        listCVq.append(QPointF(m_time, (m_Vdc/65536) * Param::GetFloat(Param::uq)));
        listCVd.append(QPointF(m_time, (m_Vdc/65536) * Param::GetFloat(Param::ud)));

        listCIq.append(QPointF(m_time, Param::GetFloat(Param::iq)));
        listCId.append(QPointF(m_time, Param::GetFloat(Param::id)));

        listCifw.append(QPointF(m_time, Param::GetFloat(Param::ifw)));
        //listCivlim.append(QPointF(m_time, Param::GetFloat(Param::vlim)));

        listVVd.append(QPointF(m_time, motor->getVd()));
        listVVq.append(QPointF(m_time, motor->getVq()));
        listVVq_bemf.append(QPointF(m_time, motor->getVq_bemf()));
        listVVq_dueto_id.append(QPointF(m_time, motor->getVq_dueto_id()));
        listVVd_dueto_iq.append(QPointF(m_time, motor->getVd_dueto_iq()));
        listVVq_dueto_Rq.append(QPointF(m_time, motor->getVq_dueto_Rq()));
        listVVd_dueto_Rd.append(QPointF(m_time, motor->getVd_dueto_Rd()));
        listVVLd.append(QPointF(m_time, motor->getVLd()));
        listVVLq.append(QPointF(m_time, motor->getVLq()));

        if(ui->rb_OP_Amps->isChecked())
            listIdIq.append(QPointF(motor->getId(), motor->getIq()));
        else
            listIdIq.append(QPointF(motor->getVd(), motor->getVq()));

        if(ui->rb_Speed->isChecked())
        {
            listPower.append(QPointF(motor->getMotorFreq()*60, motor->getPower()/1000));
            listTorque.append(QPointF(motor->getMotorFreq()*60, motor->getTorque()));
        }
        else
        {
            listPower.append(QPointF(m_time, motor->getPower()/1000));
            listTorque.append(QPointF(m_time, motor->getTorque()));
        }

        m_time += m_timestep;
    }

    motorGraph->addDataPoints(listIa, IA);
    motorGraph->addDataPoints(listIb, IB);
    motorGraph->addDataPoints(listIc, IC);
    motorGraph->addDataPoints(listIq, IQ);
    motorGraph->addDataPoints(listId, ID);

    simulationGraph->addDataPoints(listMFreq, M_RPM);
    simulationGraph->addDataPoints(listMPos, M_MOTOR_POS);
    simulationGraph->addDataPoints(listContMPos, M_CONT_POS);

    controllerGraph->addDataPoints(listCVa, VA);
    controllerGraph->addDataPoints(listCVb, VB);
    controllerGraph->addDataPoints(listCVc, VC);
    controllerGraph->addDataPoints(listCVq, VQ);
    controllerGraph->addDataPoints(listCVd, VD);

    debugGraph->addDataPoints(listCIq, C_IQ);
    debugGraph->addDataPoints(listCId, C_ID);
    debugGraph->addDataPoints(listCifw, C_IFW);
    //debugGraph->addDataPoints(listCivlim, C_IVLIM);

    voltageGraph->addDataPoints(listVVd, VVD);
    voltageGraph->addDataPoints(listVVq, VVQ);
    voltageGraph->addDataPoints(listVVq_bemf, VVQ_BEMF);
    voltageGraph->addDataPoints(listVVq_dueto_id, VVQ_DT_ID);
    voltageGraph->addDataPoints(listVVd_dueto_iq, VVD_DT_IQ);
    voltageGraph->addDataPoints(listVVq_dueto_Rq, VVQ_DT_RQ);
    voltageGraph->addDataPoints(listVVd_dueto_Rd, VVD_DT_RD);
    voltageGraph->addDataPoints(listVVLd, VVLD);
    voltageGraph->addDataPoints(listVVLq, VVLQ);

    idigGraph->addDataPoints(listIdIq, IDIQAMPS);

    powerGraph->addDataPoints(listPower, POWER);
    powerGraph->addDataPoints(listTorque, TORQUE);

    if(ui->cb_MotCurr->isChecked()) motorGraph->updateGraph();
    if(ui->cb_Simulation->isChecked()) simulationGraph->updateGraph();
    if(ui->cb_ContVolt->isChecked()) controllerGraph->updateGraph();
    if(ui->cb_ContCurr->isChecked()) debugGraph->updateGraph();
    if(ui->cb_MotVolt->isChecked()) voltageGraph->updateGraph();
    if(ui->cb_OpPoint->isChecked()) idigGraph->updateGraph(ui->rb_OP_Amps->isChecked());
    if(ui->cb_PowTorqTime->isChecked()) powerGraph->updateGraph();
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
    Param::SetFloat(Param::udc, m_Vdc);
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
    Param::Set(Param::respolepairs,FP_FROMINT(ui->Poles->text().toInt())); //force resolver pole pairs to match motor
    motor->setPoles(m_Poles);
}

void MainWindow::on_FluxLinkage_editingFinished()
{
    m_fluxLinkage = ui->FluxLinkage->text().toDouble()/1000;
    Param::Set(Param::fluxlinkage, FP_FROMFLT(ui->FluxLinkage->text().toFloat()));
    motor->setFluxLinkage(m_fluxLinkage);
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat()); //make sure is recalculated
}

void MainWindow::on_LoopFreq_editingFinished()
{
    m_timestep = 1.0 / ui->LoopFreq->text().toDouble();
}

void MainWindow::on_pbRunFor_clicked()
{
    runFor(int(m_runTime/m_timestep));
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
    motor->Restart();
    int throt = ui->torqueDemand->text().toInt();
    ui->torqueDemand->setText(QString::number(0));
    PwmGeneration::SetOpmode(0);
    PwmGeneration::SetOpmode(ui->opMode->text().toInt()); //reset controller integrators
    PwmGeneration::SetTorquePercent(0);
    runFor(6000); //allow controller to complete initialisation
    ui->torqueDemand->setText(QString::number(throt));
    PwmGeneration::SetTorquePercent(ui->torqueDemand->text().toFloat());
    testStubsClearEncoder();
    m_time = 0;
    motor->Restart();
    motorGraph->clearData();
    simulationGraph->clearData();
    controllerGraph->clearData();
    debugGraph->clearData();
    voltageGraph->clearData();
    idigGraph->clearData();
    powerGraph->clearData();
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
    Param::Set(Param::dir, FP_FROMINT(ui->direction->text().toInt()));
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

void MainWindow::on_SyncAdv_editingFinished()
{
    Param::Set(Param::syncadv, FP_FROMINT(ui->SyncAdv->text().toInt()));
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

void MainWindow::on_FreqMax_editingFinished()
{
    Param::Set(Param::fmax, FP_FROMFLT(ui->FreqMax->text().toFloat()));
}

void MainWindow::on_SamplingPoint_editingFinished()
{
    m_samplingPoint = ui->SamplingPoint->text().toDouble()/100.0; //entered in %
    motor->setSamplingPoint(m_samplingPoint);
}

void MainWindow::on_pbTransient_clicked()
{
    QString torque = ui->torqueDemand->text();
    for(int i=0;i<2;i++)
    {
        ui->torqueDemand->setText("0");
        runFor(int(m_runTime/m_timestep));
        ui->torqueDemand->setText(torque);
        runFor(int(m_runTime/m_timestep));
    }
}

void MainWindow::on_SyncOfs_editingFinished()
{
    Param::Set(Param::syncofs, FP_FROMINT(ui->SyncOfs->text().toInt()));
}

void MainWindow::on_pbAccelCoast_clicked()
{   
    QString torque = ui->torqueDemand->text();
    runFor(int(m_runTime/m_timestep));
    ui->torqueDemand->setText("0");
    runFor(int(m_runTime/m_timestep));
    ui->torqueDemand->setText(torque);
}

void MainWindow::on_cb_OpPoint_toggled(bool checked)
{
    if(checked)
    {
        idigGraph->updateGraph(ui->rb_OP_Amps->isChecked());
        idigGraph->show();
    }
    else
        idigGraph->hide();
}

void MainWindow::on_cb_Simulation_toggled(bool checked)
{
    if(checked)
    {
        simulationGraph->updateGraph();
        simulationGraph->show();
    }
    else
        simulationGraph->hide();
}

void MainWindow::on_cb_ContVolt_toggled(bool checked)
{
    if(checked)
    {
        controllerGraph->updateGraph();
        controllerGraph->show();
    }
    else
        controllerGraph->hide();
}

void MainWindow::on_cb_ContCurr_toggled(bool checked)
{
    if(checked)
    {
        debugGraph->updateGraph();
        debugGraph->show();
    }
    else
        debugGraph->hide();
}

void MainWindow::on_cb_MotVolt_toggled(bool checked)
{
    if(checked)
    {
        voltageGraph->updateGraph();
        voltageGraph->show();
    }
    else
        voltageGraph->hide();
}

void MainWindow::on_cb_MotCurr_toggled(bool checked)
{
    if(checked)
    {
        motorGraph->updateGraph();
        motorGraph->show();
    }
    else
        motorGraph->hide();
}

void MainWindow::on_cb_PowTorqTime_toggled(bool checked)
{
    if(checked)
    {
        powerGraph->updateGraph();
        powerGraph->show();
    }
    else
        powerGraph->hide();
}

void MainWindow::on_rb_Speed_toggled(bool checked)
{
    powerGraph->clearData(); //need to restart as data arrays not right for new mode
    if(checked)
        powerGraph->setAxisText("Shaft Speed (rpm)", "Power (kW)", "Torque (Nm)");
    else
        powerGraph->setAxisText("Time (s)", "Power (kW)", "Torque (Nm)");
}

void MainWindow::on_RoadGradient_editingFinished()
{
    m_roadGradient = ui->RoadGradient->text().toDouble()/100.0; //entered in %
    motor->setRoadGradient(m_roadGradient);
}

void MainWindow::on_runTime_editingFinished()
{
    m_runTime = ui->runTime->text().toDouble();

    if(m_runTime<m_timestep)
        m_runTime = 1;
    if(m_runTime>60)
        m_runTime = 60;
}

void MainWindow::on_VLimMargin_editingFinished()
{
    Param::Set(Param::vlimmargin, FP_FROMINT(ui->VLimMargin->text().toInt()));
}

void MainWindow::on_VLimFlt_editingFinished()
{
    Param::Set(Param::vlimflt, FP_FROMINT(ui->VLimFlt->text().toInt()));
}

void MainWindow::on_FWCurrMax_editingFinished()
{
    Param::Set(Param::fwcurmax, FP_FROMINT(ui->FWCurrMax->text().toInt()));
}

void MainWindow::on_rb_OP_Amps_toggled(bool checked)
{
    idigGraph->clearData(); //need to restart as data arrays not right for new mode
    if(checked)
    {
        idigGraph->setAxisText("Id (A)", "Iq (A)", "");
        idigGraph->updateSeries("I (A)", left, IDIQAMPS);
    }
    else
    {
        idigGraph->setAxisText("Vd (V)", "Vq (V)", "");
        idigGraph->updateSeries("V (V)", left, IDIQAMPS);
    }
}

