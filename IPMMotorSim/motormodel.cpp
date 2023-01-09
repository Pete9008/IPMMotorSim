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

#include "motormodel.h"

MotorModel::MotorModel(double wheelSize,double ratio,double roadGradient,double mass,double Lq,double Ld,double Rs,double poles,double fluxLink,double timestep, double syncDelay, double sampPoint)
    :m_WheelSize{wheelSize},m_Ratio{ratio},m_RoadGradient{roadGradient},m_Mass{mass},m_Lq{Lq},m_Ld{Ld},m_Rs{Rs},m_Poles{poles},m_FluxLink{fluxLink}, m_syncdelay{syncDelay}, m_samplingPoint{sampPoint}, m_Timestep{timestep}
{
    Restart();
}

void MotorModel::Restart(void)
{
    m_Position = 0;
    m_Frequency = 0;
    m_Speed = 0;
    m_Ia = 0;
    m_Ib = 0;
    m_Ic = 0;
    m_IaSamp = 0;
    m_IbSamp = 0;
    m_IcSamp = 0;
    m_Id = 0;
    m_Iq = 0;
    m_Power = 0;
    m_Torque = 0;
}

void MotorModel::Step(double Va, double Vb, double Vc)
{
    (void)Vc;
    double Valpha = Va;
    double Vbeta = ((Va+(2.0*Vb))/qSqrt(3.0));

    double elecAngle = fmod(m_Position,360.0);

    m_Vd = (Valpha * qCos(qDegreesToRadians(m_Position))) + (Vbeta * qSin(qDegreesToRadians(elecAngle)));
    m_Vq = (-Valpha * qSin(qDegreesToRadians(m_Position))) + (Vbeta * qCos(qDegreesToRadians(elecAngle)));

    m_Vq_bemf = m_FluxLink * m_Poles * m_Frequency * 2 * M_PI;
    m_Vq_dueto_id = m_Poles * m_Frequency * 2 * M_PI * m_Ld * m_Id;
    m_Vd_dueto_iq = m_Poles * m_Frequency * 2 * M_PI * m_Lq * m_Iq;

    m_Vd_dueto_Rd = (m_Rs * m_Id);
    m_Vq_dueto_Rq = (m_Rs * m_Iq);

    m_VLd = m_Vd - m_Vd_dueto_Rd + m_Vd_dueto_iq;
    m_VLq = m_Vq - m_Vq_dueto_Rq - m_Vq_bemf - m_Vq_dueto_id;

    //variables to allow for values to be read at a variable sampling point to simulate non ideal behaviour of real controllers
    double IdSamp = m_Id + (m_VLd * m_Timestep * m_samplingPoint)/m_Ld;
    double IqSamp = m_Iq + (m_VLq * m_Timestep * m_samplingPoint)/m_Lq;
    double oldPosition = m_Position;

    double Id_delta = (m_VLd * m_Timestep)/m_Ld;
    double Iq_delta = (m_VLq * m_Timestep)/m_Lq;

    m_Id = m_Id + Id_delta;
    m_Iq = m_Iq + Iq_delta;

    double Ialpha = (m_Id * qCos(qDegreesToRadians(elecAngle))) - (m_Iq * qSin(qDegreesToRadians(elecAngle)));
    double Ibeta = (m_Id * qSin(qDegreesToRadians(elecAngle))) + (m_Iq * qCos(qDegreesToRadians(elecAngle)));

    m_Ia = Ialpha;
    m_Ib = (-Ialpha + (qSqrt(3.0) * Ibeta)) / 2.0;
    m_Ic = (-Ialpha - (qSqrt(3.0) * Ibeta)) / 2.0;

    m_Torque = (3.0/2.0) * m_Poles * ((m_FluxLink * m_Iq) + ((m_Ld - m_Lq) * m_Id * m_Iq));

    //This is a very simple model just lumping everything together in a single vehicle mass
    //A better approach would be to have a fast and slow calculation
    //The slow calculation is pretty much as below and based on car mass
    //The fast calculation kicks in on direction changes produces a position calculated just from the inertia for the motor and geartrain.  The
    //position delta from this component would be limited by a configurable driveshaft angular play parameter.
    //If added this would allow driveline shunt to be simulated by the model
    double wheelTorque = (m_Torque * m_Ratio) / m_WheelSize;//m_Wheelsize is radius (in m) to give N here
    double gradientForce = -(qSin(qAtan(m_RoadGradient))*m_Mass*9.81);
    double accelForce = wheelTorque + gradientForce;
    double accel = accelForce/m_Mass;
    m_Speed = m_Speed + (accel * m_Timestep);
    m_Frequency = (m_Speed / (2.0 * M_PI * m_WheelSize)) * m_Ratio;
    m_Power = 2.0 * M_PI * m_Frequency * m_Torque;

    double posDelta = m_Frequency * m_Timestep * (360.0 * m_Poles);
    m_Position = m_Position + posDelta;

    //Remaining variable sampling point calculation, used to simulate OpenInverter sampling point (5.20 and earlier)
    double sampPosition = ((oldPosition * (1.0-m_samplingPoint)) + (m_Position * m_samplingPoint));
    double elecAngleSamp = fmod(sampPosition, 360.0);
    Ialpha = (IdSamp * qCos(qDegreesToRadians(elecAngleSamp))) - (IqSamp * qSin(qDegreesToRadians(elecAngleSamp)));
    Ibeta = (IdSamp * qSin(qDegreesToRadians(elecAngleSamp))) + (IqSamp * qCos(qDegreesToRadians(elecAngleSamp)));

    m_IaSamp = Ialpha;
    m_IbSamp = (-Ialpha + (qSqrt(3.0) * Ibeta)) / 2.0;
    m_IcSamp = (-Ialpha - (qSqrt(3.0) * Ibeta)) / 2.0;

    //leave wrapping the position till last to make the variable sampling point calculation easier
    if(m_Position>(360.0 * m_Poles))
        m_Position = m_Position - (360.0 * m_Poles);
    if(m_Position<0)
        m_Position = m_Position + (360.0 * m_Poles);

}

double MotorModel::getMotorPosition(void)
{
    double rotorPos = m_Position - (m_syncdelay * 360.0 * m_Poles * m_Frequency);
    if(rotorPos>(360.0 * m_Poles))
        rotorPos = rotorPos - (360.0 * m_Poles);
    if(rotorPos<0)
        rotorPos = rotorPos + (360.0 * m_Poles);

    return (rotorPos / m_Poles);
}

double MotorModel::getElecPosition(void)
{
    double rotorPos = m_Position - (m_syncdelay * 360.0 * m_Poles * m_Frequency);
    if(rotorPos>(360.0 * m_Poles))
        rotorPos = rotorPos - (360.0 * m_Poles);
    if(rotorPos<0)
        rotorPos = rotorPos + (360.0 * m_Poles);

    return (fmod(rotorPos,360.0));
}






