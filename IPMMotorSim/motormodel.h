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

#ifndef MOTORMODEL_H
#define MOTORMODEL_H

#include <QtMath>

class MotorModel
{
public:
    MotorModel(double wheelSize,double ratio,double drag,double mass,double Lq,double Ld,double Rs,double poles,double fluxLink,double timestep,double fluxLinkageDelta, double syncDelay, double sampPoint);
    void Step(double Va, double Vb, double Vc);
    void Restart(void);
    void setWheelSize(double val) {m_WheelSize = val;}
    void setGboxRatio(double val) {m_Ratio = val;}
    void setDrag(double val) {m_Drag = val;}
    void setVehicleMass(double val) {m_Mass = val;}
    void setLq(double val) {m_Lq = val;}
    void setLd(double val) {m_Ld = val;}
    void setRs(double val) {m_Rs = val;}
    void setPoles(double val) {m_Poles = val;}
    void setFluxLinkage(double val) {m_FluxLink = val;}
    void setFluxLinkageDelta(double val) {m_fluxLinkageDelta = val;}
    void setSyncDelay(double val) {m_syncdelay = val;}
    void setTimestep(double val) {m_Timestep = val;}
    void setPosition(double val) {m_Position = (val * m_Poles);}
    void setSamplingPoint(double val) {m_samplingPoint = val;}
    double getMotorPosition(void);
    double getElecPosition(void);
    double getMotorFreq(void) {return m_Frequency;}
    bool getMotorDirection(void) {return (m_Speed>=0);}
    double getIa(void) {return m_Ia;} //gets current at end of period, ideal controller sampling point
    double getIb(void) {return m_Ib;}
    double getIc(void) {return m_Ic;}
    double getIaSamp(void) {return m_IaSamp;} //gets current at samplingPoint into period, real controller sampling point
    double getIbSamp(void) {return m_IbSamp;}
    double getIcSamp(void) {return m_IcSamp;}
    double getIq(void) {return m_Iq;} //model output
    double getId(void) {return m_Id;}

private:
    double m_WheelSize;
    double m_Ratio;
    double m_Drag;
    double m_Mass;
    double m_Lq;
    double m_Ld;
    double m_Rs;
    double m_Poles;
    double m_FluxLink; //Hz
    double m_fluxLinkageDelta; //% change in BEMF per A of neg Id
    double m_syncdelay;
    double m_samplingPoint; //sampling position as fraction of period, 0=start, 1=end

    double m_Position; //degrees
    double m_Frequency; // Hz
    double m_Timestep;
    double m_Ia, m_Ib, m_Ic;
    double m_IaSamp, m_IbSamp, m_IcSamp;
    double m_Id, m_Iq;
    double m_Speed; // m/s

};

#endif // MOTORMODEL_H
