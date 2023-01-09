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

#include <QtMath>
#include "idiqgraph.h"
#include "params.h"

IdIqGraph::IdIqGraph(QString name, QWidget *parent) : DataGraph (name, parent)
{
    addSeries("Is (A)",left, 10);
}

void IdIqGraph::updateGraph(bool isAmps)
{
    double d, q, s;
    QList<QPointF> list;

    //add is circle
    if(isAmps)
    {
        updateSeries("Is (A)",left, 10);
        s = Param::GetFloat(Param::throtcur) * 100;
    }
    else
    {
        updateSeries("Vs (V)",left, 10);
        s = 1.1547 * Param::GetFloat(Param::udc) * 0.5;
    }

    for(double ang = 0;ang < 360;ang++)
    {
        d = s * qCos(qDegreesToRadians(ang));
        q = s * qSin(qDegreesToRadians(ang));
        list.append(QPointF(d, q));
    }
    addDataPoints(list, 10);

    DataGraph::updateGraph();

    s = s * 1.1;
    DataGraph::updateXaxis(-s, s);
    DataGraph::updateLeftYaxis(-s, s);
}
