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

void IdIqGraph::updateGraph(void)
{
    double id, iq;
    QList<QPointF> list;

    //add is circle
    double is = Param::GetFloat(Param::throtcur) *100;
    for(double ang = 0;ang < 360;ang++)
    {
        id = is * qCos(qDegreesToRadians(ang));
        iq = is * qSin(qDegreesToRadians(ang));
        list.append(QPointF(id, iq));
    }
    addDataPoints(list, 10);

    DataGraph::updateGraph();

    is = is * 1.1;
    DataGraph::updateXaxis(-is, is);
    DataGraph::updateLeftYaxis(-is, is);
}
