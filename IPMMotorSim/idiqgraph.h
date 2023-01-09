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

#ifndef IDIQGRAPH_H
#define IDIQGRAPH_H

#include <QMainWindow>
#include "datagraph.h"

class IdIqGraph : public DataGraph
{
public:
    explicit IdIqGraph(QString name, QWidget *parent = nullptr);
    void updateGraph(bool isAmps);
};

#endif // IDIQGRAPH_H
