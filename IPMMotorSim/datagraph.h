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

#ifndef DATAGRAPH_H
#define DATAGRAPH_H

#include <QMainWindow>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include "chartview.h"
#include "chart.h"

enum axisSel {left,right};

class DataGraph : public QMainWindow
{
    Q_OBJECT
public:
    explicit DataGraph(QString name, QWidget *parent = nullptr);
    ~DataGraph();
    void saveWinState();
    void addSeries(QString legend, axisSel axis, int key);
    void updateSeries(QString legend, axisSel axis, int key);
    void addDataPoint(double x, double y, int key);
    void addDataPoints(QList<QPointF> pointList, int key);
    void clearData();
    void updateGraph(void);
    void updateXaxis(double min, double max);
    void updateLeftYaxis(double min, double max);
    void setColour(QColor colour, int key);
    void setOpacity(qreal opacity, int key);
    void setAxisText(QString x, QString left, QString right);

private:
    Chart *m_chart;
    ChartView *m_chartView;
    QMap<int, QList<QPointF> *> m_series;
    QMap<int, QString> m_legends;
    QMap<int, QColor> m_colours;
    QMap<int, qreal> m_opacity;
    QMap<int, axisSel> m_axis;

    double minX, maxX, minY_L, maxY_L, minY_R, maxY_R;
    QString mName;

    QValueAxis *m_axisL;
    QValueAxis *m_axisR;
    QValueAxis *m_axisX;


signals:

public slots:
};

#endif // DATAGRAPH_H
