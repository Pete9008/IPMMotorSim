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

#include "datagraph.h"
#include <QtCore/QRandomGenerator>
#include <QtCore/QtMath>
#include <QtCharts/QValueAxis>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <limits>

DataGraph::DataGraph(QWidget *parent) : QMainWindow(parent)
{
    minY =  std::numeric_limits<double>::max();
    maxY =  std::numeric_limits<double>::lowest();
    minX =  std::numeric_limits<double>::max();
    maxX =  std::numeric_limits<double>::lowest();

    m_chart = new Chart();
    m_chart->legend()->show();

    m_chartView = new ChartView(m_chart);
    m_chartView->setRenderHint(QPainter::Antialiasing);

    setCentralWidget(m_chartView);
    resize(1600, 300);
    grabGesture(Qt::PanGesture);
    grabGesture(Qt::PinchGesture);
    show();
}

DataGraph::~DataGraph()
{
    clearData();
}

void DataGraph::addSeries(QString legend, int key)
{
    if(m_series.contains(key))
        return;

    QList<QPointF> *series = new QList<QPointF>;
    m_series[key] = series;
    m_legends[key] = legend;
}



void DataGraph::addDataPoint(double x, double y, int key)
{
    if(m_series.contains(key))
    {
        if(y<minY) minY = y;
        if(y>maxY) maxY = y;
        if(x<minX) minX = x;
        if(x>maxX) maxX = x;

        m_series[key]->append(QPointF(x, y));
    }
}

void DataGraph::addDataPoints(QList<QPointF> pointList, int key)
{
    if(m_series.contains(key))
    {
        QListIterator<QPointF> i(pointList);
        while (i.hasNext())
        {
            QPointF p = i.next();

            if(p.y()<minY) minY = p.y();
            if(p.y()>maxY) maxY = p.y();
            if(p.x()<minX) minX = p.x();
            if(p.x()>maxX) maxX = p.x();
        }

        m_series[key]->append(pointList);
    }
}

void DataGraph::updateGraph(void)
{
    m_chart->removeAllSeries();

    QMap<int, QList<QPointF> *>::iterator i;
    for (i = m_series.begin(); i != m_series.end(); ++i)
    {
        QLineSeries *series = new QLineSeries(); //chart will take ownership of this and delete when done
        series->append(*i.value());
        m_chart->addSeries(series);
        series->setName(m_legends[i.key()]);
        if(m_colours.contains(i.key())) //if we have a colour then override standard one
            series->setColor(m_colours[i.key()]);
        if(m_opacity.contains(i.key())) //if we have an opacity then override standard one
            series->setOpacity(m_opacity[i.key()]);
    }
    m_chart->createDefaultAxes();
    m_chart->axisX()->setRange(minX, maxX);
    m_chart->axisY()->setRange(minY, maxY);
}

void DataGraph::updateXaxis(void)
{
    m_chart->axisX()->setRange(minX, maxX);
}

void DataGraph::updateYaxis(void)
{
    m_chart->axisY()->setRange(minY, maxY);
}

void DataGraph::clearData(void)
{
    minY =  std::numeric_limits<double>::max();
    maxY =  std::numeric_limits<double>::lowest();
    minX =  std::numeric_limits<double>::max();
    maxX =  std::numeric_limits<double>::lowest();
    m_chart->removeAllSeries();
    QMap<int, QList<QPointF> *>::iterator i;
    for (i = m_series.begin(); i != m_series.end(); ++i)
    {
        i.value()->clear();
    }
}

void DataGraph::setColour(QColor colour, int key)
{
    m_colours[key] = colour;
}

void DataGraph::setOpacity(qreal opacity, int key)
{
    m_opacity[key] = opacity;
}


