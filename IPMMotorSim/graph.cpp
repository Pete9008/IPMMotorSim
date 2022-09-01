#include "graph.h"


Graph::Graph(QWidget *parent) : QWidget(parent)
{
    QLineSeries *series = new QLineSeries();
    for (int i = 0; i < 50000; i++) {
        QPointF p((qreal) i, qSin(M_PI / 50 * i) * 100);
        p.ry() += QRandomGenerator::global()->bounded(20);
        *series << p;
    }

    chart = new Chart();
    chart->addSeries(series);
    chart->setTitle("Zoom in/out example");
    chart->setAnimationOptions(QChart::SeriesAnimations);
    chart->legend()->hide();
    chart->createDefaultAxes();
    //ui->graphicsView->setChart(chart);

    chartView = new ChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);


    chartView->resize(1600, 900);
    chartView->grabGesture(Qt::PanGesture);
    chartView->grabGesture(Qt::PinchGesture);
    chartView->show();
}


