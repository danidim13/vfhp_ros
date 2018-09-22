#include "hist_widget.h"
#include <QtCharts/QAbstractAxis>
#include <QtCharts/QValueAxis>
#include <QtCharts/QSplineSeries>
// #include <iostream>

namespace vfhp_rviz_plugin
{

HistWidget::HistWidget(QWidget *parent):
     QWidget(parent)
 {
     setupUi(this);

     diagrama();


 }

void HistWidget::diagrama()
{
    QSplineSeries *series = new QSplineSeries();
    //series->setName("Densidad");
    int points = 32;
    for (int i = points; i >= 0; i--) {
        double angle = i*3.1415*2/points;
        double radius = i*2;

        series->append(angle, radius);
    }

    this->polarHist->addSeries(series);


    QValueAxis *angularAxis = new QValueAxis();
    // angularAxis->setTickCount(points);
    this->polarHist->addAxis(angularAxis, QPolarChart::PolarOrientationAngular);

    QValueAxis *radialAxis = new QValueAxis();
    // angularAxis->setTickCount(points);
    this->polarHist->addAxis(radialAxis, QPolarChart::PolarOrientationRadial);



    series->attachAxis(radialAxis);
    series->attachAxis(angularAxis);


    // radialAxis->setRange(0, 50);
    // angularAxis->setRange(0, 360);

}

}
