#include "hist_widget.h"
#include <QtCharts/QAbstractAxis>
#include <QtCharts/QScatterSeries>
#include <cmath>
// #include <iostream>

namespace vfhp_rviz_plugin
{

HistWidget::HistWidget(QWidget *parent):
    QWidget(parent),
    // m_series(new QSplineSeries(this)),
    m_series(new QLineSeries(this)),
    m_lowSeries(new QLineSeries(this)),
    m_highSeries(new QLineSeries(this)),
    m_axisX(new QValueAxis),
    m_axisY(new QValueAxis)
{
    setupUi(this);

    // m_series->setUseOpenGL(true);

    polarHist->addSeries(m_series);
    polarHist->setAxisX(m_axisX, m_series);
    polarHist->setAxisY(m_axisY, m_series);


    polarHist->addSeries(m_lowSeries);
    m_lowSeries->attachAxis(m_axisX);
    m_lowSeries->attachAxis(m_axisY);

    polarHist->addSeries(m_highSeries);
    m_highSeries->attachAxis(m_axisX);
    m_highSeries->attachAxis(m_axisY);

    // m_axisX->setLabelsVisible(false);
    // m_axisY->setLabelsVisible(false);

    //diagrama();

}



void HistWidget::updatePoints(const QVector<QPointF> &pointsPolar)
{
    int points = pointsPolar.size();

    QVector<QPointF> pointsCart;
    pointsCart.reserve(points+1);

    for (int i = 0; i < points; i++) {
        double angle = pointsPolar[i].x();
        double radius = pointsPolar[i].y();
        double x = radius*std::cos(angle);
        double y = radius*std::sin(angle);

        pointsCart.append(QPointF(x,y));
    }
    pointsCart.append(pointsCart.first());

    m_series->replace(pointsCart);
}

void HistWidget::setLimits(const qreal &low, const qreal &high)
{
    qreal max = high*2.0;

    m_axisX->setRange(-max,max);
    m_axisY->setRange(-max,max);

    m_lowSeries->clear();
    m_highSeries->clear();

    for (int i = 0; i <= 25; i++) {
        qreal tcos = std::cos(i*M_PI*2/25);
        qreal tsin = std::sin(i*M_PI*2/25);
        m_lowSeries->append(low*tcos, low*tsin);
        m_highSeries->append(high*tcos, high*tsin);
    }

}

void HistWidget::diagrama()
{

    int points = 32;
    QVector<QPointF> data;
    data.reserve(points);

    for (int i = 0; i < points; i++) {
        double angle = i*M_PI*2/points;
        double radius = 0.5;

        data.append(QPointF(angle, radius));
    }

    this->updatePoints(data);

    this->setLimits(0, 1);


}
}
