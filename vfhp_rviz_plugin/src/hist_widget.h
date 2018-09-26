#ifndef HIST_WIDGET_H
#define HIST_WIDGET_H

#include <QWidget>
#include <QtCharts/QValueAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QLineSeries>
#include "ui_PolarHistWidget.h"

namespace vfhp_rviz_plugin {

class HistWidget: public QWidget, public Ui::Form
{
    Q_OBJECT
    public:

        explicit HistWidget(QWidget *parent = 0);

    public Q_SLOTS:
        void updatePoints(const QVector<QPointF> &pointsPolar);
        void setLimits(const qreal &low, const qreal &high);


    private:
        void diagrama();

        // QSplineSeries *m_series;
        QLineSeries *m_series, *m_lowSeries, *m_highSeries;

        QValueAxis *m_axisX;
        QValueAxis *m_axisY;



};

}

#endif
