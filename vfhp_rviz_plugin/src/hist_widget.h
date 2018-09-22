#ifndef HIST_WIDGET_H
#define HIST_WIDGET_H

#include <QWidget>
#include "ui_PolarHistWidget.h"

namespace vfhp_rviz_plugin {

class HistWidget: public QWidget, public Ui::Form
{
    Q_OBJECT
    public:

        explicit HistWidget(QWidget *parent = 0);

        
    private:
        void diagrama();



};

}

#endif
