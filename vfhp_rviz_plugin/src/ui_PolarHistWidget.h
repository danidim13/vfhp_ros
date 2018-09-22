/********************************************************************************
** Form generated from reading UI file 'PolarHistWidgetG15778.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef POLARHISTWIDGETG15778_H
#define POLARHISTWIDGETG15778_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
// #include <QtWidgets/QGraphicsView>
#include <QtCharts/QPolarChart>
#include <QtCharts/QChartView>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>


QT_CHARTS_USE_NAMESPACE
QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QLabel *labelTitle;
    QHBoxLayout *horizontalLayout;
    QLabel *labelTopic;
    QLineEdit *lineEditTopic;
    // QGraphicsView *histogram;
    QPolarChart *polarHist;
    QChartView *chartView;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QStringLiteral("Form"));
        Form->resize(304, 260);
        verticalLayout_2 = new QVBoxLayout(Form);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        labelTitle = new QLabel(Form);
        labelTitle->setObjectName(QStringLiteral("labelTitle"));
        QFont font;
        font.setPointSize(14);
        labelTitle->setFont(font);
        labelTitle->setFrameShape(QFrame::NoFrame);
        labelTitle->setTextFormat(Qt::AutoText);
        labelTitle->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(labelTitle);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        labelTopic = new QLabel(Form);
        labelTopic->setObjectName(QStringLiteral("labelTopic"));

        horizontalLayout->addWidget(labelTopic);

        lineEditTopic = new QLineEdit(Form);
        lineEditTopic->setObjectName(QStringLiteral("lineEditTopic"));

        horizontalLayout->addWidget(lineEditTopic);


        verticalLayout->addLayout(horizontalLayout);

        // histogram = new QGraphicsView(Form);
        // histogram->setObjectName(QStringLiteral("histogram"));

        chartView = new QChartView(Form);
        chartView->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
        polarHist = new QPolarChart;
        polarHist->setObjectName(QStringLiteral("histogram"));

        // verticalLayout->addWidget(histogram);
        chartView->setChart(polarHist);
        verticalLayout->addWidget(chartView);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", Q_NULLPTR));
        labelTitle->setText(QApplication::translate("Form", "Title", Q_NULLPTR));
        labelTopic->setText(QApplication::translate("Form", "Topic:", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // POLARHISTWIDGETG15778_H
