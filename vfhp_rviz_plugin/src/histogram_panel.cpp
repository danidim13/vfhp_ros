
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFrame>

#include "histogram_panel.h"
#include "hist_widget.h"

namespace vfhp_rviz_plugin
{
    HistogramPanel::HistogramPanel(QWidget* parent):
        rviz::Panel( parent )
    {

        // std::cout << "Creating Histogam Panel" << std::endl;

        QVBoxLayout *layout = new QVBoxLayout;
        /*
        QPushButton *btn1 = new QPushButton();
        QLabel *raw_topic_label = new QLabel("Polar Histogram topic:");
        QLabel *bin_topic_label = new QLabel("Binary Histogram topic:");
        QLabel *masked_topic_label = new QLabel("Masked Histogram topic:");
        layout->addWidget(btn1);
        layout->addWidget(raw_topic_label);
        layout->addWidget(bin_topic_label);
        layout->addWidget(masked_topic_label);
        */

        HistWidget *hist1 = new HistWidget;
        HistWidget *hist2 = new HistWidget;
        HistWidget *hist3 = new HistWidget;

        hist1->labelTitle->setText("Polar Histogam");
        hist2->labelTitle->setText("Binary Histogam");
        hist3->labelTitle->setText("Masked Histogam");


        QFrame *line1 = new QFrame;
        line1->setObjectName(QStringLiteral("line1"));
        line1->setFrameShape(QFrame::HLine);
        line1->setFrameShadow(QFrame::Sunken);

        QFrame *line2 = new QFrame;
        line2->setObjectName(QStringLiteral("line2"));
        line2->setFrameShape(QFrame::HLine);
        line2->setFrameShadow(QFrame::Sunken);

        layout->addWidget(hist1);
        layout->addWidget(line1);
        layout->addWidget(hist2);
        layout->addWidget(line2);
        layout->addWidget(hist3);

        setLayout(layout);

    }
/*
    void HistogramPanel::setRawHistTopic(const QString& topic)
    {
        if (raw_topic_ != topic)
            raw_topic_ = topic;
    }

    void HistogramPanel::setBinHistTopic(const QString& topic)
    {
        if (bin_topic_ != topic)
            bin_topic_ = topic;
    }

    void HistogramPanel::setMaskedHistTopic(const QString& topic)
    {
        if (masked_topic_ != topic)
            masked_topic_ = topic;
    }

    void HistogramPanel::updateRawHistogramData(const vfhp_local_planner::Histogram::ConstPtr& msg)
    {

    }
    void HistogramPanel::updateBinHistogramData(const vfhp_local_planner::Histogram::ConstPtr& msg)
    {

    }
    void HistogramPanel::updateMaskedHistogramData(const vfhp_local_planner::Histogram::ConstPtr& msg)
    {

    }

    void HistogramPanel::updateRawTopic()
    {

    }
    void HistogramPanel::updateBinTopic()
    {

    }
    void HistogramPanel::updateMaskedTopic()
    {

    }
*/
    void HistogramPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    void HistogramPanel::load(const rviz::Config& config)
    {
        rviz::Panel::load(config);
    }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vfhp_rviz_plugin::HistogramPanel,rviz::Panel )
