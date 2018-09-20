
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFrame>
#include <QObject>

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

        HistWidget *histWraw = new HistWidget;
        HistWidget *histWbin = new HistWidget;
        HistWidget *histWmasked = new HistWidget;

        histWraw->labelTitle->setText("Polar Histogam");
        histWbin->labelTitle->setText("Binary Histogam");
        histWmasked->labelTitle->setText("Masked Histogam");


        QFrame *line1 = new QFrame;
        line1->setObjectName(QStringLiteral("line1"));
        line1->setFrameShape(QFrame::HLine);
        line1->setFrameShadow(QFrame::Sunken);

        QFrame *line2 = new QFrame;
        line2->setObjectName(QStringLiteral("line2"));
        line2->setFrameShape(QFrame::HLine);
        line2->setFrameShadow(QFrame::Sunken);

        layout->addWidget(histWraw);
        layout->addWidget(line1);
        layout->addWidget(histWbin);
        layout->addWidget(line2);
        layout->addWidget(histWmasked);

        setLayout(layout);

        connect(histWraw->lineEditTopic, &QLineEdit::editingFinished, this, &HistogramPanel::updateRawTopic);
        connect(histWbin->lineEditTopic, &QLineEdit::editingFinished, this, &HistogramPanel::updateBinTopic);
        connect(histWmasked->lineEditTopic, &QLineEdit::editingFinished, this, &HistogramPanel::updateMaskedTopic);

    }

    void HistogramPanel::setRawHistTopic(const QString& topic)
    {
        if (raw_topic_ != topic)
        {
            raw_topic_ = topic;

            if (raw_topic_ == "") {
                raw_hist_sub_.shutdown();
            } else {
                std::cout << "Setting raw topic to: " << topic.toStdString() << std::endl;
                raw_hist_sub_ = nh_.subscribe(raw_topic_.toStdString(), 5, &HistogramPanel::updateRawHistogramData, this);
            }
            Q_EMIT configChanged();
        }
    }

    void HistogramPanel::setBinHistTopic(const QString& topic)
    {
        if (bin_topic_ != topic)
        {
            bin_topic_ = topic;

            if (bin_topic_ == "") {
                bin_hist_sub_.shutdown();
            } else {
                std::cout << "Setting bin topic to: " << topic.toStdString() << std::endl;
                bin_hist_sub_ = nh_.subscribe(bin_topic_.toStdString(), 5, &HistogramPanel::updateBinHistogramData, this);
            }
            Q_EMIT configChanged();
        }
    }

    void HistogramPanel::setMaskedHistTopic(const QString& topic)
    {
        if (masked_topic_ != topic)
        {
            masked_topic_ = topic;

            if (masked_topic_ == "") {
                masked_hist_sub_.shutdown();
            } else {
                std::cout << "Setting masked topic to: " << topic.toStdString() << std::endl;
                masked_hist_sub_ = nh_.subscribe(masked_topic_.toStdString(), 5, &HistogramPanel::updateMaskedHistogramData, this);
            }
            Q_EMIT configChanged();
        }
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
        QLineEdit *sender;
        sender = (QLineEdit*)QObject::sender();
        setRawHistTopic(sender->text());
        std::cout << "HOLI1" << std::endl;
    }
    void HistogramPanel::updateBinTopic()
    {
        QLineEdit *sender;
        sender = (QLineEdit*)QObject::sender();
        setBinHistTopic(sender->text());
        std::cout << "HOLI2" << std::endl;
    }
    void HistogramPanel::updateMaskedTopic()
    {
        QLineEdit *sender;
        sender = (QLineEdit*)QObject::sender();
        setMaskedHistTopic(sender->text());
        std::cout << "HOLI3" << std::endl;
    }

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
