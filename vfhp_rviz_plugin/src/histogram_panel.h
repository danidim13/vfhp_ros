#ifndef HISTOGRAM_PANEL_H
#define HISTOGRAM_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
# include <vfhp_local_planner/Histogram.h>
#endif

namespace vfhp_rviz_plugin
{

class HistogramPanel: public rviz::Panel
{
    Q_OBJECT
    public:

        HistogramPanel(QWidget* parent = 0);

        virtual void load(const rviz::Config& config);
        virtual void save(const rviz::Config config) const;


    public Q_SLOTS:
        void setRawHistTopic(const QString& topic);
        // void setBinHistTopic(const QString& topic);
        // void setMaskedHistTopic(const QString& topic);
        void updateRawHistogramData(const vfhp_local_planner::Histogram::ConstPtr& msg);
        // void updateBinHistogramData(const vfhp_local_planner::Histogram::ConstPtr& msg);
        // void updateMaskedHistogramData(const vfhp_local_planner::Histogram::ConstPtr& msg);

    protected Q_SLOTS:
        void updateRawTopic();
        // void updateBinTopic();
        // void updateMaskedTopic();
        //
    Q_SIGNALS:
        void histDataReceived(QVector<QPointF> newData);
        void histThreshChanged(qreal low, qreal high);

    protected:

        QString raw_topic_;
        // QString bin_topic_;
        // QString masked_topic_;

        ros::NodeHandle nh_;
        ros::Subscriber raw_hist_sub_, bin_hist_sub_, masked_hist_sub_;

        vfhp_local_planner::Histogram raw_hist_;
        // vfhp_local_planner::Histogram bin_hist_;
        // vfhp_local_planner::Histogram masked_hist_;


};

}

#endif
