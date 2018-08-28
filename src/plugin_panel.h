#include <QtWidgets>

#include <rviz/panel.h>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_client.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include "waypoint_table_model.h"


class PluginPanel: public rviz::Panel
{
    Q_OBJECT

public:
    PluginPanel(QWidget* parent = 0);

protected:
    QVBoxLayout* _vbox;
    QHBoxLayout* _hbox1;

    QPushButton* _button1;
    QPushButton* _button2;
    QPushButton* _button3;

    QListView* _listview;
    QStringListModel* _stringlist1;

    QTableView* _tableview;
    QTableWidget* _tablewidget;
    WaypointModel* _waypointmodel;

    

    // The ROS node handle.
    ros::NodeHandle nh_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
    boost::shared_ptr<interactive_markers::InteractiveMarkerClient> _client;
    // interactive_markers::InteractiveMarkerClient* _client;

private:
    // void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void make6DofMarker(std::string name, bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof );

private Q_SLOTS:
    void button1_on_click();
    void button2_on_click();
    void button3_on_click();

};