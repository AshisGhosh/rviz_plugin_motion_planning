#include <pluginlib/class_list_macros.h>

#include "plugin_panel.h"
#include <string>





PluginPanel::PluginPanel(QWidget* parent):
    rviz::Panel(parent)
{
    _vbox = new QVBoxLayout();
    _hbox1 = new QHBoxLayout();

    _button1 = new QPushButton(tr("Add"));
    _button2 = new QPushButton(tr("Delete"));
    _button3 = new QPushButton(tr("Load Path"));

    _button1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _button2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    _button3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect(_button1, SIGNAL (released()), this, SLOT (button1_on_click()));
    connect(_button2, SIGNAL (released()), this, SLOT (button2_on_click()));

    connect(_button3, SIGNAL (released()), this, SLOT (button3_on_click()));

    _listview = new QListView();

    _stringlist1 = new QStringListModel();

    QStringList list;
    list << "a" << "b" << "c";
    _stringlist1->setStringList(list);

    _listview->setModel(_stringlist1);
    _listview->
            setEditTriggers(QAbstractItemView::AnyKeyPressed |
                            QAbstractItemView::DoubleClicked);

    _tableview = new QTableView();

    _waypointmodel = new WaypointModel();
    // _waypointmodel->Columns[0].clear();

    _tableview->setModel(_waypointmodel);

    _tablewidget = new QTableWidget();
    

    _hbox1->addWidget(_button1);
    _hbox1->addWidget(_button2);
    
    _vbox->addWidget(_tableview);
    _vbox->addLayout(_hbox1);
    _vbox->addWidget(_button3);
    

    
    setLayout(_vbox);
    
    // tf::Transformer tf;
    // _client.reset(new interactive_markers::InteractiveMarkerClient(tf, "base_link", "create_markers")) ;
    // _client->setUpdateCb(&PluginPanel::processFeedback);
    server.reset( new interactive_markers::InteractiveMarkerServer("create_markers","",false) );

    ros::Duration(0.1).sleep();

    

}

void PluginPanel::button1_on_click()
{
    int row = _stringlist1->rowCount();

    // Enable add one or more rows
    _stringlist1->insertRows(row,1);

    // Get the row for Edit mode
    QModelIndex index = _stringlist1->index(row);

    // Enable item selection and put it edit mode
    _tableview->setCurrentIndex(index);
    _tableview->edit(index);
}

void PluginPanel::button2_on_click()
{
    _stringlist1->removeRows(_tableview->currentIndex().row(),1);
}


void PluginPanel::button3_on_click()
{
    server->clear();
    server->applyChanges();

    int count = _stringlist1->rowCount();
    ROS_INFO_STREAM("Path has " << count << " waypoints.");

    for( int i = 0; i < count; i++ ) {
      tf::Vector3 position;
      position = tf::Vector3( i, 0, 0);
      char name[50];
      sprintf(name,"%d",i);
      make6DofMarker(name, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );
      server->applyChanges();
    }
    
    
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
      ROS_INFO_STREAM( s.str() << ": button click" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
      ROS_INFO_STREAM( s.str() << ": pose changed"
          << "\nposition = "
          << feedback->pose.position.x
          << ", " << feedback->pose.position.y
          << ", " << feedback->pose.position.z
          << "\norientation = "
          << feedback->pose.orientation.w
          << ", " << feedback->pose.orientation.x
          << ", " << feedback->pose.orientation.y
          << ", " << feedback->pose.orientation.z
          << "\nframe: " << feedback->header.frame_id
          << " time: " << feedback->header.stamp.sec << "sec, "
          << feedback->header.stamp.nsec << " nsec" );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
      ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
      ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
      break;
  }
}

using namespace visualization_msgs;

interactive_markers::MenuHandler menu_handler;

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(6DOF)%
void PluginPanel::make6DofMarker(std::string name, bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = name;
  int_marker.description = "Simple 6-DOF Control";

  // insert a box
  makeBoxControl(int_marker);

  int_marker.controls[0].interaction_mode = interaction_mode;

  InteractiveMarkerControl control;

  if ( fixed )
  {
    int_marker.name += "_fixed";
    int_marker.description += "\n(fixed orientation)";
    control.orientation_mode = InteractiveMarkerControl::FIXED;
  }

  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
  {
      std::string mode_text;
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
      if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
      int_marker.name += "_" + mode_text;
      int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
  }

  if(show_6dof)
  {
    tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
    orien.normalize();
    tf::quaternionTFToMsg(orien, control.orientation);
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  }

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
  if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    menu_handler.apply( *server, int_marker.name );
}
// %EndTag(6DOF)%

PLUGINLIB_EXPORT_CLASS(PluginPanel, rviz::Panel)