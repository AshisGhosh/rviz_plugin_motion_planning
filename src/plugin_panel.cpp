#include <pluginlib/class_list_macros.h>

#include "plugin_panel.h"
#include <string>
#include <iostream>
#include <fstream>



WaypointModel* g_waypointmodel = new WaypointModel();

PluginPanel::PluginPanel(QWidget* parent):
    rviz::Panel(parent)
{
    _vbox = new QVBoxLayout();
    _hbox1 = new QHBoxLayout();

    _button1 = new QPushButton(tr("Add"));
    _button2 = new QPushButton(tr("Delete"));
    _button3 = new QPushButton(tr("Load Path"));
    _button4 = new QPushButton(tr("Update Path"));
    _button5 = new QPushButton(tr("Load Waypoints"));
    _button6 = new QPushButton(tr("Save Waypoints"));


    _button1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _button2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    _button3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    _button4->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    _button5->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    _button6->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    connect(_button1, SIGNAL (released()), this, SLOT (button1_on_click()));
    connect(_button2, SIGNAL (released()), this, SLOT (button2_on_click()));

    connect(_button3, SIGNAL (released()), this, SLOT (button3_on_click()));

    connect(_button4, SIGNAL (released()), this, SLOT (button4_on_click()));

    connect(_button5, SIGNAL (released()), this, SLOT (button5_on_click()));
    connect(_button6, SIGNAL (released()), this, SLOT (button6_on_click()));


    _listview = new QListView();

    _stringlist1 = new QStringListModel();

    QStringList list;
    list << "a" << "b" << "c";
    _stringlist1->setStringList(list);

    _listview->setModel(_stringlist1);
    _listview->
            setEditTriggers(QAbstractItemView::AnyKeyPressed |
                            QAbstractItemView::DoubleClicked);


   

    Waypoint waypoint = {};
    QList<Waypoint> waypoints;
    waypoints.push_back(waypoint);
    // waypointmodel = g_waypointmodel;
    waypointmodel = new WaypointModel(waypoints);
    // waypointmodel->setWaypoints(waypoints);

    _tableview = new QTableView();
    _tableview->setModel(waypointmodel); 

    _lineedit1 = new QLineEdit();
    _lineedit2 = new QLineEdit();   

    _hbox1->addWidget(_button1);
    _hbox1->addWidget(_button2);
    
    _vbox->addWidget(_tableview);
    _vbox->addLayout(_hbox1);
    _vbox->addWidget(_button3);
    _vbox->addWidget(_button4);
    _vbox->addWidget(_lineedit1); 
    _vbox->addWidget(_button5);
    _vbox->addWidget(_lineedit2); 
    _vbox->addWidget(_button6);
    
    setLayout(_vbox);
    
    // tf::Transformer tf;
    // _client.reset(new interactive_markers::InteractiveMarkerClient(tf, "base_link", "create_markers")) ;
    // _client->setUpdateCb(&PluginPanel::processFeedback);
    server.reset( new interactive_markers::InteractiveMarkerServer("create_markers","",false) );

    ros::Duration(0.1).sleep();

    

}

// Add
void PluginPanel::button1_on_click()
{
    int row = waypointmodel->rowCount(QModelIndex());

    // Enable add one or more rows
    waypointmodel->insertRows(row,1);

    // TODO: Figure out indexing
    // Get the row for Edit mode
    QModelIndex index = waypointmodel->index(row,0, QModelIndex());

    // Enable item selection and put it edit mode
    _tableview->setCurrentIndex(index);
    _tableview->edit(index);
}

// Delete
void PluginPanel::button2_on_click()
{
    waypointmodel->removeRows(_tableview->currentIndex().row(),1);
}

// Load path (Update visualization on click)
void PluginPanel::button3_on_click()
{
    server->clear();
    server->applyChanges();
    
    visualization_msgs::InteractiveMarker int_line_strip;
    visualization_msgs::InteractiveMarkerControl int_line_strip_control;

    int_line_strip.scale = 0.25;
    int_line_strip.name = "line_strip";
    int_line_strip.description = "Just a line strip";
    int_line_strip.header.frame_id = "world";

    
    visualization_msgs::Marker line_strip;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = int_line_strip.scale * 0.02;
    line_strip.scale.y = int_line_strip.scale * 0.02;

    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;
    
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;


    int count = waypointmodel->rowCount(QModelIndex());
    ROS_INFO_STREAM("Path has " << count << " waypoints.");
    QList<Waypoint> waypoints = waypointmodel->getWaypoints();
    for (QList<Waypoint>::iterator it = waypoints.begin(); it != waypoints.end(); ++it){
        ROS_INFO_STREAM(it->x << it->y << it->z);

        int i = it - waypoints.begin();
        tf::Vector3 position;
        tf::Quaternion orientation;
        position = tf::Vector3( it->x, it->y, it->z);
        orientation = tf::Quaternion( it->qx, it->qy, it->qz, it->qw);
        char name[50];
        sprintf(name,"%d",i);
        make6DofMarker(name, false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, orientation, true );
        
        geometry_msgs::Point p;
        p.x = it->x;
        p.y = it->y;
        p.z = it->z;
        line_strip.points.push_back(p);
        server->applyChanges();        
    }

    int_line_strip_control.markers.push_back(line_strip);
    int_line_strip_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    int_line_strip_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;
    int_line_strip_control.always_visible = true;
    int_line_strip.controls.push_back(int_line_strip_control);
    server->insert(int_line_strip);
    server->applyChanges();
    
}

// Update path (update table on click)
void PluginPanel::button4_on_click()
{
  visualization_msgs::InteractiveMarker int_marker;
  // QList<Waypoint> waypoints = waypointmodel->getWaypoints();
  for (QList<Waypoint>::iterator it = waypointmodel->waypoints.begin(); it != waypointmodel->waypoints.end(); ++it){
        int i = it - waypointmodel->waypoints.begin();
        char name[50];
        sprintf(name,"%d",i);

        bool exist = server->get(name, int_marker);
        // ROS_INFO_STREAM(exist);
        it->x = int_marker.pose.position.x;
        it->y = int_marker.pose.position.y;
        it->z = int_marker.pose.position.z;
        it->qx = int_marker.pose.orientation.x;
        it->qy = int_marker.pose.orientation.y;
        it->qz = int_marker.pose.orientation.z;
        it->qw = int_marker.pose.orientation.w;
        
    } 
  Q_EMIT (waypointmodel->layoutChanged());
}

// Helper function to skip headers in csv
void skip_headers(std::ifstream &infile){
    int header_length = 13;
    std::string header_temp;
    for (int i = 0; i < header_length; i++){
        infile >> header_temp;
    }
};

// Load from file
void PluginPanel::button5_on_click()
{
  std::string line;
  ROS_INFO_STREAM("Reading file.\n");
  QString filepath= _lineedit1->text();
  ROS_INFO_STREAM(filepath.toUtf8().constData());
  std::ifstream infile(filepath.toUtf8().constData());

  QList<Waypoint> waypoints;
  float j1, j2, j3, j4, j5, j6, x_pos, y_pos, z_pos, qx, qy, qz, qw;
  
  if (infile.is_open())
  {
      skip_headers(infile);
      while (infile >> j1 >> j2 >> j3 >> j4 >> j5 >> j6 >> x_pos >> y_pos >> z_pos >> qx >> qy >> qz >> qw)
      {
          Waypoint waypoint;
          std::cout << j1<<j2<<j3<<j4<<j5<<j6<<x_pos<<y_pos<<z_pos<<qx<<qy << qz << qw << std::endl;
          waypoint.x = x_pos;
          waypoint.y = y_pos;
          waypoint.z = z_pos;
          waypoint.qx = qx;
          waypoint.qy = qy;
          waypoint.qz = qz;
          waypoint.qw = qw;
          waypoints.push_back(waypoint);
      }
      waypointmodel->waypoints = waypoints;
      Q_EMIT (waypointmodel->layoutChanged());

  }
  else std::cout << "Unable to open file\n";

}

// Save to file
void PluginPanel::button6_on_click()
{
  std::string line;
  ROS_INFO_STREAM("Saving file.\n");
  QString filepath= _lineedit2->text();
  ROS_INFO_STREAM(filepath.toUtf8().constData());
  std::ofstream outfile(filepath.toUtf8().constData());

  QList<Waypoint> waypoints = waypoints;
  float j1, j2, j3, j4, j5, j6, x_pos, y_pos, z_pos, qx, qy, qz, qw;
  
  if (outfile.is_open())
  {
    outfile << "x " << "y " << "z " << "qx " << "qy " << "qz " << "qw" <<std::endl;

      for (QList<Waypoint>::iterator it = waypointmodel->waypoints.begin(); it != waypointmodel->waypoints.end(); ++it)
      {
        outfile << it->x << it->y << it->z << it->qx << it->qy << it->qz << it->qw <<std::endl; 
      } 

  }
  else std::cout << "Unable to open file\n";
}



// Load points from file
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

  std::stringstream m_name(feedback->marker_name);
  int i = 0;
  m_name >> i;
  // QList<Waypoint> waypoints = this->waypoints;
  // QList<Waypoint>* waypoints = g_waypoints;
  // QList<Waypoint>::iterator it = waypoints->begin() + i;
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

  float scale_factor = 0.85;
  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * scale_factor;
  marker.scale.y = msg.scale * scale_factor;
  marker.scale.z = msg.scale * scale_factor;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  // control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(6DOF)%
void PluginPanel::make6DofMarker(std::string name, bool fixed, unsigned int interaction_mode, const tf::Vector3& position, const tf::Quaternion& orientation, bool show_6dof )
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "world";
  tf::pointTFToMsg(position, int_marker.pose.position);
  tf::quaternionTFToMsg(orientation, int_marker.pose.orientation);
  int_marker.scale = 0.025;

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
      // int_marker.name += "_" + mode_text;
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