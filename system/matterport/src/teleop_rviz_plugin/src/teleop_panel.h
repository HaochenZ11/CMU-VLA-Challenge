#ifndef TELEOP_PANEL_H
#define TELEOP_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>

# include <rviz/panel.h>
#endif

#include <stdio.h>

#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPushButton>
#include <QCheckBox>
#include <QFileDialog>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

class QLineEdit;

namespace teleop_rviz_plugin
{

class DriveWidget;

class TeleopPanel: public rviz::Panel
{
Q_OBJECT
public:
  TeleopPanel( QWidget* parent = 0 );

public Q_SLOTS:
  void setVel( float linear_velocity_, float angular_velocity_, bool mouse_pressed_ );

protected Q_SLOTS:
  void pressButton1();
  void sendVel();

protected:
  DriveWidget* drive_widget_;
  ros::Publisher velocity_publisher_;
  ros::NodeHandle nh_;

  QPushButton *push_button_1_;

  float linear_velocity_;
  float angular_velocity_;
  bool mouse_pressed_;
  bool mouse_pressed_sent_;
};

}

#endif // TELEOP_PANEL_H
