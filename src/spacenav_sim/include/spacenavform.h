#ifndef SPACENAVFORM_H
#define SPACENAVFORM_H

#include <QWidget>
#include <QKeyEvent>
#include <QTimer>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include "ui_spacenavform.h"
#include "controline.h"
#include "mousekeyline.h"

namespace Ui
{
class spaceNavForm;
}
class spaceNavForm : public QWidget
{
  Q_OBJECT
  QTimer buttonCheckTimer;
  QTimer sendTimer;

public:
  explicit spaceNavForm(QWidget* parent = nullptr);
  ~spaceNavForm();

private slots:

public:
private:
  //  HotKeyList ;
  HotKeyList hotkeyList;
  std::vector<ControlLine> controlLines;
  std::vector<MouseKeyLine> mouseKeyLines;

  ros::NodeHandle nh;
  ros::Publisher joyInfoPublisher;
  ros::Publisher offset_pub;
  ros::Publisher rot_offset_pub;
  ros::Publisher twist_pub;
  Ui::spaceNavForm* ui;

  bool allNull(std::vector<float>& joy, std::vector<int>& button);
  void sendJoyMsg();
  void exec();
  void keyPressEvent(QKeyEvent* event);
  void keyReleaseEvent(QKeyEvent* event);
  void mousePressEvent(QMouseEvent* event);
};

#endif  // SPACENAVFORM_H
