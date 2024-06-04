#include "spacenavform.h"

spaceNavForm::spaceNavForm(QWidget* parent) : QWidget(parent), ui(new Ui::spaceNavForm)
{
  ui->setupUi(this);
  controlLines = { { hotkeyList, *ui->slider0, *ui->sb0, *ui->setZeroButton0, *ui->isSetableToZero0, *ui->decrementKey0,
                     *ui->incrementKey0 },
                   { hotkeyList, *ui->slider1, *ui->sb1, *ui->setZeroButton1, *ui->isSetableToZero1, *ui->decrementKey1,
                     *ui->incrementKey1 },
                   { hotkeyList, *ui->slider2, *ui->sb2, *ui->setZeroButton2, *ui->isSetableToZero2, *ui->decrementKey2,
                     *ui->incrementKey2 },
                   { hotkeyList, *ui->slider3, *ui->sb3, *ui->setZeroButton3, *ui->isSetableToZero3, *ui->decrementKey3,
                     *ui->incrementKey3 },
                   { hotkeyList, *ui->slider4, *ui->sb4, *ui->setZeroButton4, *ui->isSetableToZero4, *ui->decrementKey4,
                     *ui->incrementKey4 },
                   { hotkeyList, *ui->slider5, *ui->sb5, *ui->setZeroButton5, *ui->isSetableToZero5, *ui->decrementKey5,
                     *ui->incrementKey5 } };

  mouseKeyLines = { { hotkeyList, *ui->leftMouseButton, *ui->lmbIndicator, *ui->lmbHotkeyLineEdit },
                    { hotkeyList, *ui->rightMouseButton, *ui->rmbIndicator, *ui->rmbHotkeyLineEdit } };
  this->setFocus();
  sendTimer.setInterval(8);
  buttonCheckTimer.setInterval(100);

  joyInfoPublisher = nh.advertise<sensor_msgs::Joy>(ui->topicLineEdit->text().toStdString(), 10);
  offset_pub = nh.advertise<geometry_msgs::Vector3>("spacenav/offset", 2);
  rot_offset_pub = nh.advertise<geometry_msgs::Vector3>("spacenav/rot_offset", 2);
  twist_pub = nh.advertise<geometry_msgs::Twist>("spacenav/twist", 2);
  connect(&buttonCheckTimer, &QTimer::timeout, this, &spaceNavForm::exec);

  connect(&sendTimer, &QTimer::timeout, this, &spaceNavForm::sendJoyMsg);
  sendTimer.start();
  buttonCheckTimer.start();
}

spaceNavForm::~spaceNavForm()
{
  delete ui;
}

bool spaceNavForm::allNull(std::vector<float>& joy, std::vector<int>& button)
{
  for (auto& value : joy)
  {
    if (value != 0.0f)
      return false;
  }
  for (auto& value : button)
  {
    if (value != 0)
      return false;
  }
  return true;
}

void spaceNavForm::sendJoyMsg()
{
  sensor_msgs::Joy joystick_msg;
  float linear_scale[3];
  float angular_scale[3];
  joystick_msg.axes.resize(6);
  joystick_msg.buttons.resize(2);
  for (auto i = 0; i < controlLines.size(); i++)
  {
    float value = static_cast<float>(controlLines[i].GetValue());
    if (i < 3)
    {
      linear_scale[i] = value;
    }
    else
    {
      angular_scale[i - 3] = value;
    }
    joystick_msg.axes[i] = value;
  }
  for (auto i = 0; i < mouseKeyLines.size(); i++)
  {
    joystick_msg.buttons[i] = mouseKeyLines[i].GetValue();
  }
  //    if(!allNull(joystick_msg.axes,joystick_msg.buttons))
  //    {
  joystick_msg.header.stamp = ros::Time::now();
  joyInfoPublisher.publish(joystick_msg);

  geometry_msgs::Vector3 offset_msg;
  offset_msg.x = linear_scale[0];
  offset_msg.y = linear_scale[1];
  offset_msg.z = linear_scale[2];
  offset_pub.publish(offset_msg);
  geometry_msgs::Vector3 rot_offset_msg;
  rot_offset_msg.x = angular_scale[0];
  rot_offset_msg.y = angular_scale[1];
  rot_offset_msg.z = angular_scale[2];
  rot_offset_pub.publish(rot_offset_msg);

  geometry_msgs::Twist twist_msg;
  twist_msg.linear = offset_msg;
  twist_msg.angular = rot_offset_msg;
  twist_pub.publish(twist_msg);
  //    }
}

void spaceNavForm::exec()
{
  for (auto& line : controlLines)
  {
    line.checkKey();
  }
  for (auto& mouseKey : mouseKeyLines)
  {
    mouseKey.checkKey();
  }
}

void spaceNavForm::keyPressEvent(QKeyEvent* event)
{
  if (!event->isAutoRepeat() && event->spontaneous())
  {
    QChar symbol = HotKey::get_symbol(event->text());
    bool in = HotKey::in(symbol, hotkeyList);
    if (in)
    {
      for (auto& line : controlLines)
      {
        line.keyPressed(symbol);
      }
      for (auto& mouseKey : mouseKeyLines)
      {
        mouseKey.keyPressed(symbol);
      }
    }
  }
}

void spaceNavForm::keyReleaseEvent(QKeyEvent* event)
{
  if (!event->isAutoRepeat() && event->spontaneous())
  {
    QChar symbol = HotKey::get_symbol(event->text());
    if (HotKey::in(symbol, hotkeyList))
    {
      for (auto& line : controlLines)
      {
        line.keyReleased(symbol);
      }
      for (auto& mouseKey : mouseKeyLines)
      {
        mouseKey.keyReleased(symbol);
      }
    }
  }
}

void spaceNavForm::mousePressEvent(QMouseEvent* event)
{
  if (event->spontaneous())
  {
    this->setFocus();
  }
}
