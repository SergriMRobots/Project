#include "spacenavform.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  QApplication a(argc, argv);
  ros::init(argc, argv, "spacenav_sim");

  spaceNavForm mainDisp;
  mainDisp.show();

  return a.exec();
}
