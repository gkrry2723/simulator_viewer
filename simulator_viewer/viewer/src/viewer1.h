#ifndef VIEWER11_H
#define VIEWER11_H

#include <QWidget>
#include <QMainWindow>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QImage>
#include <QToolBar>
#include <QSpacerItem>

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <string>
#include <boost/bind.hpp>
#include <boost/function.hpp>

#include <xycar_data/xycar_sensor.h>

using boost::bind;

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

//namespace image_transport
//{
//class ImageTransport;
//class Subscriber;
//}

class Viewer1: public QMainWindow
{
Q_OBJECT
  ros::NodeHandle nh_;
  //ros::NodeHandle nh1;
  //image_transport::ImageTransport it_;
  //image_transport::Subscriber image_sub_;
  ros::Subscriber Sub;

public:
  Viewer1(QWidget* parent = 0);
  ~Viewer1();

  //void showImage(cv::Mat img, int num);
  //void image_callback(const sensor_msgs::ImageConstPtr& msg, int num);
  void callback(const xycar_data::xycar_sensor& msg);
  std::string getTopic(std::string param_name, int num);

  
  QHBoxLayout* main_layout;
  QGridLayout* left;
  QGridLayout* right;
  //QVBoxLayout* left;
  //QVBoxLayout* right;
  QGridLayout* center;

  QGridLayout* widget_group[8];
  QLabel* imageLabel[8];
  QTabWidget* tabs_in_viewer[8];
  QVBoxLayout* qvbox_tab[8];
  QSpacerItem* space[8];
  QVBoxLayout* space_lay[8];
  //cv_bridge::CvImagePtr cv_ptr[8];

  void resizeEvent(QResizeEvent*);

  int panel_w = 100;
  int panel_h = 100;
  int start = true;
  //int width = 200;
  //int height = 200;

private:
  rviz::VisualizationManager* slam_manager[9];
  rviz::RenderPanel* slam_render_panel[9];
  rviz::Display* slam_grid[9];
};
#endif