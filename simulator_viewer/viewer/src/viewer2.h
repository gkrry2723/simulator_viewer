#ifndef XYVIEWER2_H
#define XYVIEWER2_H

#include <QWidget>
#include <QMainWindow>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QImage>
#include <QPushButton>
#include <QProgressBar>
#include <QToolBar>
#include <QKeyEvent>
#include <QFileDialog>

#include <ros/ros.h>
#include <ros/console.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

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
using boost::bind;

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
}

namespace image_transport
{
class ImageTransport;
class Subscriber;
}

class Viewer2: public QMainWindow
{
Q_OBJECT
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
public:
  Viewer2( QWidget* parent = 0 );
  ~Viewer2();

  void showImage(cv::Mat img, int num, int w, int h);
  void image_callback(const sensor_msgs::ImageConstPtr& msg, int num);
  std::string getTopic(std::string param_name, int num);
  
  QGridLayout* left_group;
  QGridLayout* right_group;
  QGridLayout* main_layout;

  QVBoxLayout* left;
  QVBoxLayout* right;

  QLabel* imageLabel;

  cv_bridge::CvImagePtr cv_ptr;

  int width = 640;
  int height = 480;

  QHBoxLayout* player_group;
  
  QPushButton* openBtn;
  QPushButton* playBtn;
  QPushButton* recordBtn;
   
  QProgressBar* progressBar;
  QString open;

  bool play = true, record = true;

  void keyReleaseEvent(QKeyEvent *event);
  // void keyPressEvent(QKeyEvent* event);

private:
  rviz::VisualizationManager* slam_manager[2];
  rviz::RenderPanel* slam_render_panel[2];
  rviz::Display* slam_map;
  rviz::Display* slam_imu;
  void on_openBtn_clicked();
  void on_recordBtn_clicked();
  void on_playBtn_clicked();
};
#endif
