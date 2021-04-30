#include "viewer1.h"

Viewer1::Viewer1( QWidget* parent )
  : QMainWindow(parent)//, it_(nh_)
{
  // layout
  main_layout = new QHBoxLayout();
  left = new QGridLayout();
  right = new QGridLayout();
  center = new QGridLayout();

  // make, initialize 8 rviz render panels and connect them with imu rviz display
  // make panels that show camera image
  for(int i=0;i<8;i++)
  {
    // render panel
    slam_render_panel[i] = new rviz::RenderPanel();
    slam_manager[i] = new rviz::VisualizationManager( slam_render_panel[i] );
    slam_render_panel[i]->initialize( slam_manager[i]->getSceneManager(), slam_manager[i]);
    slam_manager[i]-> initialize();
    slam_manager[i]-> startUpdate();
    slam_grid[i] = slam_manager[i] -> createDisplay( "xycar_imu/Imu", "IMU", true);
    slam_grid[i]-> subProp("Topic") -> setValue(getTopic("/viewer/grid_topic", i).c_str());

    //image widget
    imageLabel[i] = new QLabel();
    imageLabel[i]->setScaledContents(true);


    //tab 
    tabs_in_viewer[i] = new QTabWidget();
    tabs_in_viewer[i]->addTab(imageLabel[i],"cam");
    tabs_in_viewer[i]->addTab(slam_render_panel[i],"IMU");
    qvbox_tab[i] = new QVBoxLayout();
    qvbox_tab[i]->addWidget(tabs_in_viewer[i]);
    space[i] = new QSpacerItem(240, 30);
  }

  // 9th render panel -> connect with map rviz display
  slam_render_panel[8] = new rviz::RenderPanel();
  slam_manager[8] = new rviz::VisualizationManager( slam_render_panel[8] );
  slam_render_panel[8]-> initialize( slam_manager[8]->getSceneManager(), slam_manager[8]);
  slam_manager[8]-> initialize();
  slam_manager[8]-> startUpdate();
  slam_grid[8] = slam_manager[8]->createDisplay( "rviz/Map", "Map", true);
  slam_grid[8] -> subProp("Topic") -> setValue(getTopic("/viewer/map_topic", 0).c_str());

  // layout
  for(int i=0;i<4;i++){
    left->addLayout(qvbox_tab[i],i,0);
    right->addLayout(qvbox_tab[i+4],i,0);
  }
  
  center -> addWidget(slam_render_panel[8], 0, 0);
  center -> setColumnMinimumWidth(0, 1000);
  center -> setRowMinimumHeight(0, 1000);
  //slam_render_panel[8] -> setFixedSize(900,900);

  main_layout -> addLayout(left);
  main_layout -> addLayout(center);
  main_layout -> addLayout(right);
  
  QWidget *widget = new QWidget();

  QSizePolicy p = slam_render_panel[8]->sizePolicy();
  p.setHeightForWidth( true );
  slam_render_panel[8]->setSizePolicy( p );
  
  widget->setLayout(main_layout);
  setCentralWidget(widget);

  Sub = nh_.subscribe("xycar_sensor", 1, &Viewer1::callback, this);

  this->setMinimumWidth(1700);
  this->setMinimumHeight(1000);
  this->showMinimized();
  std::cout << imageLabel[0]->height() << " : " << imageLabel[0]->width() << "\n";
}

void Viewer1::callback(const xycar_data::xycar_sensor& msg){
  cv::Mat img = cv::imdecode(cv::Mat(msg.camera.data),1);
  cv::Mat transfer_img;

  if (img.data)
  {
    cv::cvtColor(img, transfer_img, CV_BGR2RGB);
    QPixmap p = QPixmap::fromImage(QImage((unsigned char*)transfer_img.data, transfer_img.cols, transfer_img.rows, QImage::Format_RGB888));
    imageLabel[msg.car_num]->setPixmap(p);
  } 
}

void Viewer1::resizeEvent(QResizeEvent *)
{
    //std::cout << "Width : " << this->width() << ", Height : " <<  this->height();
    std::cout << imageLabel[0]->height() << " : " << imageLabel[0]->width() << "\n";

}

// get a topic
std::string Viewer1::getTopic(std::string param_name, int num)
{
  std::string topic;
  param_name = param_name + std::to_string(num);
  ros::param::get(param_name, topic);
  std::cout << param_name << " : " << topic.c_str() << "\n";
  return topic.c_str();
}

/*
// get images and convert images to images for cv(BGR8)
void Viewer1::image_callback(const sensor_msgs::ImageConstPtr& msg, int num){

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat img = cv_ptr->image;
  cv::Mat transfer_img;

  if (img.data)
  {
    cv::cvtColor(img, transfer_img, CV_BGR2RGB);
    QPixmap p = QPixmap::fromImage(QImage((unsigned char*)transfer_img.data, transfer_img.cols, transfer_img.rows, QImage::Format_RGB888));
    //QPixmap pp = p.scaled(imageLabel[num]->size().width(), imageLabel[num]->size().height(), Qt::KeepAspectRatio,  Qt::FastTransformation);
    //cameraImage.scaled( (uint)(imageLabel[num]->size().width()), (uint)(imageLabel[num]->size().height()), Qt::KeepAspectRatio);
    
    //QImage _img(img.data, ->size().width(), imageLabel[num]->size().height(), QImage::Format_RGB888);
    imageLabel[num]->setPixmap(p);
  } 
}
*/

//destructor
Viewer1::~Viewer1()
{
  for (int i=0;i<9;i++)
  {
    delete slam_render_panel[i];
    delete slam_manager[i];
    delete slam_grid[i];
  }

  for (int i=0;i<8;i++)
  {
    //delete widget_group[i];
    delete tabs_in_viewer[i];
    delete imageLabel[i];
  }

  delete main_layout;
  delete left;
  delete right;
  delete center;
}