#include "viewer2.h"

Viewer2::Viewer2(QWidget* parent)
  : QMainWindow(parent), it_(nh_)   // it(nh) : imageTransport with node handler
{

  // layout
  main_layout = new QGridLayout();
  left = new QVBoxLayout();
  right = new QVBoxLayout();
  imageLabel = new QLabel();
  left_group = new QGridLayout();
  right_group = new QGridLayout();
  
  // make 2 rviz render panels and initialize
  for(int i=0; i<2; i++)
  {
    slam_render_panel[i] = new rviz::RenderPanel();
    slam_manager[i] = new rviz::VisualizationManager(slam_render_panel[i]);
    slam_render_panel[i]->initialize(slam_manager[i]->getSceneManager(), slam_manager[i]);
    slam_manager[i]->initialize();
    slam_manager[i]->startUpdate();
  }

  // connects the render panel1 with imu, rviz display
  slam_imu = slam_manager[0] -> createDisplay( "xycar_imu/Imu", "IMU", true);
  slam_imu -> subProp("Topic") -> setValue(getTopic("/viewer/grid_topic", 0).c_str());

  // connects the render panel2 with map, rviz display
  slam_map = slam_manager[1] -> createDisplay( "rviz/Map", "Map", true);
  slam_map -> subProp("Topic") -> setValue(getTopic("/viewer/map_topic", 0).c_str());

  // left group : imu render panel, camera image
  left_group -> addWidget(slam_render_panel[0], 0, 0);
  left_group -> setColumnMinimumWidth(0, width);
  left_group -> setRowMinimumHeight(0, height);

  left_group -> addWidget(imageLabel, 1, 0);
  left_group -> setColumnMinimumWidth(0, width);
  left_group -> setRowMinimumHeight(0, height);

  // right group : map render panel
  right_group -> addWidget(slam_render_panel[1], 0, 0);
  right_group -> setColumnMinimumWidth(0, 1200);
  right_group -> setRowMinimumHeight(0, 1000);

  left -> addLayout(left_group);
  right -> addLayout(right_group);

  // button layout(with push buttons like open file, play, record)
  player_group = new QHBoxLayout();

  // button 1 : open
  openBtn = new QPushButton();
  openBtn -> setText("Open");
  connect(openBtn, &QPushButton::clicked, this, &Viewer2::on_openBtn_clicked);

  // button 2: record
  recordBtn = new QPushButton();
  recordBtn -> setText("●");
  connect(recordBtn, &QPushButton::clicked, this, &Viewer2::on_recordBtn_clicked);

  // button 3 : play
  playBtn = new QPushButton();
  playBtn -> setText("▶");
  connect(playBtn, &QPushButton::clicked, this, &Viewer2::on_playBtn_clicked);

  progressBar = new QProgressBar();

  player_group -> addWidget(openBtn);
  player_group -> addWidget(recordBtn);
  player_group -> addWidget(playBtn);
  player_group -> addWidget(progressBar);


  // construct the main_layout
  main_layout -> addLayout(left, 0, 0);
  main_layout -> addLayout(right, 0, 1);
  main_layout -> addLayout(player_group, 1, 0, 1, 2);
  
  QWidget *widget = new QWidget();
  widget->setLayout(main_layout);
  setCentralWidget(widget);
  
  // get images from a camera
  image_sub = it_.subscribe(getTopic("/viewer/cam_topic", 0), 1, boost::bind(&Viewer2::image_callback, this, _1, 0));
}

// get a topic
std::string Viewer2::getTopic(std::string param_name, int num)
{
  std::string topic;
  param_name = param_name + std::to_string(num);
  ros::param::get(param_name, topic);
  std::cout << param_name << " : " << topic.c_str() << "\n";
  return topic.c_str();
}

// get images and convert images to images for cv(BGR8)
void Viewer2::image_callback(const sensor_msgs::ImageConstPtr& msg, int num){
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  showImage(cv_ptr->image, num, width, height);
}

// get converted BGR8 images and convert them to RGB and resize
void Viewer2::showImage(cv::Mat img, int num, int w, int h)
{
  if (img.data)
  {
    cv::cvtColor(img, img, CV_BGR2RGB);
    cv::resize(img, img, cv::Size(w, h));
    QImage _img(img.data, img.cols, img.rows, QImage::Format_RGB888);
    imageLabel->setPixmap(QPixmap::fromImage(_img));
  } 
  else 
  {
    imageLabel->setText("Cannot load the input image!");
  }
}

// slot function for the open button
// show "/home" directory and user can select a file
void Viewer2::on_openBtn_clicked()
{
  printf("openBtn clicked\n");
  open = QFileDialog::getOpenFileName(this, "Select rosbag files", "/home", "rosbag file (*.bag)");
  std::cout << open.toStdString() << "\n";
}

// slot function for the record button
// change the play status and shape (square <-> circle)
void Viewer2::on_recordBtn_clicked()
{
  printf("recordBtn clicked\n");
  this->play = !this->play;
  if(this->play == false){
    recordBtn -> setText("■");
  } else if(this->play == true){
    recordBtn -> setText("●");
  }

}

// slot function for the play button
// change the play status and shape(square <-> triangle)
void Viewer2::on_playBtn_clicked()
{
  printf("playBtn clicked\n");
  this->play = !this->play;

  if(this->play == false){  //play start
    playBtn -> setText("■");
    printf("false %d\n", play);

    //rosbag::Bag bag;
    //bag.open(open, rosbag::bagmode::Read);
    

  } else if(this->play == true){
    playBtn -> setText("▶");
    printf("true %d\n", play);
  }
}

// keyboard event handler
void Viewer2::keyReleaseEvent(QKeyEvent *event)
{
  if(event->key() == Qt::Key_Space){
    printf("space\n");
  }

  if(event->key() == Qt::Key_Left){
    printf("left\n");
  }

  if(event->key() == Qt::Key_Right){
    printf("right\n");
  }
}

// void Viewer2::keyPressEvent(QKeyEvent* event)
// {
//   printf("keyPressEvent\n");
// }


// destructor
Viewer2::~Viewer2()
{
  for(int i=0; i<2; i++)
  {
    delete slam_render_panel[i];
    delete slam_manager[i];
  }

  delete left_group;
  delete right_group;
  delete slam_map;
  delete slam_imu;
  delete main_layout;
  delete left;
  delete right;
  delete imageLabel;
}
