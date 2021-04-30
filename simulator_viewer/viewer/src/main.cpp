#include <QApplication>
#include <ros/ros.h>
#include "viewer1.h"
#include "viewer2.h"

int main(int argc, char **argv)
{
  if( !ros::isInitialized())
  {
   ros::init(argc, argv, "viewer", ros::init_options::AnonymousName);
  }
  
  QApplication app(argc, argv);

  QWidget* window = new QWidget();
  
  Viewer1* viewer1 = new Viewer1();
  //Viewer2* viewer2 = new Viewer2();


/*
  QToolBar* toolBar = new QToolBar();
  toolBar->addAction("Open");
  toolBar->setStyleSheet("QToolBar {border: 1px}");
*/  

  //QTabWidget* tabs = new QTabWidget();
  //tabs->addTab(viewer1, "Tab1");
  //tabs->addTab(viewer2, "Tab2");

  //QVBoxLayout* qvbox = new QVBoxLayout();
  //qvbox->setMenuBar(toolBar);
  //qvbox->addWidget(tabs);
  //qvbox->addWidget(viewer1);
  //tabs->setWindowFlags(Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint | Qt::WindowCloseButtonHint);
  //tabs->showMaximized();
  
  app.exec();

  delete viewer1;

}
