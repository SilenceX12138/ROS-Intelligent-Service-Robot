#include "../include/fight_with_hair_ui/build_map_auto.h"
#include "ui_build_map_auto.h"
#include <QMessageBox>
#include <QKeyEvent>
#include <QString>
#include <stdio.h>
#include <string>
#include <QAbstractItemView>
#include <QCloseEvent>

build_map_auto::build_map_auto(bool isSim,QWidget *parent) :
  QWidget(parent),
  ui(new Ui::build_map_auto)
{
  ui->setupUi(this);

  if(isSim){
    QString autoMap="gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch rrt_slam rrt_slam_sim.launch'&";
    system(autoMap.toLatin1().data());
  }else{
    QString autoMap="gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch rrt_slam rrt_slam.launch'&";
    system(autoMap.toLatin1().data());
  }


  ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
  render_panel_ = new rviz::RenderPanel;
  ui->mapdisplay_layout->addWidget(render_panel_);
  manager_ = new rviz::VisualizationManager(render_panel_);
  tool_manager_ = manager_->getToolManager();
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();
}

build_map_auto::~build_map_auto()
{
  delete ui;
}

void build_map_auto::on_add_map_button_clicked()
{
  static int row=0;
  std::string str=std::to_string(row);
  const char *p = str.data();
  //    itoa(row,p,10);
  QString mapName=ui->input->text();
  ui->tableWidget->insertRow(ui->tableWidget->rowCount());
  ui->tableWidget->setItem(row, 0, new QTableWidgetItem(QString(p)));
  ui->tableWidget->setItem(row, 1, new QTableWidgetItem(mapName));
  row++;
  QString s1="gnome-terminal -x bash -c 'source ~/.bashrc;rosrun map_server map_saver -f ~/fight_with_hair/src/navi/maps/"+mapName+"'&";
  system(s1.toLatin1().data());
  QMessageBox::information(this, tr("提示"), tr("保存成功!"));
  hasSaved = true;
}

void build_map_auto::on_returnMain_clicked()
{
  QString string;
  if(!hasSaved){
    QMessageBox::StandardButton choice=QMessageBox::question(this,"Warning","地图尚未保存，确定退出建图界面？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
    if(choice==QMessageBox::Yes){
//      system(string.toLatin1().data());
      QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
      system(killString.toLatin1().data());
      killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
      system(killString.toLatin1().data());
      
      this->close();
    }
    return ;
  }
//  system(string.toLatin1().data());
  QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
  system(killString.toLatin1().data());
  killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
  system(killString.toLatin1().data());
  this->close();
}

void build_map_auto::on_logout_clicked()
{
  QMessageBox::StandardButton choice=QMessageBox::question(this,"确认","确定要退出吗？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  if(choice==QMessageBox::Yes){
    this->close();
//    QString string;
//    system(string.toLatin1().data());
//    string;
//    system(string.toLatin1().data());
    QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
    system(killString.toLatin1().data());
    killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
    system(killString.toLatin1().data());
    QApplication::quit();
  }
}

void build_map_auto::on_addNaviGoal_clicked()
{
  current_tool_ = tool_manager_->addTool("rviz/SetGoal");
  rviz::Property *pro = current_tool_->getPropertyContainer();
  pro->subProp("Topic")->setValue("move_base_simple/goal");
  manager_->setFixedFrame("map");
  tool_manager_->setCurrentTool(current_tool_);
}

void build_map_auto::on_addPublishPoint_clicked()
{
  current_tool_ = tool_manager_->addTool("rviz/PublishPoint");
  rviz::Property *pro = current_tool_->getPropertyContainer();
  pro->subProp("Topic")->setValue("/clicked_point");
  manager_->setFixedFrame("map");
  tool_manager_->setCurrentTool(current_tool_);
}

void build_map_auto::on_map_show_btn_clicked()
{

  manager_->removeAllDisplays();

  rviz::Display *map_=manager_->createDisplay("rviz/Map","fwh_ui map",true);
  ROS_ASSERT(map_!=NULL);
  map_->subProp("Topic")->setValue("/map");
  map_->subProp("Color Scheme")->setValue("map");

  rviz::Display *map2_=manager_->createDisplay("rviz/Map","fwh_ui map2",true);
  ROS_ASSERT(map2_!=NULL);
  map2_->subProp("Topic")->setValue("/move_base_node/global_costmap/costmap");
  map2_->subProp("Color Scheme")->setValue("costmap");



  rviz::Display *robot_=manager_->createDisplay("rviz/RobotModel","fwh_ui robot",true);
  ROS_ASSERT(robot_!=NULL);
  robot_->subProp("Robot Description")->setValue("robot_description");

  rviz::Display *laser_=manager_->createDisplay("rviz/LaserScan","fwh_ui laser",true);
  ROS_ASSERT(laser_!=NULL);
  laser_->subProp("Topic")->setValue("/scan");
  laser_->subProp("Size (m)")->setValue("0.1");


  rviz::Display *path1_=manager_->createDisplay("rviz/Path","fwh_ui_path_1",true);
  ROS_ASSERT(path1_!=NULL);
  path1_->subProp("Topic")->setValue("/move_base_node/NavfnROS/plan");
  path1_->subProp("Line Style")->setValue("Lines");
  path1_->subProp("Color")->setValue("25; 255; 0");
  path1_->subProp("Alpha")->setValue("1");
  path1_->subProp("Buffer Length")->setValue("1");

  rviz::Display *path2_=manager_->createDisplay("rviz/Path","fwh_ui_path_2",true);
  ROS_ASSERT(path2_!=NULL);
  path2_->subProp("Topic")->setValue("/move_base_node/TrajectoryPlannerROS/global_plan");
  path2_->subProp("Line Style")->setValue("Lines");
  path2_->subProp("Color")->setValue("85; 0; 127");
  path2_->subProp("Alpha")->setValue("1");
  path2_->subProp("Buffer Length")->setValue("1");


  rviz::Display *path3_=manager_->createDisplay("rviz/Path","fwh_ui_path_3",true);
  ROS_ASSERT(path3_!=NULL);
  path3_->subProp("Topic")->setValue("/move_base_node/TrajectoryPlannerROS/local_plan");
  path3_->subProp("Line Style")->setValue("Lines");
  path3_->subProp("Color")->setValue("255; 255; 127");
  path3_->subProp("Alpha")->setValue("1");
  path3_->subProp("Buffer Length")->setValue("1");


  rviz::Display *marker1_=manager_->createDisplay("rviz/Marker","fwh_ui_marker_1",true);
  ROS_ASSERT(marker1_!=NULL);
  marker1_->subProp("Marker Topic")->setValue("/global_detector_shapes");
  marker1_->subProp("Queue Size")->setValue("100");

  rviz::Display *marker2_=manager_->createDisplay("rviz/Marker","fwh_ui_marker_2",true);
  ROS_ASSERT(marker2_!=NULL);
  marker2_->subProp("Marker Topic")->setValue("/local_detector_shapes");
  marker2_->subProp("Queue Size")->setValue("100");

  rviz::Display *marker3_=manager_->createDisplay("rviz/Marker","fwh_ui_marker_3",true);
  ROS_ASSERT(marker3_!=NULL);
  marker3_->subProp("Marker Topic")->setValue("/centroids");
  marker3_->subProp("Queue Size")->setValue("100");

  manager_->startUpdate();
}


void build_map_auto::setMain(fight_with_hair_ui::MainWindow *m){
  main=m;
}


void build_map_auto::closeEvent(QCloseEvent *event){
  main->show();
  emit build_map_auto_finish();
}
