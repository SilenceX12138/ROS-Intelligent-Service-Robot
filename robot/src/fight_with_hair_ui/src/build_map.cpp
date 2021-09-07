#include "../include/fight_with_hair_ui/build_map.h"
#include "ui_build_map.h"
#include "../include/fight_with_hair_ui/mainframe.h"
#include <QMessageBox>
#include <QKeyEvent>
#include <QString>
#include <stdio.h>
#include <string>
#include <QAbstractItemView>
#include <QCloseEvent>
build_map::build_map(bool isSim,QWidget *parent) :
  QWidget(parent),
  ui(new Ui::build_map)
{
  ui->setupUi(this);
//  this->setFixedSize(this->geometry().width(),this->geometry().height());

//  QString mapStr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch slam gmapping.launch'&";
//  system(mapStr.toLatin1().data());


  if (isSim){
    QString mapStr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch slam gmapping_simulate.launch'&";
    system(mapStr.toLatin1().data());
  }else{
    QString mapStr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch slam gmapping.launch'&";
    system(mapStr.toLatin1().data());
  }

  QString moveStr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch user_listen keyboard_user_listen_for_map.launch'&";
  system(moveStr.toLatin1().data());

  ui->tableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
  render_panel_ = new rviz::RenderPanel;
  ui->mapdisplay_layout->addWidget(render_panel_);
  manager_ = new rviz::VisualizationManager(render_panel_);
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();
}

build_map::~build_map()
{
  delete ui;
}


void build_map::ros_init(int argc,char **argv)
{
  ros::init(argc,argv,"build_map",ros::init_options::AnonymousName);
}

void build_map::on_logout_clicked()
{
  QMessageBox::StandardButton choice=QMessageBox::question(this,"确认","确定要退出吗？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  if(choice==QMessageBox::Yes){
    this->close();
    QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /arm_move;"
                     "rosnode kill /basic_avoidance;"
                     "rosnode kill /basic_move;"
                     "rosnode kill /my_joy;"
                     "rosnode kill /keyboard_listen;"
                     "rosnode kill /user_listen;"
                     "rosnode kill /slam_gmapping;"
                     "rosnode kill /slam_lidar_filter;"
                     "rosnode kill /wpb_home_core;"
                     "rosnode kill /map_saver;'&";
   // system(string.toLatin1().data());
    string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /joint_state_publisher;rosnode kill /robot_state_publisher;'&";
    //system(string.toLatin1().data());
    QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
    system(killString.toLatin1().data());
    killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
    system(killString.toLatin1().data());
    QApplication::quit();
  }
}

void build_map::on_returnMain_clicked()
{
  //  mainframe *main=new mainframe;
  //  main->move(550,200);
  //  main->show();
  QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /arm_move;"
                   "rosnode kill /basic_avoidance;"
                   "rosnode kill /basic_move;"
                   "rosnode kill /my_joy;"
                   "rosnode kill /keyboard_listen;"
                   "rosnode kill /user_listen;"
                   "rosnode kill /slam_gmapping;"
                   "rosnode kill /slam_lidar_filter;"
                   "rosnode kill /wpb_home_core;"
                   "rosnode kill /map_saver;'&";
  if(!hasSaved){
    QMessageBox::StandardButton choice=QMessageBox::question(this,"Warning","地图尚未保存，确定退出建图界面？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
    if(choice==QMessageBox::Yes){
      //system(string.toLatin1().data());
      QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
      system(killString.toLatin1().data());
      killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
      system(killString.toLatin1().data());
      this->close();
    }
    return ;
  }
  //system(string.toLatin1().data());
  QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
  system(killString.toLatin1().data());
  killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
  system(killString.toLatin1().data());
  this->close();
}

void build_map::keyPressEvent(QKeyEvent *event){
  if(event->key() == Qt::Key_Escape){
    on_logout_clicked();
  }else if(event->key()==Qt::Key_W){
    ui->forward->setStyleSheet("color:red;");
    emit movemsg(0.3,0,0);
  }else if(event->key()==Qt::Key_A){
    ui->left->setStyleSheet("color:red;");
    emit movemsg(0,0,0.6);
  }else if(event->key()==Qt::Key_X){
    ui->backward->setStyleSheet("color:red;");
    emit movemsg(-0.3,0,0);
  }else if(event->key()==Qt::Key_D){
    ui->right->setStyleSheet("color:red;");
    emit movemsg(0,0,-0.6);
  }else if(event->key()==Qt::Key_S){
    ui->stop->setStyleSheet("color:red;");
    emit movemsg(0,0,0);
  }else if(event->key()==Qt::Key_Q){
    ui->left_forward->setStyleSheet("color:red;");
    emit movemsg(0.3,0,0.3);
  }else if(event->key()==Qt::Key_E){
    ui->right_forward->setStyleSheet("color:red;");
    emit movemsg(0.3,0,-0.3);
  }else if(event->key()==Qt::Key_Z){
    ui->left_back->setStyleSheet("color:red;");
    emit movemsg(-0.3,0,0.3);
  }else if(event->key()==Qt::Key_C){
    ui->right_back->setStyleSheet("color:red;");
    emit movemsg(-0.3,0,-0.3);
  }
}
void build_map::keyReleaseEvent(QKeyEvent *event) {
  if(event->key()==Qt::Key_W){
    ui->forward->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_A){
    ui->left->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_X){
    ui->backward->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_D){
    ui->right->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_S){
    ui->stop->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_Q){
    ui->left_forward->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_E){
    ui->right_forward->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_Z){
    ui->left_back->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_C){
    ui->right_back->setStyleSheet("color:black;");
  }
}

void build_map::on_add_map_button_clicked()
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

void build_map::on_map_show_btn_clicked()
{

  manager_->removeAllDisplays();

  rviz::Display *map_=manager_->createDisplay("rviz/Map","fwh_ui map",true);
  ROS_ASSERT(map_!=NULL);
  map_->subProp("Topic")->setValue("/map");

  rviz::Display *robot_=manager_->createDisplay("rviz/RobotModel","fwh_ui robot",true);
  ROS_ASSERT(robot_!=NULL);
  robot_->subProp("Robot Description")->setValue("robot_description");

  rviz::Display *laser_=manager_->createDisplay("rviz/LaserScan","fwh_ui laser",true);
  ROS_ASSERT(laser_!=NULL);
  laser_->subProp("Topic")->setValue("/scan");
  laser_->subProp("Size (m)")->setValue("0.1");
  manager_->startUpdate();
}

void build_map::on_forward_clicked()
{
  emit movemsg(0.3,0,0);
}

void build_map::on_right_forward_clicked()
{
  emit movemsg(0.3,0,-0.3);
}

void build_map::on_right_clicked()
{
  emit movemsg(0,0,-0.6);
}

void build_map::on_right_back_clicked()
{
  emit movemsg(0.3,0,-0.3);
}

void build_map::on_backward_clicked()
{
  emit movemsg(-0.3,0,0);
}

void build_map::on_left_back_clicked()
{
  emit movemsg(-0.3,0,0.3);
}

void build_map::on_left_clicked()
{
  emit movemsg(0,0,0.6);
}

void build_map::on_left_forward_clicked()
{
  emit movemsg(0.3,0,0.3);
}

void build_map::on_stop_clicked()
{
  emit movemsg(0,0,0);
}

void build_map::closeEvent(QCloseEvent *event){
  main->show();
  emit build_map_finish();
}

void build_map::setMain(fight_with_hair_ui::MainWindow *m){
  main=m;
}

void build_map::setEnvironment(bool s)
{
  isSim = s;
}
