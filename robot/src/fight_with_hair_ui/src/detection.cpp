#include "../include/fight_with_hair_ui/detection.h"
#include "ui_detection.h"
#include "../include/fight_with_hair_ui/mainframe.h"
#include <QMessageBox>
#include <QString>
#include <QCloseEvent>
#include <QListWidgetItem>
#include <QRadioButton>
#include <string>

detection::detection(bool isSim,QWidget *parent) :
  QWidget(parent),
  ui(new Ui::detection)
{
  ui->setupUi(this);
 //this->setFixedSize(this->geometry().width(),this->geometry().height());
  QString objStr;
  if (isSim){
    objStr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch fwh_obj fwh_obj_t.launch'&";

  }else{
    objStr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch fwh_obj fwh_obj_l.launch'&";
  }
  system(objStr.toLatin1().data());
  ifIsSim = isSim;
  render_panel_ = new rviz::RenderPanel;
  ui->mapdisplay_layout->addWidget(render_panel_);
  manager_ = new rviz::VisualizationManager(render_panel_);

  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  //manager_->load(config);
  manager_->startUpdate();
  manager_->setFixedFrame("/base_footprint");
  //  setObjectSize(10);
}

detection::~detection()
{
  delete ui;
}

void detection::ros_init(int argc,char **argv)
{
  ros::init(argc,argv,"navigation",ros::init_options::AnonymousName);
}

void detection::on_logout_clicked()
{
  QMessageBox::StandardButton choice=QMessageBox::question(this,"确认","确定要退出吗？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  if(choice==QMessageBox::Yes){
    this->close();
    QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /fwh_obj_detect;rosnode kill /fwh_obj_grab'&";
//    system(string.toLatin1().data());
    string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /joint_state_publisher;rosnode kill /robot_state_publisher;'&";
//    system(string.toLatin1().data());
    QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
    system(killString.toLatin1().data());
    killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
    system(killString.toLatin1().data());
    QApplication::quit();
  }
}

void detection::on_returnMain_clicked()
{
  //  mainframe *main=new mainframe;
  //  main->move(550,200);
  //  main->show();

  QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /fwh_obj_detect;rosnode kill /fwh_obj_grab'&";
//  system(string.toLatin1().data());
  QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
  system(killString.toLatin1().data());
  killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
  system(killString.toLatin1().data());
  this->close();
}


void detection::on_detect_start_clicked()
{

  manager_->removeAllDisplays();

  rviz::Display *grid_ = manager_->createDisplay( "rviz/Grid", "fwh_grid", true);
  ROS_ASSERT(grid_!=NULL);
  grid_->subProp( "Line Style" )->setValue("Billboards");

  //机器人模型
  rviz::Display *robot_ = manager_->createDisplay("rviz/RobotModel", "fwh_robot", true);
  ROS_ASSERT(robot_!=NULL);
  robot_->subProp("Robot Description")->setValue("robot_description");
  //相机
  rviz::Display *camera_ = manager_->createDisplay("rviz/PointCloud2", "fwh_camera", true);
  ROS_ASSERT(camera_!=NULL);
  if (ifIsSim){
    camera_->subProp("Topic")->setValue("/kinect2/sd/points");
  }else{
    camera_->subProp("Topic")->setValue("/kinect2/qhd/points");
  }
  camera_->subProp("Size (m)")->setValue("0.01");
  camera_->subProp("Alpha")->setValue("1");
  rviz::Display *marker_ = manager_->createDisplay("rviz/Marker", "fwh_marker", true);
  ROS_ASSERT(marker_!=NULL);
  marker_->subProp("Marker Topic")->setValue("/obj_marker");
  
  marker_->subProp("Queue Size")->setValue("100");

//  rviz::Display *kine_=manager_->createDisplay("rviz/Kinect", "fwh_kine", true);
//  ROS_ASSERT(kine_!=NULL);
//  kine_->subProp("Topic")->setValue("/kinect2/qhd/points");
//  kine_->subProp("Size (m)")->setValue("0.01");
//  kine_->subProp("Alpha")->setValue("1");

  manager_->startUpdate();

  emit sendDetectMsg(true);

}

void detection::setObjectSize(int num){
  choseObjNum=-1;
  objectSize=num;
  ui->object_list->clear();
  for (int i=0;i<num;i++)
  {
    QListWidgetItem *objItem = new QListWidgetItem();
    std::string str=std::to_string(i);
    std::string description = "Object" + str;
    const char *p = description.data();
    QRadioButton *box = new QRadioButton();
    box->setText(QString(p));
    ui->object_list->addItem(objItem);
    ui->object_list->setItemWidget(objItem, box);
    connect(box,&QRadioButton::clicked,this,&detection::getAndSendObjectNum);
  }
}

void detection::closeEvent(QCloseEvent *event){
  main->show();
  emit detection_finish();
}

void detection::setMain(fight_with_hair_ui::MainWindow *m){
  main=m;
}

void detection::getAndSendObjectNum(){
  QRadioButton *obj=(QRadioButton *)(sender());
  std::string str=obj->text().toStdString();
  str=str.substr(6,str.length());
  const char *p = str.data();
  choseObjNum=std::atoi(p);
}

void detection::on_sendObjNum_clicked()
{
  if (choseObjNum==-1){
    QMessageBox::question(this,"Warning","未选择物品",QMessageBox::Yes,QMessageBox::Yes);
    return;
  }
  emit sendObjectNum(choseObjNum);
  choseObjNum=-1;
}
