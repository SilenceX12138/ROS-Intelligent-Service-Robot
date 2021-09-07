#include "../include/fight_with_hair_ui/navigation.h"
#include "ui_navigation.h"
#include "../include/fight_with_hair_ui/mainframe.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QListWidgetItem>
#include <QCheckBox>
#include <QCloseEvent>
#include "tinyxml.h"

navigation::navigation(bool isS,QWidget *parent) :
  QWidget(parent),
  ui(new Ui::navigation)
{
  ui->setupUi(this);
//  this->setFixedSize(this->geometry().width(),this->geometry().height());
  ui->waypoints_list->horizontalHeader()->setStretchLastSection(true);
  ui->waypoints_list->setEditTriggers(QAbstractItemView::NoEditTriggers);
  mapfile = "";
  mapname = "";
  xmlfile = "";

  isSim = isS;

  render_panel_ = new rviz::RenderPanel();
  ui->mapdisplay_layout->addWidget(render_panel_);
  manager_ = new rviz::VisualizationManager(render_panel_);
  tool_manager_ = manager_->getToolManager();
  render_panel_->initialize(manager_->getSceneManager(), manager_);
  manager_->initialize();
  manager_->startUpdate();
}

navigation::~navigation()
{
  delete ui;
}


void navigation::ros_init(int argc,char **argv)
{
  ros::init(argc,argv,"navigation",ros::init_options::AnonymousName);
}


void navigation::on_logout_clicked()
{
  QMessageBox::StandardButton choice=QMessageBox::question(this,"确认","确定要退出吗？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  if(choice==QMessageBox::Yes){
    this->close();
//    QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /amcl;"
//                     "rosnode kill /map_server;"
//                     "rosnode kill /move_base;"
//                     "rosnode kill /slam_lidar_filter;"
//                     "rosnode kill /wp_manager;"
//                     "rosnode kill /wp_nav_test;"
//                     "rosnode kill /robot_state_publisher;'&";

//    system(string.toLatin1().data());
//    string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /joint_state_publisher;rosnode kill /robot_state_publisher;'&";
//    system(string.toLatin1().data());
    QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
    system(killString.toLatin1().data());
    killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
    system(killString.toLatin1().data());
    QApplication::quit();
  }
}

void navigation::on_returnMain_clicked()
{
  //  mainframe *main=new mainframe;
  //  main->move(550,200);
  //  main->show();
  QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /amcl;"
                   "rosnode kill /map_server;"
                   "rosnode kill /move_base;"
                   "rosnode kill /slam_lidar_filter;"
                   "rosnode kill /wp_manager;"
                   "rosnode kill /wp_nav_test;'&";

//  system(string.toLatin1().data());
  QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
  system(killString.toLatin1().data());
  killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
  system(killString.toLatin1().data());
  this->close();
}

void navigation::on_display_button_clicked()
{
  if (mapfile == ""){
    QMessageBox::StandardButton choice=QMessageBox::question(this,"Warning","未选择地图，是否加载默认地图？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
    if(choice==QMessageBox::No){
      return;
    }
  }
  else if (mapfile.right(mapfile.length() - mapfile.lastIndexOf(".") - 1) != "yaml"){
    QMessageBox::information(this, tr("ERROR"), tr("地图文件后缀名应为yaml"));
    return;
  }
  mapname = mapfile.mid(mapfile.lastIndexOf("/") + 1, mapfile.lastIndexOf(".") - mapfile.lastIndexOf("/") - 1);

  manager_->setFixedFrame("/map");
  manager_->removeAllDisplays();

  rviz::Display *map_ = manager_->createDisplay("rviz/Map", "fwh_ui map", true);
  ROS_ASSERT(map_!=NULL);
  map_->subProp("Topic")->setValue("/map");

  rviz::Display *robot_ = manager_->createDisplay("rviz/RobotModel", "fwh_ui robot", true);
  ROS_ASSERT(robot_!=NULL);
  robot_->subProp("Robot Description")->setValue("robot_description");

  rviz::Display *laser_=manager_->createDisplay("rviz/LaserScan","fwh_ui laser",true);
  ROS_ASSERT(laser_!=NULL);
  laser_->subProp("Topic")->setValue("/scan");
  laser_->subProp("Size (m)")->setValue("0.1");

  rviz::Display *path1_=manager_->createDisplay("rviz/Path","fwh_ui_path_1",true);
  ROS_ASSERT(path1_!=NULL);
  path1_->subProp("Topic")->setValue("/move_base/GlobalPlanner/plan");
  path1_->subProp("Line Style")->setValue("Billboards");
  path1_->subProp("Color")->setValue("255; 85; 255");
  path1_->subProp("Alpha")->setValue("1");
  path1_->subProp("Buffer Length")->setValue("1");


  rviz::Display *mark_ = manager_->createDisplay("rviz/Marker", "fwh_ui mark", true);
  if (mark_!=NULL)
  {
    ROS_ASSERT(mark_!=NULL);
    mark_->subProp("Marker Topic")->setValue("/waypoints_marker");
  }

  ui->waypoints_list->clearContents();
  int row = 0;
  int i=0;
  TiXmlDocument doc;
  QString xmlQStr="~/fight_with_hair/navi/waypoints/"+mapname;
  const char* xmlf=xmlQStr.toLatin1().data();
  if (doc.LoadFile(xmlf))
  {
    //    ui->map_name->setText("QString(p)");
    TiXmlElement *root = doc.FirstChildElement();
    for (TiXmlElement *elem = root->FirstChildElement(); elem != NULL; elem=elem->NextSiblingElement())
    {
      i++;
      // 获取元素名
      std::string elemName = elem->Value();
      const char *p = elemName.data();
      if(i==1){ui->map_name->setText(QString(p));}
      if (elemName == "Name"){
        TiXmlNode *e1 = elem->FirstChild();
        ui->waypoints_list->insertRow(ui->waypoints_list->rowCount());
        ui->waypoints_list->setItem(row, 0, new QTableWidgetItem(QString(e1->ToText()->Value())));
        row++;
      }
    }
  }
  QString mapShow;
  if (isSim){
    mapShow = "gnome-terminal -x bash -c 'source ~/.bashrc; roslaunch fight_with_hair_ui navigation_sim.launch map_file:=" + mapfile + " waypoint_name:="+ mapname + "'&";
  }else{
    mapShow = "gnome-terminal -x bash -c 'source ~/.bashrc; roslaunch fight_with_hair_ui navigation_gen.launch map_file:=" + mapfile + " waypoint_name:="+ mapname + "'&";
  }
  system(mapShow.toLatin1().data());

}

void navigation::on_chooseMap_clicked()
{
  mapfile = QFileDialog::getOpenFileName(this, tr("选择地图"), "..", "", 0);
  QString mapname = "地图:" + mapfile.right(mapfile.length() - mapfile.lastIndexOf("/") - 1);
  if (mapfile == "")
  {
    QMessageBox::information(this, tr("WARNING"), tr("未选择地图"));
    ui->map_name->setText("请选择正确地图");
  }
  else if (mapfile.right(mapfile.length() - mapfile.lastIndexOf(".") - 1) != "yaml")
  {
    QMessageBox::information(this, tr("WARNING"), tr("地图格式应为yaml"));
    ui->map_name->setText("请选择正确地图");
  }else{
    ui->map_name->setText(mapname);
  }
}

void navigation::getButtonText(bool check)
{

}


void navigation::on_chooseDes_clicked()
{
  current_tool_ = tool_manager_->addTool("rviz/SetGoal");
  rviz::Property *pro = current_tool_->getPropertyContainer();
  pro->subProp("Topic")->setValue("move_base_simple/goal");
  manager_->setFixedFrame("map");
  tool_manager_->setCurrentTool(current_tool_);
}

void navigation::on_chooseWayPoint_clicked()
{
  current_tool_ = tool_manager_->addTool("rviz/AddWaypoint");
  rviz::Property *pro = current_tool_->getPropertyContainer();
  pro->subProp("Topic")->setValue("/waterplus/add_waypoint");
  manager_->setFixedFrame("map");
  tool_manager_->setCurrentTool(current_tool_);

  manager_->startUpdate();
}

void navigation::closeEvent(QCloseEvent *event)
{
  main->show();
  emit navigation_finish();
}

void navigation::on_cruise_button_clicked()
{
  QString cruise = "gnome-terminal -x bash -c 'source ~/.bashrc;rosrun waterplus_map_tools wp_nav_test'&";
  system(cruise.toLatin1().data());
}

void navigation::on_save_waypoints_clicked()
{
  QString saveWaypoint = "gnome-terminal -x bash -c 'source ~/.bashrc;rosrun navi way_saver ~/fight_with_hair/src/navi/waypoints/"+mapname+".xml'&";
  system(saveWaypoint.toLatin1().data());
  QMessageBox::information(this, tr("提示"), tr("保存成功!"));
}

void navigation::setMain(fight_with_hair_ui::MainWindow *m){
  main=m;
}
