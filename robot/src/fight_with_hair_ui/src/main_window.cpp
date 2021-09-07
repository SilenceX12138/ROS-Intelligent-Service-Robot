/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/fight_with_hair_ui/main_window.hpp"
#include "../include/fight_with_hair_ui/mainframe.h"
#include "../include/fight_with_hair_ui/usercontrol.h"
#include "../include/fight_with_hair_ui/build_map.h"
#include "../include/fight_with_hair_ui/detection.h"
#include "../include/fight_with_hair_ui/spinslider.h"
#include "../include/fight_with_hair_ui/navigation.h"
#include "../include/fight_with_hair_ui/follow.h"
#include "../include/fight_with_hair_ui/build_map_auto.h"
#include <QPixmap>
#include <QLabel>
#include <QCloseEvent>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace fight_with_hair_ui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  this->setFixedSize(this->geometry().width(),this->geometry().height());
  ReadSettings();
  setWindowIcon(QIcon(":/images/icon.png"));
  QPixmap *ro=new QPixmap(":/images/robot.png");
  ro->scaled(ui.pic->size(),Qt::KeepAspectRatio);
  ui.pic->setScaledContents(true);
  ui.pic->setPixmap(*ro);
  qnode.init();

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/
void MainWindow::ReadSettings() {

}

void MainWindow::WriteSettings() {

}

void MainWindow::closeEvent(QCloseEvent *event)
{
  WriteSettings();
  QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /joint_state_publisher;rosnode kill /robot_state_publisher;'&";
  system(string.toLatin1().data());
  QMainWindow::closeEvent(event);
}

}  // namespace fight_with_hair_ui


void fight_with_hair_ui::MainWindow::on_control_with_key_clicked()
{
  //  qnode.sendMoveMsg(1,0,0);
  userControl *us=new userControl;
  us->move(550,200);
  us->setMain(this);
  this->hide();
  us->show();
  //  connect(us,SIGNAL(movemsg(float,float,float)),&qnode,SLOT(sendMoveMsg(float,float,float)));
  connect(us,&userControl::movemsg,&qnode,&QNode::sendMoveMsg);
  connect(us,&userControl::armmsg,&qnode,&QNode::sendArmMsg);
  connect(us,&userControl::userControl_finish,&qnode,&QNode::sendVoiceSigMsg);
}

void fight_with_hair_ui::MainWindow::on_navigation_clicked()
{
  navigation *na=new navigation(isSim);
  na->move(550,200);
  na->setMain(this);
  this->hide();
  na->show();
  connect(na,&navigation::navigation_finish,&qnode,&QNode::sendVoiceSigMsg);
}

void fight_with_hair_ui::MainWindow::on_build_map_clicked()
{
  build_map *bm=new build_map(isSim);
  bm->move(550,200);
  bm->setMain(this);
  this->hide();
  bm->show();
  connect(bm,&build_map::movemsg,&qnode,&QNode::sendMoveMsg);
  connect(bm,&build_map::build_map_finish,&qnode,&QNode::sendVoiceSigMsg);
}


void fight_with_hair_ui::MainWindow::on_detection_clicked()
{
  detection *de=new detection(isSim);
  de->move(550,200);
  de->setMain(this);
  this->hide();
  de->show();
  connect(de,&detection::sendObjectNum,&qnode,&QNode::sendObjDetectMsg);
  connect(&qnode,&QNode::objFind,de,&detection::setObjectSize);
  connect(de,&detection::sendDetectMsg,&qnode,&QNode::sendObjSigMsg);
  connect(de,&detection::detection_finish,&qnode,&QNode::sendVoiceSigMsg);
}

void fight_with_hair_ui::MainWindow::on_logout_clicked()
{
  QMessageBox::StandardButton choice=QMessageBox::question(this,"确认","确定要退出吗？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  if(choice==QMessageBox::Yes){
    this->close();
    QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /joint_state_publisher;rosnode kill /robot_state_publisher;'&";
    system(string.toLatin1().data());
    QApplication::quit();
  }
}

void fight_with_hair_ui::MainWindow::on_follow_clicked()
{
  Follow *fo=new Follow(isSim);
  fo->move(550,200);
  fo->setMain(this);
  this->hide();
  fo->show();
  connect(fo,&Follow::send_follow_begin_msg,&qnode,&QNode::sendFollowBeginMsg);
  connect(fo,&Follow::send_follow_end_msg,&qnode,&QNode::sendFollowEndMsg);
  connect(fo,&Follow::follow_finish,&qnode,&QNode::sendVoiceSigMsg);
}

void fight_with_hair_ui::MainWindow::on_ASR_clicked()
{
  if (isSim){
    QString ASRstrTest = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch fwh_sound fwh_sound_t.launch'&";
    system(ASRstrTest.toLatin1().data());
  }else{
    QString ASRstr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch fwh_sound fwh_sound_l.launch'&";
    system(ASRstr.toLatin1().data());
  }

  connect(&qnode,&QNode::asr_order,this,&MainWindow::receive_asr_orders);
  connect(this,&MainWindow::send_follow_end_msg,&qnode,&QNode::sendFollowEndMsg);
}

void fight_with_hair_ui::MainWindow::receive_asr_orders(std::string str){
  if(str=="obj"){
    on_detection_clicked();
    return;
  }
  if(str=="navi"){
    on_navigation_clicked();
    return;
  }
  if(str=="mani"){
    on_control_with_key_clicked();
    return;
  }
  if (str=="follow"){
    on_follow_clicked();
    return;
  }
  if (str=="build"){
    on_build_map_clicked();
    return;
  }
  if (str=="follow_end"){
    emit send_follow_end_msg();
    return;
  }
}

void fight_with_hair_ui::MainWindow::chooseEnvironment(){
  QMessageBox::StandardButton choice=QMessageBox::question(this,"Question","是否在机器人上运行？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  if(choice==QMessageBox::Yes){
    isSim=false;
  }else{
    isSim=true;
  }
}

void fight_with_hair_ui::MainWindow::on_aaaaa_clicked()
{
  QString yanshi = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch wpb_home_apps innovation.launch'&";
  system(yanshi.toLatin1().data());
}

void fight_with_hair_ui::MainWindow::on_autoMap_clicked()
{
  build_map_auto *bma=new build_map_auto(isSim);
  bma->move(550,200);
  bma->setMain(this);
  this->hide();
  bma->show();
}
