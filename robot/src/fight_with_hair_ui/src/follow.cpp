#include "../include/fight_with_hair_ui/follow.h"
#include "ui_follow.h"
#include <QMessageBox>
#include <QPixmap>
#include <QCloseEvent>

Follow::Follow(bool isSim,QWidget *parent) :
  QWidget(parent),
  ui(new Ui::Follow)
{
  ui->setupUi(this);
  this->setFixedSize(this->geometry().width(),this->geometry().height());
  QString followStr;
  if (isSim){
    followStr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch fwh_follow fwh_follow_t.launch'&";
  }else{
    followStr = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch fwh_follow fwh_follow_l.launch'&";
  }
  system(followStr.toLatin1().data());

  QPixmap *ro=new QPixmap(":/images/follow_robot.jpg");
  ro->scaled(ui->pic->size(),Qt::KeepAspectRatio);
  ui->pic->setScaledContents(true);
  ui->pic->setPixmap(*ro);
}

Follow::~Follow()
{
  delete ui;
}

void Follow::on_returnMain_clicked()
{
  QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /fwh_follow_yz'&";
//  system(string.toLatin1().data());
  QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
  system(killString.toLatin1().data());
  killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
  system(killString.toLatin1().data());
  this->close();
}

void Follow::on_logout_clicked()
{
  QMessageBox::StandardButton choice=QMessageBox::question(this,"确认","确定要退出吗？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  if(choice==QMessageBox::Yes){
    this->close();
    QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /fwh_follow_yz'&";

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

void Follow::on_follow_begin_clicked()
{
  emit send_follow_begin_msg();
}

void Follow::on_follow_end_clicked()
{
  emit send_follow_end_msg();
}

void Follow::closeEvent(QCloseEvent *event){
  main->show();
  emit follow_finish();
}

void Follow::setMain(fight_with_hair_ui::MainWindow *m){
  main=m;
}
