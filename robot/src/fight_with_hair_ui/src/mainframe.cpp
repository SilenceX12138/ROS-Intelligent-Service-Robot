#include "../include/fight_with_hair_ui/mainframe.h"
#include "ui_mainframe.h"
#include <QMessageBox>

mainframe::mainframe(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::mainframe)
{
  ui->setupUi(this);
  this->setFixedSize(this->geometry().width(),this->geometry().height());

}

mainframe::~mainframe()
{
  delete ui;
}

void mainframe::on_control_with_key_clicked()
{
  userControl *us=new userControl;
  us->move(550,200);
  us->show();
  this->close();
}

void mainframe::on_navigation_clicked()
{
    navigation *na=new navigation(true);
    na->move(550,200);
    na->show();
    this->close();
}

void mainframe::on_build_map_clicked()
{
    build_map *bm=new build_map(true);
    bm->move(550,200);
    bm->show();
    this->close();
}

void mainframe::on_detection_clicked()
{
    detection *de=new detection(1);
    de->move(550,200);
    de->show();
    this->close();
}

void mainframe::on_logout_clicked()
{
    QMessageBox::StandardButton choice=QMessageBox::question(this,"确认","确定要退出吗？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
    if(choice==QMessageBox::Yes){
        this->close();
        QApplication::quit();
    }
}
