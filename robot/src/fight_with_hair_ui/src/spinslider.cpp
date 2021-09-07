#include "../include/fight_with_hair_ui/spinslider.h"
#include "ui_spinslider.h"

spinslider::spinslider(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::spinslider)
{
  ui->setupUi(this);
  DashBoard_x = new CCtrlDashBoard(ui->speed_x_widget);
  DashBoard_x->setGeometry(ui->speed_x_widget->rect());
  DashBoard_x->setValue(0);
  DashBoard_x->setUnit(0);
}

spinslider::~spinslider()
{
  delete ui;
}
