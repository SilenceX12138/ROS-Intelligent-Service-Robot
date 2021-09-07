#include "../include/fight_with_hair_ui/loginwindow.h"
#include "ui_loginwindow.h"
#include <QString>
#include <QMessageBox>
#include <QCloseEvent>
#include <QLineEdit>

LoginWindow::LoginWindow(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::LoginWindow)
{
  ui->setupUi(this);
  this->setFixedSize(this->geometry().width(),this->geometry().height());
  ui->input_password->setEchoMode(QLineEdit::Password);
}

LoginWindow::~LoginWindow()
{
  delete ui;
}

void LoginWindow::on_loginButton_clicked()
{
  QString user=ui->input_name->text();
  QString password=ui->input_password->text();
  if(user.isEmpty()||password.isEmpty()){
    QMessageBox::information(NULL, tr("提示"),tr("用户名或密码不能为空！"));
    return;
  }
  if(user=="fwh"&&password=="1"){
    // QMessageBox::information(NULL, "提示", "登陆成功",QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    QMessageBox::information(this, tr("提示"),tr("登陆成功！"));
    main->show();
    this->close();
    main->chooseEnvironment();
  }else{
    QMessageBox::information(this, tr("警告"),tr("用户名或密码错误！"));
    return;
  }
}

void LoginWindow::on_input_password_returnPressed()
{
  on_loginButton_clicked();
}

void LoginWindow::on_resetButton_clicked()
{
  ui->input_name->setText("");
  ui->input_password->setText("");
}

void LoginWindow::setMain(fight_with_hair_ui::MainWindow *m){
  main=m;
}

void LoginWindow::closeEvent(QCloseEvent *event){
}
