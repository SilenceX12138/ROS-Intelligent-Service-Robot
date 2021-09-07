#include "../include/fight_with_hair_ui/usercontrol.h"
#include "ui_usercontrol.h"
#include "../include/fight_with_hair_ui/mainframe.h"
#include <QMessageBox>
#include <QKeyEvent>
#include <QString>
#include <QCloseEvent>

userControl::userControl(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::userControl)
{
  ui->setupUi(this);
  this->setFixedSize(this->geometry().width(),this->geometry().height());
  QString str = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch user_listen keyboard_user_listen.launch'&";
  system(str.toLatin1().data());
//  render_panel_ = new rviz::RenderPanel;
//  ui->mapdisplay_layout->addWidget(render_panel_);
//  manager_ = new rviz::VisualizationManager(render_panel_);

//  render_panel_->initialize(manager_->getSceneManager(), manager_);
//  manager_->initialize();
//  //manager_->load(config);
//  manager_->startUpdate();
//  manager_->setFixedFrame("/base_footprint");


  x_speed = new CCtrlDashBoard(ui->speed_x_widget);
  x_speed->setGeometry(ui->speed_x_widget->rect());
  x_speed->setValue(0);
  x_speed->setUnit(0);
  y_speed = new CCtrlDashBoard(ui->speed_y_widget);
  y_speed->setGeometry(ui->speed_y_widget->rect());
  y_speed->setValue(0);
  y_speed->setUnit(0);
  z_speed = new CCtrlDashBoard(ui->speed_z_widget);
  z_speed->setGeometry(ui->speed_z_widget->rect());
  z_speed->setValue(0);
  z_speed->setUnit(0);
}

userControl::~userControl()
{
  delete ui;
}

void userControl::on_logout_clicked()
{
  QMessageBox::StandardButton choice=QMessageBox::question(this,"确认","确定要退出吗？",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  if(choice==QMessageBox::Yes){
    //    QString str = "gnome-terminal -x bash -c 'source ~/.bashrc; rosnode kill all'&";
    //    system(str.toLatin1().data());
    this->close();
    QString string = "gnome-terminal -x bash -c 'source ~/.bashrc;rosnode kill /arm_move;"
                     "rosnode kill /basic_avoidance;"
                     "rosnode kill /basic_move;"
                     "rosnode kill /my_joy;"
                     "rosnode kill /keyboard_listen;"
                     "rosnode kill /user_listen;"
                     "'&";
    //rosnode kill /robot_state_publisher;
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

void userControl::on_returnMain_clicked()
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
                   "rosnode kill /slam_lidar_filter;"
                   "rosnode kill /wpb_home_core;"
                   "'&";
  //rosnode kill /robot_state_publisher;
//  system(string.toLatin1().data());
  QString killString = "rosnode list | awk '{print $1}' | grep -v 'fight_with_hair_ui' | xargs rosnode kill";
  system(killString.toLatin1().data());
  killString = "ps -a | grep roslaunch | awk '{if (NR >= 2) {print $1}}' | xargs kill -9&";
  system(killString.toLatin1().data());
  this->close();
}

void userControl::keyPressEvent(QKeyEvent *event){
  //键盘按下事件
  if(event->key() == Qt::Key_Escape){
    on_logout_clicked();
  }else if(event->key()==Qt::Key_W){
    ui->forward->setStyleSheet("color:red;");
    if(clickForward){
      if (x_send < 1){
        x_send+=0.01;
      }
      emit movemsg(x_send,0,0);
      showSpeed(x_send,0,0);
      clickForward = true;
      clickBack = false;
    }else{
      sendDataInit();
      emit movemsg(x_send,0,0);
      showSpeed(x_send,0,0);
      clickForward = true;
      clickBack = false;
    }
  }else if(event->key()==Qt::Key_A){
    ui->left->setStyleSheet("color:red;");
    if(clickTurnLeft){
      if (z_send < 1){
        z_send+=0.01;
      }
      emit movemsg(0,0,z_send );
      showSpeed(0,0, z_send);
      clickTurnLeft = true ;
    }else{
      sendDataInit();
      emit movemsg(0,0,z_send);
      showSpeed(0,0, z_send);
      clickTurnLeft = true;
    }
  }else if(event->key()==Qt::Key_X){
    ui->backward->setStyleSheet("color:red;");
    if(clickBack){
      if (x_send < 1){
        x_send+=0.01;
      }
      emit movemsg(x_send * (-1),0,0);
      showSpeed(x_send * (-1),0,0);
      clickForward = false;
      clickBack = true;
    }else{
      sendDataInit();
      emit movemsg(x_send * (-1),0,0);
      showSpeed(x_send * (-1),0,0);
      clickForward = false;
      clickBack = true;
    }
  }else if(event->key()==Qt::Key_D){
    ui->right->setStyleSheet("color:red;");
    if(clickTurnRight){
      if (z_send < 1){
        z_send+=0.01;
      }
      emit movemsg(0,0,z_send * (-1));
      showSpeed(0,0, z_send * (-1));
      clickTurnRight = true ;
    }else{
      sendDataInit();
      emit movemsg(0,0,z_send * (-1));
      showSpeed(0,0, z_send * (-1));
      clickTurnRight = true;
    }
  }else if(event->key()==Qt::Key_S){
    ui->stop->setStyleSheet("color:red;");
    emit movemsg(0,0,0);
    showSpeed(0,0,0);
    sendDataInit();
  }else if(event->key()==Qt::Key_I){
    ui->up->setStyleSheet("color:red;");
    emit armmsg(1,0.01,0,0);
    sendDataInit();
  }else if(event->key()==Qt::Key_J){
    ui->grab->setStyleSheet("color:red;");
    emit armmsg(0,0,1,1);
    sendDataInit();
  }else if(event->key()==Qt::Key_K){
    ui->down->setStyleSheet("color:red;");
    emit armmsg(2,0.01,0,0);
    sendDataInit();
  }else if(event->key()==Qt::Key_L){
    ui->release->setStyleSheet("color:red;");
    emit armmsg(0,0,2,1);
    sendDataInit();
  }else if(event->key()==Qt::Key_Q){
    ui->left_forward->setStyleSheet("color:red;");
    emit movemsg(0.3,0,0.3);
    showSpeed(0.3,0,0.3);
    sendDataInit();
  }else if(event->key()==Qt::Key_E){
    ui->right_forward->setStyleSheet("color:red;");
    emit movemsg(0.3,0,-0.3);
    showSpeed(0.3,0,-0.3);
    sendDataInit();
  }else if(event->key()==Qt::Key_Z){
    ui->left_back->setStyleSheet("color:red;");
    emit movemsg(-0.3,0,0.3);
    showSpeed(-0.3,0,0.3);
    sendDataInit();
  }else if(event->key()==Qt::Key_C){
    ui->right_back->setStyleSheet("color:red;");
    emit movemsg(-0.3,0,-0.3);
    showSpeed(-0.3,0,-0.3);
    sendDataInit();
  }else if(event->key()==Qt::Key_O){
    ui->recover->setStyleSheet("color:red;");
    emit armmsg(0,0,0,0);
    sendDataInit();
  }
}
void userControl::keyReleaseEvent(QKeyEvent *event) {
  //键盘松开事件
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
  }else if(event->key()==Qt::Key_I){
    ui->up->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_J){
    ui->grab->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_K){
    ui->down->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_L){
    ui->release->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_Q){
    ui->left_forward->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_E){
    ui->right_forward->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_Z){
    ui->left_back->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_C){
    ui->right_back->setStyleSheet("color:black;");
  }else if(event->key()==Qt::Key_O){
    ui->recover->setStyleSheet("color:black;");
  }
}

void userControl::on_forward_clicked()
{
  if(clickForward){
    if (x_send < 1){
      x_send+=0.01;
    }
    emit movemsg(x_send,0,0);
    showSpeed(x_send,0,0);
    clickForward = true;
    clickBack = false;
  }else{
    sendDataInit();
    emit movemsg(x_send,0,0);
    showSpeed(x_send,0,0);
    clickForward = true;
    clickBack = false;
  }
}

void userControl::on_left_clicked()
{
  emit movemsg(0,0,0.6);
  showSpeed(0,0,0.6);
  sendDataInit();
}

void userControl::on_backward_clicked()
{
  if(clickBack){
    if (x_send < 1){
      x_send+=0.01;
    }
    emit movemsg(x_send * (-1),0,0);
    showSpeed(x_send * (-1),0,0);
    clickForward = false;
    clickBack = true;
  }else{
    sendDataInit();
    emit movemsg(x_send * (-1),0,0);
    showSpeed(x_send * (-1),0,0);
    clickForward = false;
    clickBack = true;
  }
}

void userControl::on_right_clicked()
{
  if(clickTurnRight){
    if (z_send < 1){
      z_send+=0.01;
    }
    emit movemsg(0,0,z_send * (-1));
    showSpeed(0,0, z_send * (-1));
    clickTurnRight = true ;
  }else{
    sendDataInit();
    emit movemsg(0,0,z_send * (-1));
    showSpeed(0,0, z_send * (-1));
    clickTurnRight = true;
  }
}

void userControl::on_stop_clicked()
{
  emit movemsg(0,0,0);
  showSpeed(0,0,0);
  sendDataInit();
}

void userControl::on_left_forward_clicked()
{
  emit movemsg(0.3,0,0.3);
  showSpeed(0.3,0,0.3);
  sendDataInit();
}

void userControl::on_right_forward_clicked()
{
  emit movemsg(0.3,0,-0.3);
  showSpeed(0.3,0,-0.3);
  sendDataInit();
}

void userControl::on_left_back_clicked()
{
  emit movemsg(-0.3,0,0.3);
  showSpeed(-0.3,0,0.3);
  sendDataInit();
}

void userControl::on_right_back_clicked()
{
  emit movemsg(-0.3,0,-0.3);
  showSpeed(-0.3,0,-0.3);
  sendDataInit();
}

void userControl::on_up_clicked()
{
  emit armmsg(1,0.01,0,0);
  sendDataInit();
}

void userControl::on_down_clicked()
{
  emit armmsg(2,0.01,0,0);
  sendDataInit();
}

void userControl::on_grab_clicked()
{
  emit armmsg(0,0,1,1);
  sendDataInit();
}

void userControl::on_release_clicked()
{
  emit armmsg(0,0,2,1);
  sendDataInit();
}

void userControl::on_open_rviz_clicked()
{


}

void userControl::closeEvent(QCloseEvent *event)
{
  main->show();
  emit userControl_finish();
}

void userControl::setMain(fight_with_hair_ui::MainWindow *m){
  main=m;
}

void userControl::showSpeed(float x, float y, float z){
  x_speed->setValue(x * 50);
  y_speed->setValue(y * 50);
  z_speed->setValue(z * (-50));
}

void userControl::sendDataInit(){
  clickForward = false;
  clickBack = false;
  clickTurnLeft = false;
  clickTurnRight = false;
  x_send = 0.2;
  y_send = 0;
  z_send = 0.5;

}

void userControl::on_open_rviz_image_clicked()
{

}

void userControl::on_recover_clicked()
{
    sendDataInit();
    emit armmsg(0,0,0,0);
}
