#ifndef USERCONTROL_H
#define USERCONTROL_H

#include <QWidget>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <rviz/tool.h>
#include <fight_with_hair_ui/main_window.hpp>
#include "cctrldashboard.h"


namespace Ui {
class userControl;
}

class userControl : public QWidget
{
  Q_OBJECT

public:
  explicit userControl(QWidget *parent = 0);
  ~userControl();

  void setMain(fight_with_hair_ui::MainWindow *);
  void sendDataInit();

Q_SIGNALS:
  void movemsg(float,float,float);
  void armmsg(float,float,float,float);
  void userControl_finish();

private slots:
  void on_logout_clicked();

  void on_returnMain_clicked();

  void on_forward_clicked();

  void on_left_clicked();

  void on_backward_clicked();

  void on_right_clicked();

  void on_stop_clicked();

  void on_left_forward_clicked();

  void on_right_forward_clicked();

  void on_left_back_clicked();

  void on_right_back_clicked();

  void on_up_clicked();

  void on_down_clicked();

  void on_grab_clicked();

  void on_release_clicked();

  void on_open_rviz_clicked();

  void showSpeed(float,float,float);

  void on_open_rviz_image_clicked();

  void on_recover_clicked();

private:
  Ui::userControl *ui;
  rviz::VisualizationManager *manager_;
  rviz::RenderPanel *render_panel_;
  fight_with_hair_ui::MainWindow *main;
  CCtrlDashBoard *x_speed;
  CCtrlDashBoard *y_speed;
  CCtrlDashBoard *z_speed;
  float x_send=0.2;
  float y_send=0;
  float z_send=0.5;
  bool clickForward = false;
  bool clickBack = false;
  bool clickTurnLeft = false;
  bool clickTurnRight = false;

protected:
  void keyPressEvent(QKeyEvent *ev);
  void keyReleaseEvent(QKeyEvent *ev);
  void closeEvent(QCloseEvent *event);
};

#endif // USERCONTROL_H
