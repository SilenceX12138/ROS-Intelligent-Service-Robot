#ifndef DETECTION_H
#define DETECTION_H

#include <QWidget>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>

#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <fight_with_hair_ui/main_window.hpp>

namespace Ui {
class detection;
}

class detection : public QWidget
{
  Q_OBJECT

public:
  explicit detection(bool isSim,QWidget *parent = 0);
  ~detection();

  void ros_init(int argc,char **argv);

  void setMain(fight_with_hair_ui::MainWindow*);

  void setEnvironment(bool);

public Q_SLOTS:
  void setObjectSize(int num);

private slots:
  void on_logout_clicked();

  void on_returnMain_clicked();

  void on_detect_start_clicked();

  void getAndSendObjectNum();

  void on_sendObjNum_clicked();

Q_SIGNALS:
  void sendObjectNum(int);
  void sendDetectMsg(bool);
  void detection_finish();

private:
  Ui::detection *ui;
  rviz::VisualizationManager *manager_;
  rviz::RenderPanel *render_panel_;
  int objectSize=0;
  fight_with_hair_ui::MainWindow *main;
  int choseObjNum=-1;
  bool ifIsSim = false;

protected:
  void closeEvent(QCloseEvent *event);
};

#endif // DETECTION_H
