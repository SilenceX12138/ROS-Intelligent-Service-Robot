#ifndef BUILD_MAP_H
#define BUILD_MAP_H

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
class build_map;
}

class build_map : public QWidget
{
  Q_OBJECT

public:
  explicit build_map(bool isSim,QWidget *parent = 0);
  ~build_map();

  void ros_init(int argc,char **argv);

  void setMain(fight_with_hair_ui::MainWindow*);

  void setEnvironment(bool);


Q_SIGNALS:
  void movemsg(float,float,float);
  void build_map_finish();

private slots:
  void on_logout_clicked();

  void on_returnMain_clicked();

  void on_add_map_button_clicked();

  void on_map_show_btn_clicked();

  void on_forward_clicked();

  void on_right_forward_clicked();

  void on_right_clicked();

  void on_right_back_clicked();

  void on_backward_clicked();

  void on_left_back_clicked();

  void on_left_clicked();

  void on_left_forward_clicked();

  void on_stop_clicked();

private:
  Ui::build_map *ui;
  rviz::VisualizationManager *manager_;
  rviz::RenderPanel *render_panel_;
  rviz::Display *map_;
  QTimer *m_timer;
  bool hasSaved=false;
  fight_with_hair_ui::MainWindow *main;
  bool isSim=false;

protected:
  void keyPressEvent(QKeyEvent *ev);
  void keyReleaseEvent(QKeyEvent *ev);
  void closeEvent(QCloseEvent *event);
};

#endif // BUILD_MAP_H
