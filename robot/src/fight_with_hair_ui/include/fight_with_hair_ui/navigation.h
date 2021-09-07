#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <QWidget>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>

#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <rviz/tool.h>
#include <QString>
#include "myviz.h"
#include "fight_with_hair_ui/loadWayPoint.h"
#include <waterplus_map_tools/Waypoint.h>
#include <vector>
#include <fight_with_hair_ui/main_window.hpp>

namespace Ui {
class navigation;
}

class navigation : public QWidget
{
  Q_OBJECT

public:
  explicit navigation(bool isS,QWidget *parent = 0);
  ~navigation();

  void ros_init(int argc,char **argv);

  void setMain(fight_with_hair_ui::MainWindow *);

Q_SIGNALS:
  void navigationmsg_create(WayPoint);
  void navigation_finish();


private slots:
  void on_logout_clicked();

  void on_returnMain_clicked();

  void on_display_button_clicked();

  void on_chooseMap_clicked();

  void getButtonText(bool check);

  void on_chooseDes_clicked();

  void on_chooseWayPoint_clicked();

  void on_cruise_button_clicked();

  void on_save_waypoints_clicked();

private:
  Ui::navigation *ui;
  rviz::VisualizationManager *manager_;
  rviz::RenderPanel *render_panel_;
  rviz::Display *map_;
  rviz::Tool *current_tool_;
  rviz::ToolManager *tool_manager_;
  fight_with_hair_ui::MainWindow *main;

  QString mapfile;
  QString mapname;
  QString xmlfile;

  bool isSim;
protected:
  void closeEvent(QCloseEvent *event);
};

#endif // NAVIGATION_H
