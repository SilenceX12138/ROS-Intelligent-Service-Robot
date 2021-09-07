#ifndef BUILD_MAP_AUTO_H
#define BUILD_MAP_AUTO_H

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
class build_map_auto;
}

class build_map_auto : public QWidget
{
  Q_OBJECT

public:
  explicit build_map_auto(bool isSim,QWidget *parent = 0);
  ~build_map_auto();

  void setMain(fight_with_hair_ui::MainWindow *);

Q_SIGNALS:
  void build_map_auto_finish();

private slots:
  void on_add_map_button_clicked();

  void on_returnMain_clicked();

  void on_logout_clicked();

  void on_addNaviGoal_clicked();

  void on_addPublishPoint_clicked();

  void on_map_show_btn_clicked();

private:
  Ui::build_map_auto *ui;
  rviz::VisualizationManager *manager_;
  rviz::RenderPanel *render_panel_;
  rviz::Display *map_;
  QTimer *m_timer;
  rviz::Tool *current_tool_;
  rviz::ToolManager *tool_manager_;
  bool hasSaved=false;
  fight_with_hair_ui::MainWindow *main;
  bool isSim=false;

protected:
  void closeEvent(QCloseEvent *event);

};

#endif // BUILD_MAP_AUTO_H
