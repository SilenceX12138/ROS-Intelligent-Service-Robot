/**
 * @file /include/fight_with_hair_ui/main_window.hpp
 *
 * @brief Qt based gui for fight_with_hair_ui.
 *
 * @date November 2010
 **/
#ifndef fight_with_hair_ui_MAIN_WINDOW_H
#define fight_with_hair_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <string>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace fight_with_hair_ui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();
  void ReadSettings(); // Load up qt program settings at startup
  void WriteSettings(); // Save qt program settings when closing

  void closeEvent(QCloseEvent *event); // Overloaded function

  void chooseEnvironment();

public Q_SLOTS:
  void receive_asr_orders(std::string);
  /******************************************
  ** Auto-connections (connectSlotsByName())
  *******************************************/


private slots:
  void on_control_with_key_clicked();

  void on_navigation_clicked();

  void on_build_map_clicked();

  void on_detection_clicked();

  void on_logout_clicked();

  void on_follow_clicked();

  void on_ASR_clicked();

  void on_aaaaa_clicked();

  void on_autoMap_clicked();

Q_SIGNALS:
  void send_follow_end_msg();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;
  bool isSim=false;
};

}  // namespace fight_with_hair_ui

#endif // fight_with_hair_ui_MAIN_WINDOW_H
