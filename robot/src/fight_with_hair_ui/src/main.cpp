/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/fight_with_hair_ui/main_window.hpp"
#include "../include/fight_with_hair_ui/loginwindow.h"
#include "../include/fight_with_hair_ui/spinslider.h"
#include "../include/fight_with_hair_ui/build_map_auto.h"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

  /*********************
    ** Qt
    **********************/
  QApplication app(argc, argv);
  QString str = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch wpb_sim wpb_simple.launch'&";
  //  system(str.toLatin1().data());
  fight_with_hair_ui::MainWindow w(argc,argv);
  LoginWindow *login=new LoginWindow;
  login->setMain(&w);
  login->move(550,200);
  w.move(550,200);
  //  spinslider *n=new spinslider;
  //  n->show();
  //  w.show();
  login->show();

//  build_map_auto *b=new build_map_auto(true);
//  b->move(550,200);
//  b->show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
