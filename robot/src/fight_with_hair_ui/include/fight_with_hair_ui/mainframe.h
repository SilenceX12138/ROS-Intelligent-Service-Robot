#ifndef MAINFRAME_H
#define MAINFRAME_H

#include <QWidget>
#include "usercontrol.h"
#include "build_map.h"
#include "detection.h"
#include "spinslider.h"
#include "navigation.h"

namespace Ui {
class mainframe;
}

class mainframe : public QWidget
{
  Q_OBJECT

public:
  explicit mainframe(QWidget *parent = 0);
  ~mainframe();

private slots:
  void on_control_with_key_clicked();

  void on_navigation_clicked();

  void on_build_map_clicked();

  void on_detection_clicked();

  void on_logout_clicked();

private:
  Ui::mainframe *ui;
};

#endif // MAINFRAME_H
