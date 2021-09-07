#ifndef SPINSLIDER_H
#define SPINSLIDER_H

#include <QWidget>
#include "cctrldashboard.h"

namespace Ui {
class spinslider;
}

class spinslider : public QWidget
{
  Q_OBJECT

public:
  explicit spinslider(QWidget *parent = 0);
  ~spinslider();

private:
  Ui::spinslider *ui;
  CCtrlDashBoard *DashBoard_x;
};

#endif // SPINSLIDER_H
