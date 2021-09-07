#ifndef LOGINWINDOW_H
#define LOGINWINDOW_H

#include <QWidget>
#include "mainframe.h"
#include <fight_with_hair_ui/main_window.hpp>

namespace Ui {
class LoginWindow;
}

class LoginWindow : public QWidget
{
  Q_OBJECT

public:
  explicit LoginWindow(QWidget *parent = 0);
  ~LoginWindow();

  void setMain(fight_with_hair_ui::MainWindow *);

private slots:
  void on_loginButton_clicked();

  void on_input_password_returnPressed();

  void on_resetButton_clicked();

private:
  Ui::LoginWindow *ui;
  fight_with_hair_ui::MainWindow *main;

protected:
  void closeEvent(QCloseEvent *event);
};

#endif // LOGINWINDOW_H
