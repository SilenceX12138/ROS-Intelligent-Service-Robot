#ifndef FOLLOW_H
#define FOLLOW_H

#include <QWidget>
#include <fight_with_hair_ui/main_window.hpp>

namespace Ui {
class Follow;
}

class Follow : public QWidget
{
  Q_OBJECT

public:
  explicit Follow(bool isSim,QWidget *parent = 0);
  ~Follow();

  void setMain(fight_with_hair_ui::MainWindow*);

Q_SIGNALS:
  void send_follow_begin_msg();
  void send_follow_end_msg();
  void follow_finish();


private slots:
  void on_returnMain_clicked();

  void on_logout_clicked();

  void on_follow_begin_clicked();

  void on_follow_end_clicked();

private:
  Ui::Follow *ui;
  fight_with_hair_ui::MainWindow *main;

protected:
  void closeEvent(QCloseEvent *event);
};

#endif // FOLLOW_H
