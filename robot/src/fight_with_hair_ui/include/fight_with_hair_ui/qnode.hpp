/**
 * @file /include/fight_with_hair_ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef fight_with_hair_ui_QNODE_HPP_
#define fight_with_hair_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include <basic/moveMsg.h>
#include <arm/armMsg.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace fight_with_hair_ui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();


  /*********************
  ** Logging
  **********************/
  enum LogLevel {
           Debug,
           Info,
           Warn,
           Error,
           Fatal
   };

  QStringListModel* loggingModel() { return &logging_model; }
  void log( const LogLevel &level, const std::string &msg);
  void sendMoveMsg(float,float,float);
  void sendArmMsg(float,float,float,float);
  void sendObjSigMsg(bool);
  void sendObjDetectMsg(int);
  void sendVoiceSigMsg();
  void objFind_Listener(const std_msgs::UInt32ConstPtr &msg);
  void voice_Listener(const std_msgs::StringConstPtr &msg);
  void sendFollowEndMsg();
  void sendFollowBeginMsg();
Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();
  void objFind(int);
  void asr_order(std::string);


private:
  int init_argc;
  char** init_argv;
  ros::Publisher chatter_publisher;
    QStringListModel logging_model;
  ros::Publisher vel_publisher;
  ros::Publisher arm_publisher;
  ros::Publisher objSig_publisher;
  ros::Publisher objDetect_publisher;
  ros::Subscriber objFind_subscriber;
  ros::Publisher voice_publisher;
  ros::Subscriber voice_subscriber;
  ros::Publisher followEnd_publisher;
  ros::Publisher followBegin_publisher;
};

}  // namespace fight_with_hair_ui

#endif /* fight_with_hair_ui_QNODE_HPP_ */
