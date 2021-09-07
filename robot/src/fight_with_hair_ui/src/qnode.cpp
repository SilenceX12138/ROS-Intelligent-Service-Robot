/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <sstream>
#include "../include/fight_with_hair_ui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace fight_with_hair_ui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
  {}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
  wait();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"fight_with_hair_ui");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  vel_publisher = n.advertise<basic::moveMsg>("/fight_with_hair/basic_move/vel",10);
  arm_publisher = n.advertise<arm::armMsg>("/fight_with_hair/arm_move",10);
  objSig_publisher = n.advertise<std_msgs::Bool>("/fight_with_hair/uito/obj_begin",10);
  objDetect_publisher = n.advertise<std_msgs::UInt32>("/fight_with_hair/uito/obj_tar",10);
  objFind_subscriber = n.subscribe<std_msgs::UInt32>("/fight_with_hair/toui/obj_num",10, &QNode::objFind_Listener, this);
  voice_publisher = n.advertise<std_msgs::UInt32>("/fight_with_hair/uito/state_sound", 1);

  voice_subscriber = n.subscribe<std_msgs::String>("/fight_with_hair/toui/sound_ins", 10,&QNode::voice_Listener,this);
  followEnd_publisher = n.advertise<std_msgs::UInt32>("/fight_with_hair/uito/follow_stop", 1);
  followBegin_publisher = n.advertise<std_msgs::UInt32>("/fight_with_hair/uito/follow_begin", 1);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"fight_with_hair_ui");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  // Add your ros communications here.
  chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  start();
  return true;
}

void QNode::run() {
//  ros::Rate loop_rate(1);
//  int count = 0;
//  while ( ros::ok() ) {

//    std_msgs::String msg;
//    std::stringstream ss;
//    ss << "hello world " << count;
//    msg.data = ss.str();
//    chatter_publisher.publish(msg);
//    log(Info,std::string("I sent: ")+msg.data);
//    ros::spinOnce();
//    loop_rate.sleep();
//    ++count;
//  }
//  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
//  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch ( level ) {
    case(Debug) : {
        ROS_DEBUG_STREAM(msg);
        logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Info) : {
        ROS_INFO_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Warn) : {
        ROS_WARN_STREAM(msg);
        logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Error) : {
        ROS_ERROR_STREAM(msg);
        logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
        break;
    }
    case(Fatal) : {
        ROS_FATAL_STREAM(msg);
        logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
        break;
    }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendMoveMsg(float x, float y, float z){
  basic::moveMsg vel;
  vel.x = x;
  vel.y = y;
  vel.z = z;
  vel_publisher.publish(vel);
}

void QNode::sendArmMsg(float dire_v, float len, float dire_h, float degree){
  arm::armMsg arm_vel;
  arm_vel.direction_v = dire_v;
  arm_vel.length = len;
  arm_vel.direction_h = dire_h;
  arm_vel.degree = degree;
  arm_publisher.publish(arm_vel);
}

void QNode::sendObjSigMsg(bool signal){
  std_msgs::Bool sig;
  sig.data = signal;
  objSig_publisher.publish(sig);
}

void QNode::sendObjDetectMsg(int obj_id){
  std_msgs::UInt32 id;
  id.data = obj_id;
  objDetect_publisher.publish(id);
}

void QNode::objFind_Listener(const std_msgs::UInt32ConstPtr &msg){
  int num = msg->data;
  Q_EMIT QNode::objFind(num);
}

void QNode::sendVoiceSigMsg(){
  std_msgs::UInt32 start;
  start.data = 1;
  voice_publisher.publish(start);
}

void QNode::voice_Listener(const std_msgs::StringConstPtr &msg){
  std::string str = msg->data;
  ROS_INFO("qqqqqq");
  emit asr_order(str);

}

void QNode::sendFollowEndMsg(){
  std_msgs::UInt32 end;
  end.data = 1;
  ROS_INFO("send start");
  followEnd_publisher.publish(end);
}

void QNode::sendFollowBeginMsg(){
  std_msgs::UInt32 begin;
  begin.data = 1;
  followBegin_publisher.publish(begin);
}

}  // namespace fight_with_hair_ui
