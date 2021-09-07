#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

static ros::Publisher spk_pub;
static ros::Publisher ins_pub;
static std_msgs::String speaker_str;
static std_msgs::String ins_str;
static bool ifdoing = false;

void sound_check(const std_msgs::String::ConstPtr & msg){
    if (ifdoing == false) {
        printf("[识别到的语音] - %s \n",msg->data.c_str());
        int num = 0;
        num = msg->data.find("启动目标识别");
        if( num >= 0 ){
            speaker_str.data = "收到，正在启动目标识别";
            spk_pub.publish(speaker_str);
            ins_str.data = "obj";
            ins_pub.publish(ins_str);
            ifdoing = true;
        }
        num = msg->data.find("迷路");
        num = num + msg->data.find("启动导航");
        if( num >= -1 ){
            speaker_str.data = "收到，正在启动导航";
            spk_pub.publish(speaker_str);
            ins_str.data = "navi";
            ins_pub.publish(ins_str);
            ifdoing = true;
        }
        num = msg->data.find("控制");
        if( num >= 0 ){
            speaker_str.data = "收到，正在启动控制功能";
            spk_pub.publish(speaker_str);
            ins_str.data = "mani";
            ins_pub.publish(ins_str);
            ifdoing = true;
        }
        num = msg->data.find("跟着我");
        num = num + msg->data.find("跟随");
        if( num >= -1 ){
            speaker_str.data = "收到，正在启动跟随功能";
            spk_pub.publish(speaker_str);
            ins_str.data = "follow";
            ins_pub.publish(ins_str);
            ifdoing = true;
        }
        num = msg->data.find("建图");
        num = num + msg->data.find("建立地图");
        if( num >= -1){
            speaker_str.data = "收到，正在启动建图功能";
            spk_pub.publish(speaker_str);
            ins_str.data = "build";
            ins_pub.publish(ins_str);
            ifdoing = true;
        }
    }
    else {
        speaker_str.data = "稍等，正在执行前一条指令";
        spk_pub.publish(speaker_str);
    }
}

void state_change(const std_msgs::UInt32::ConstPtr & msg){
    ROS_WARN("finish get!");
    ifdoing=false;
    speaker_str.data = "执行完毕，可以输入下一条指令";
    spk_pub.publish(speaker_str);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "fwh_sound_yz");
    ROS_WARN("fwh_sound_yz start!");
    ros::NodeHandle nh;
    ros::Subscriber ifdo = nh.subscribe("/fight_with_hair/uito/state_sound", 1, state_change);
    ins_pub = nh.advertise<std_msgs::String>("/fight_with_hair/toui/sound_ins", 10);
    ros::Subscriber sound = nh.subscribe("/xfyun/iat", 10, sound_check);
    spk_pub = nh.advertise<std_msgs::String>("/xfyun/tts", 10);
    ros::spin();
    return 0;
}
