#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Bool.h>
#include <wpb_home_behaviors/Coord.h>
#include <fwh_obj/tarobj.h>

static bool ifbegin = true;
static ros::Publisher obj_uploader;
static ros::Publisher tar_uploader;
float x[100],y[100],z[100];

void obj_start(const std_msgs::Bool::ConstPtr &msg){
    ifbegin=true;
    ROS_WARN("begin_got_from_ui!");
}

void obj_detec_upload(const wpb_home_behaviors::Coord::ConstPtr &msg){
    ROS_WARN("obj_got_from_ui! _stop");
    if (ifbegin == true) {
        ifbegin=false;
        ROS_WARN("obj_got_from_ui! _begin");
        int objNum = msg->name.size();
        std_msgs::UInt32 uptp;
        uptp.data=objNum;
        obj_uploader.publish(uptp);
        ROS_WARN("objNum = %d",objNum);
        for(int i=0 ; i < objNum ; i++){
            ROS_WARN("obj[%s]'s location is (%.2f , %.2f , %.2f)",msg->name[i].c_str(),msg->x[i],msg->y[i],msg->z[i]);
            x[i]=msg->x[i];
            y[i]=msg->y[i];
            z[i]=msg->z[i];
            ROS_WARN("temp[%d]'s location is (%.2f , %.2f , %.2f)",i,x[i],y[i],z[i]);
        }
    }
}

void obj_tar_upload(const std_msgs::UInt32::ConstPtr &msg){
    ROS_WARN("tar_num = %d",msg->data);
    fwh_obj::tarobj uptp;
    uptp.x=x[msg->data];
    uptp.y=y[msg->data];
    uptp.z=z[msg->data];
    ROS_WARN("tar's location is (%.2f , %.2f , %.2f)",uptp.x,uptp.y,uptp.z);
    tar_uploader.publish(uptp);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "fwh_obj_detect");
    ROS_WARN("fwh_obj_detect start!");
    ros::NodeHandle nh;
    ros::Subscriber ifstart = nh.subscribe("fight_with_hair/uito/obj_begin", 1, obj_start);
    ros::Subscriber obj_sub = nh.subscribe("/wpb_home/objects_3d", 1, obj_detec_upload);
    obj_uploader = nh.advertise<std_msgs::UInt32>("fight_with_hair/toui/obj_num",1);
    ros::Subscriber tarnum_sub = nh.subscribe("fight_with_hair/uito/obj_tar", 1, obj_tar_upload);
    tar_uploader = nh.advertise<fwh_obj::tarobj>("/tograb/obj_target",1);

    
    ros::spin();
    return 0;

}
