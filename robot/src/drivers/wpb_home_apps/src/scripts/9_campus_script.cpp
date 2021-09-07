#include "9_campus_script.h"
CCampusScript::CCampusScript()
{
    
}

CCampusScript::~CCampusScript()
{

}

void CCampusScript::Queue()
{
    stAct newAct;

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "智慧校园 项目开始";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "机器人";   //建议台词"机器人"
    newAct.nDuration = 3;    //识别时长
    arAct.push_back(newAct);
    
    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "你好,小明,该去上历史课了";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "去吧";    //建议台词"我今天不舒服,你替我去吧"
    newAct.nDuration = 8;    //识别时长
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "好的,我会替你录制上课内容,回来让你看录像复习";
    newAct.nDuration = 8;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "class";     //在waterplus_map_tools里设置录像航点
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "开始录像";
    newAct.nDuration = 3;
    arAct.push_back(newAct);

    newAct.nAct = ACT_REC_VIDEO;
    newAct.strTarget = "/home/robot/record.avi";    //录制视频文件名
    newAct.nDuration = 3;   //录制时长(单位:秒)
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "好了,我已经录好上课内容,现在回去找小明";
    newAct.nDuration = 6;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "bed";     //在waterplus_map_tools里设置小明处航点
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "小明,我回来了,给你录好了今天上课视频,你可以看视频复习";
    newAct.nDuration = 7;
    arAct.push_back(newAct);

    newAct.nAct = ACT_PLAY_VIDEO;
    newAct.strTarget = "/home/robot/record.avi";    //播放视频文件名
    newAct.nLoopPlay = 5;   //循环播放次数
    newAct.nDuration = 10;   //持续时间(单位:秒)
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "小明,这就是今天的课程";
    newAct.nDuration = 3;
    arAct.push_back(newAct);
}