#include "10_home_script.h"
CHomeScript::CHomeScript()
{
    
}

CHomeScript::~CHomeScript()
{

}

void CHomeScript::Queue()
{
    stAct newAct;

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "科大讯飞语音交互 Demo";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "机器人";
    newAct.nDuration = 5;
    arAct.push_back(newAct);
    
    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "你好,已经跟着你了,主人";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_FOLLOW;
    newAct.fFollowDist = 0.5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "我渴了";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "好的,主人你想来瓶饮料吗";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "是的";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "好的,这就给您去取";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_ADD_WAYPOINT;
    newAct.strTarget = "follow";
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "kitchen";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "开始抓取饮料";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GRAB;
    newAct.strTarget = "";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "饮料抓取完毕";
    newAct.nDuration = 2;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "follow";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "饮料已取回,主人请慢用";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_PASS;
    newAct.strTarget = "";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "任务完成";
    newAct.nDuration = 5;
    arAct.push_back(newAct);
}