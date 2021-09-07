#include "8_innovation_script.h"
CInnovationScript::CInnovationScript()
{
    
}

CInnovationScript::~CInnovationScript()
{

}

void CInnovationScript::Queue()
{
    stAct newAct;
   
    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "拿杯水";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "好的,请稍等";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_ADD_WAYPOINT;
    newAct.strTarget = "master";
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "target";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "即将开始抓取";
    newAct.nDuration = 6;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GRAB;
    newAct.strTarget = "";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "物品抓取完毕";
    newAct.nDuration = 5;
    arAct.push_back(newAct);

    newAct.nAct = ACT_GOTO;
    newAct.strTarget = "master";
    arAct.push_back(newAct);

    newAct.nAct = ACT_SPEAK;
    newAct.strTarget = "物品已取回，请接收";
    newAct.nDuration = 5;
    arAct.push_back(newAct);


    newAct.nAct = ACT_LISTEN;
    newAct.strTarget = "给我";
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