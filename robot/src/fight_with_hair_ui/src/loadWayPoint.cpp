#include "../include/fight_with_hair_ui/loadWayPoint.h"



std::vector<WayPoint> LoadWaypointsFromFile(std::string inFilename)
{
    TiXmlDocument docLoad(inFilename);
    std::vector<WayPoint> arWaypoint;
    bool resLoad = docLoad.LoadFile();
    if(resLoad == false)
    {

        return arWaypoint;
    }
    
    TiXmlElement* RootElement = docLoad.RootElement();
    for(TiXmlNode* item = RootElement->FirstChild("Waypoint");item;item = item->NextSibling("Waypoint"))
    {
        TiXmlNode* child = item->FirstChild();
        const char* name = child->ToElement()->GetText();

        WayPoint newWayPoint;
        newWayPoint.name = std::string(name); 
        child = item->IterateChildren(child);
        const char* pos_x = child->ToElement()->GetText();
        newWayPoint.pos_x = std::atof(pos_x);
        child = item->IterateChildren(child);
        const char* pos_y = child->ToElement()->GetText();
        newWayPoint.pos_y = std::atof(pos_y);
        child = item->IterateChildren(child);
        const char* pos_z = child->ToElement()->GetText();
        newWayPoint.pos_z = std::atof(pos_z);
        child = item->IterateChildren(child);
        const char* ori_x = child->ToElement()->GetText();
        newWayPoint.ori_x = std::atof(ori_x);
        child = item->IterateChildren(child);
        const char* ori_y = child->ToElement()->GetText();
        newWayPoint.ori_y = std::atof(ori_y);
        child = item->IterateChildren(child);
        const char* ori_z = child->ToElement()->GetText();
        newWayPoint.ori_z = std::atof(ori_z);
        child = item->IterateChildren(child);
        const char* ori_w = child->ToElement()->GetText();
        newWayPoint.ori_w = std::atof(ori_w);
        arWaypoint.push_back(newWayPoint);
    }


    return arWaypoint;
}
