#pragma once

#include <tinyxml.h>
#include <vector>
#include <iostream>
#include <cstring>
#include <cstdlib>

struct WayPoint {
    std::string name;
    float pos_x, pos_y, pos_z;
    float ori_x, ori_y, ori_z, ori_w;
    
    WayPoint() {
        name = "";
        pos_x = pos_y = pos_z = 0;
        ori_x = ori_y = ori_z = ori_w = 0;
    }
};

std::vector<WayPoint> LoadWaypointsFromFile(std::string inFilename);