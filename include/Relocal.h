//
// Created by wu6shen on 18-7-31.
//

#ifndef RELOCAL_RELOCAL_H
#define RELOCAL_RELOCAL_H

#include "Frame.h"
#include "FrameDrawer.h"
#include "MapPoint.h"
#include "MapDrawer.h"
#include "Map.h"

#include <iostream>
#include <cstring>
#include <cstdio>

namespace ORB_SLAM2{
    class Relocal {
    public:
        Map *mpLastMap;
    };
}

#endif //RELOCAL_RELOCAL_H
