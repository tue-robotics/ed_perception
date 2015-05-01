/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#include "entity_live_viewer.h"


//#include <ed/perception/aggregator.h>


// Measurement data structures
#include <ed/measurement.h>
#include <rgbd/Image.h>

// C++ IO
#include <iostream>

// ----------------------------------------------------------------------------------------------------


EntityLiveViewer::EntityLiveViewer(){
    std::cout << "Created viewer" << std::endl;

}

void EntityLiveViewer::updateViewer(){
//    std::cout << "Viewer updated" << std::endl;

}


void EntityLiveViewer::addEntity(const ed::EntityConstPtr& entity){
//    std::cout << "added entity" << std::endl;

    EntityInfo e_info (entity);
}
