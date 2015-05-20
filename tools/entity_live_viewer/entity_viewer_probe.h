/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#ifndef ENTITY_VIEWER_PROBE_H_
#define ENTITY_VIEWER_PROBE_H_

#include <ed/io/transport/probe.h>

// C++ IO
#include <iostream>
#include <vector>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

// Entity Viewer Common
#include "entity_viewer_common.h"

// ----------------------------------------------------------------------------------------------------

class EntityViewerProbe : public ed::Probe{

public:

    EntityViewerProbe();

    virtual ~EntityViewerProbe();

    void configure(tue::Configuration config);

    void process(   const ed::WorldModel& world,
                    ed::UpdateRequest& update,
                    tue::serialization::InputArchive& req,
                    tue::serialization::OutputArchive& res);

private:

    std::string module_name_;
    std::vector<viewer_common::EntityInfo> entity_list_;
	
    void parseReq(  const ed::WorldModel& world,
                    tue::serialization::InputArchive& req,
                    tue::serialization::OutputArchive& res);

    void getEntityList(tue::serialization::OutputArchive& res, const ed::WorldModel &world);
    void getEntityImage(tue::serialization::OutputArchive& res, std::string entity_id, const ed::WorldModel &world);

};

#endif
