/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#include "entity_viewer_probe.h"

// ED
#include <ed/world_model.h>
#include <ed/entity.h>
#include <rgbd/Image.h>
#include <rgbd/serialization.h>

#include "../plugins/shared_methods.h"

// ----------------------------------------------------------------------------------------------------

EntityViewerProbe::EntityViewerProbe(){
    module_name_ = "[Entity Live Viewer] ";
}

// ----------------------------------------------------------------------------------------------------

EntityViewerProbe::~EntityViewerProbe(){

}

// ----------------------------------------------------------------------------------------------------

void EntityViewerProbe::configure(tue::Configuration config)
{


}

// ----------------------------------------------------------------------------------------------------

void EntityViewerProbe::process(const ed::WorldModel& world,
             ed::UpdateRequest& update,
             tue::serialization::InputArchive& req,
             tue::serialization::OutputArchive& res){   

    parseReq(world, req, res);
}


// ----------------------------------------------------------------------------------------------------


void EntityViewerProbe::parseReq(   const ed::WorldModel& world,
                                    tue::serialization::InputArchive& req,
                                    tue::serialization::OutputArchive& res){

    int type;
    std::string id;
    req >> type;

    switch (type) {
        case viewer_common::GET_ENTITY_LIST:
            getEntityList(res, world);
            break;

        case viewer_common::GET_ENTITY_ROI:
            req >> id;
            getEntityImage(res, id, world);
            break;

        default:
            std::cout << module_name_ << "Unrecognized request!" << std::endl;
            break;
    }
}


// ----------------------------------------------------------------------------------------------------


void EntityViewerProbe::getEntityImage(tue::serialization::OutputArchive& res, std::string entity_id, const ed::WorldModel& world){

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        // filter entities
        if (!e->shape() && !e->convexHull().chull.empty()){

            if (entity_id.compare(e->id().str()) == 0){
                // ---------- Prepare measurement ----------

                // Get the best measurement from the entity
                ed::MeasurementConstPtr msr = e->lastMeasurement();

                if (!msr){
                    return;
                }

                // get color image
                const cv::Mat& color_image = msr->image()->getRGBImage();

                // get depth image
//                const cv::Mat& depth_image = msr->image()->getDepthImage();

                // Mask color image
                cv::Rect rgb_roi;
                cv::Mat color_image_masked = ed::perception::maskImage(color_image, msr->imageMask(), rgb_roi);

                // build response
                e->lastMeasurement();
                res << (int)color_image_masked.rows;
                res << (int)color_image_masked.cols;

                int size = color_image_masked.cols * color_image_masked.rows * 3;
                for(int i = 0; i < size; ++i)
                    res << color_image_masked.data[i];
            }
        }
    }
}


// ----------------------------------------------------------------------------------------------------


void EntityViewerProbe::getEntityList(tue::serialization::OutputArchive& res, const ed::WorldModel& world){

    std::vector<viewer_common::EntityInfo> entity_list;

    // create model list
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        // filter entities
        if (!e->shape() && !e->convexHull().chull.empty()){
            std::stringstream data;
            data << e->data();

            entity_list.push_back(viewer_common::EntityInfo(e->id().str(), e->type(), data.str()));
        }
    }

    // generate response
    res << (int)entity_list.size();

    for (std::vector<viewer_common::EntityInfo>::const_iterator it = entity_list.begin() ; it != entity_list.end(); ++it){
        res << (std::string)it->id << (std::string)it->model_name << (std::string)it->data_str;
    }
}


ED_REGISTER_PLUGIN(EntityViewerProbe)
