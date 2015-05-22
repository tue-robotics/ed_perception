/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#include "entity_viewer_probe.h"

#include "../plugins/shared_methods.h"

// ED
#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/error_context.h>
#include <ed/io/filesystem/write.h>
#include <rgbd/Image.h>
#include <rgbd/serialization.h>

#include <boost/filesystem.hpp>


// ----------------------------------------------------------------------------------------------------

EntityViewerProbe::EntityViewerProbe(){
    module_name_ = "[Entity Live Viewer] ";
    preview_size_ = 400;

    char const* home = getenv("HOME");

    save_dir_ = std::string(home) + "/entity_viewer/";
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

    ed::ErrorContext errc("EntityViewerProbe -> process()");

    parseReq(world, req, res);
}


// ----------------------------------------------------------------------------------------------------


void EntityViewerProbe::parseReq(const ed::WorldModel& world, tue::serialization::InputArchive& req, tue::serialization::OutputArchive& res){
    ed::ErrorContext errc("EntityViewerProbe -> parseReq()");

    int type;
    std::string id;
    std::string model_name;
    req >> type;

    switch (type) {
        case viewer_common::GET_ENTITY_LIST:
            getEntityList(res, world);
            break;

        case viewer_common::GET_ENTITY_ROI:
            req >> id;
            getEntityImage(res, id, world);
            break;

        case viewer_common::STORE_MEASUREMENT:
            req >> id;
            req >> model_name;
            storeMeasurement(res, id, model_name, world);
            break;

        default:
            std::cout << module_name_ << "Unrecognized request!" << std::endl;
            break;
    }
}


// ----------------------------------------------------------------------------------------------------


void EntityViewerProbe::getEntityImage(tue::serialization::OutputArchive& res, const std::string &entity_id, const ed::WorldModel& world){
    ed::ErrorContext errc("EntityViewerProbe -> getEntityList()");

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it){
        const ed::EntityConstPtr& e = *it;

        // filter entities
        if (!e->shape() && !e->convexHull().points.empty()){
            // comare IDs
            if (entity_id == e->id().str()){
                
                // save last entity requested
                last_entity_ = *it;
                        
                // ---------- Prepare measurement ----------

                // Get the best measurement from the entity
                ed::MeasurementConstPtr msr = e->lastMeasurement();

                if (!msr){
                    return;
                }

                // get color image
                const cv::Mat& color_image = msr->image()->getRGBImage();

                if (color_image.rows * color_image.cols > (1280*1024) || color_image.rows * color_image.cols == 0){
                    std::cout << "Bad image size: " << color_image.cols << "x" << color_image.rows << std::endl;
                }

                // get depth image
//                const cv::Mat& depth_image = msr->image()->getDepthImage();

                // Mask color image
                cv::Rect rgb_roi;
                cv::Mat color_image_masked = ed::perception::maskImage(color_image, msr->imageMask(), rgb_roi);

                // copy just the region of interest from the color image
                cv::Mat roi_masked;
                color_image_masked(rgb_roi).copyTo(roi_masked);

                // build response
                res << (int)roi_masked.rows;
                res << (int)roi_masked.cols;

                int size = roi_masked.cols * roi_masked.rows * 3;
                for(int i = 0; i < size; ++i)
                    res << roi_masked.data[i];
            }
        }
    }
}


// ----------------------------------------------------------------------------------------------------


void EntityViewerProbe::getEntityList(tue::serialization::OutputArchive& res, const ed::WorldModel& world){
    ed::ErrorContext errc("EntityViewerProbe -> getEntityList()");

    std::vector<viewer_common::EntityInfo> entity_list;

    // create model list
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it){
        const ed::EntityConstPtr& e = *it;

        // filter entities
        if (!e->shape() && !e->convexHull().points.empty()){
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


// ----------------------------------------------------------------------------------------------------


void EntityViewerProbe::storeMeasurement(tue::serialization::OutputArchive& res,
                                         const std::string &entity_id,
                                         const std::string &model_name,
                                         const ed::WorldModel& world){

    ed::ErrorContext errc("EntityViewerProbe -> storeMeasurement()");


    boost::filesystem::path dir(save_dir_ + model_name + "/");
    boost::filesystem::create_directories(dir);

    std::string filename = dir.string() + ed::Entity::generateID().str();
    ed::write(filename, *last_entity_);

    std::cout << module_name_ << "Writing measurement to '" << filename << "'." << std::endl;

    // generate response, no error
    res << 0;

    /*
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it){
        const ed::EntityConstPtr& e = *it;

        // filter entities
        if (!e->shape() && !e->convexHull().chull.empty()){

            // find the correct entity
            if (entity_id.compare(e->id().str()) == 0){
                boost::filesystem::path dir(save_dir_ + model_name + "/");
                boost::filesystem::create_directories(dir);

                std::string filename = dir.string() + ed::Entity::generateID().str();
                ed::write(filename, *e);

                std::cout << module_name_ << "Writing measurement to '" << filename << "'." << std::endl;

                // generate response, no error
                res << 0;
            }
        }
    }

    // generate response, error
    res << 1;

    */
}

ED_REGISTER_PLUGIN(EntityViewerProbe)
