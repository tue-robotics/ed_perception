/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#ifndef ENTITY_LIVE_VIEWER_CV_H_
#define ENTITY_LIVE_VIEWER_CV_H_

// OpenCV
#include <opencv2/highgui/highgui.hpp>

// ED
#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/perception/module.h>
#include <ed/io/transport/probe_client.h>

// Entity Viewer Common
#include "entity_viewer_common.h"


// ----------------------------------------------------------------------------------------------------

/*
class EntityInfo{

public:
    ed::EntityConstPtr entity_pt;
    std::string id;
    cv::Mat color_img;
    cv::Mat depth_img;
    cv::Mat masked_roi;
    cv::Mat mask;
    cv::Rect roi;
    tue::config::DataConstPointer data;
    int last_updated;

    EntityInfo(const ed::EntityConstPtr& e)
        : entity_pt(e),
          id(e->id().str()),
          data(e->data()),
          last_updated(0){}
};
*/

// ---------------------------------------------------


class EntityLiveViewerCV{
    public:

       EntityLiveViewerCV();

       // FUNCTIONS
       int mainLoop();

    private:

        // CONFIGURATION VARIABLES
        std::string module_name_;
        std::string debug_dir_;

        int max_age_;
        int focused_idx_;
        int window_margin_;
        int preview_size_;
        bool state_freeze_;
        std::string model_name_;
        int saved_measurements_;
        int font_face_;
        double font_scale_;
        ed::ProbeClient client_;
        bool exit_;
        int sleep_interval_;

//        tue::serialization::Archive req_;
//        tue::serialization::Archive res_;

        void addEntity(const ed::EntityConstPtr& entity);
        void updateViewer(std::vector<viewer_common::EntityInfo> &entity_list);
        void processKeyPressed(char key, std::vector<viewer_common::EntityInfo>& list);
        void putTextMultipleLines(std::string text, std::string delimiter, cv::Point origin, cv::Mat& image_out);
        void storeMeasurement(const ed::EntityConstPtr &entity, const std::string& type);

        int requestEntityList(std::vector<viewer_common::EntityInfo> &list);
        int requestEntityROI(std::string id, cv::Mat& roi);
};

#endif
