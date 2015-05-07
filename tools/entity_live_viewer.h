
#ifndef ENTITY_LIVE_VIEWER_H_
#define ENTITY_LIVE_VIEWER_H_

// Show measurement
#include <opencv2/highgui/highgui.hpp>

// ED
#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/perception/module.h>

// ----------------------------------------------------------------------------------------------------

class EntityInfo{

public:
    ed::EntityConstPtr entity_pointer;
    std::string id;
    cv::Mat color_img;
    cv::Mat depth_img;
    cv::Mat masked_roi;
    cv::Mat mask;
    cv::Rect roi;
    tue::config::DataConstPointer data;
    int last_updated;

//    bool updated;

    EntityInfo(const ed::EntityConstPtr& e)
        : entity_pointer(e),
          id(e->id().str()),
          data(e->data()),
          last_updated(0){}
};


// ---------------------------------------------------

class EntityLiveViewer{
    public:

       EntityLiveViewer();

       // FUNCTIONS
       void addEntity(const ed::EntityConstPtr& entity);
       void updateViewer();

    private:

        // CONFIGURATION VARIABLES
        std::string module_name_;
        std::string debug_dir_;
        std::vector<EntityInfo> entity_list_;
        int max_age_;
        int focused_idx_;
        char key_pressed_;
        int window_margin_;
        int preview_size_;
        bool viewer_freeze_;

        void processKeyPressed(char key);
        void putTextMultipleLines(std::string text, std::string delimiter, cv::Point origin, cv::Mat& image_out);
        void storeMeasurement(const ed::EntityConstPtr &entity, const std::string& type);
};

#endif
