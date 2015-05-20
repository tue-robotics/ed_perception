/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#ifndef ENTITY_VIEWER_COMMON_H_
#define ENTITY_VIEWER_COMMON_H_

#include <opencv2/highgui/highgui.hpp>

// C++ IO
#include <vector>
#include <stdio.h>
#include <string>

// ED
#include <ed/entity.h>

namespace viewer_common
{

    class EntityInfo{
        public:
            std::string id;
            std::string model_name;
            std::string data_str;

            ed::EntityConstPtr entity_pt;
            cv::Mat color_img;
            cv::Mat depth_img;
            cv::Mat masked_roi;
            cv::Mat mask;
            cv::Rect roi;
            tue::config::DataConstPointer data_ptr;
            int last_updated;

            EntityInfo(std::string id_, std::string model_name_, std::string data) :
                id(id_),
                model_name(model_name_),
                data_str(data),
                last_updated(0){}

            EntityInfo(const ed::EntityConstPtr& e)
                : entity_pt(e),
                  id(e->id().str()),
                  data_ptr(e->data()),
                  last_updated(0){}
    };

    enum RequestType{
        GET_ENTITY_LIST = 0,
        GET_ENTITY_ROI = 1
    };

}

#endif
