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
            std::string type;
            std::string data_str;
            tue::config::DataConstPointer data_ptr;
            double area;

            ed::EntityConstPtr entity_pt;
            cv::Mat color_img;
            cv::Mat depth_img;
            cv::Mat masked_roi;
            cv::Mat mask;
            cv::Rect roi;
            int last_updated;

            EntityInfo(std::string id_, std::string model_name_, std::string data, double area_):
                id(id_),
                type(model_name_),
                data_str(data),
                area(area_),
                last_updated(0){}

            void updateData(std::string data);
    };

    enum RequestType{
        GET_ENTITY_LIST = 0,
        GET_ENTITY_ROI = 1,
        STORE_MEASUREMENT = 2,
        GET_ENTITY_DATA = 3
    };


}

#endif
