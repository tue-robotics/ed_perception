/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#include "entity_live_viewer.h"


//#include <ed/perception/aggregator.h>
#include <ed/error_context.h>

// Measurement data structures
#include <ed/measurement.h>
#include <rgbd/Image.h>

// C++ IO
#include <iostream>

#include "../plugins/shared_methods.h"

#include <tue/config/reader.h>

// ----------------------------------------------------------------------------------------------------


EntityLiveViewer::EntityLiveViewer(){
    std::cout << "Created viewer" << std::endl;

    window_margin_ = 30;
    preview_size_ = 400;

    cv::namedWindow("Entity Live Viewer", cv::WINDOW_AUTOSIZE);

}

// ----------------------------------------------------------------------------------------------------


void EntityLiveViewer::updateViewer(){
    ed::ErrorContext errc("EntityLiveViewer -> updateViewer()");

    cv::Mat output_img = cv::Mat::zeros(768, 1024, CV_8UC3);

    cv::Point entity_list_org(10, 30);

    // set ROIs
    cv::Mat entity_preview_roi = output_img(cv::Rect(window_margin_, window_margin_, preview_size_, preview_size_));
    cv::Mat entity_list_roi = output_img(cv::Rect(window_margin_, entity_preview_roi.rows + window_margin_, 480, output_img.rows - entity_preview_roi.rows - window_margin_));
    cv::Mat entity_info_roi = output_img(cv::Rect(512 + window_margin_, window_margin_, 480, 700));

    // paint background white
    entity_preview_roi.setTo(cv::Scalar(255,255,255));

    // ------------------------------------------------------

    // Draw entity masked image
    if (!entity_list_.empty()){
        cv::Mat resized_img = cv::Mat::zeros(preview_size_, preview_size_, CV_8UC3);
        resized_img = ed::perception::resizeSameRatio(entity_list_[0].masked_roi, preview_size_);
        resized_img.copyTo(entity_preview_roi);
    }

    // ------------------------------------------------------

    // Draw entity info

    if (!entity_list_.empty()){
//        cv::putText(entity_list_roi, entity_list_[0].data, cv::Point(10,10), 1, 1.1, cv::Scalar(255,255, 255), 1, CV_AA);
//        tue::config::Reader r(e_info.data);

//        std::cout << entity_list_[0].data << std::endl;
    }

    // ------------------------------------------------------
    std::cout << "List size: " << entity_list_.size() << std::endl;
    std::cout << "Entity List:" << std::endl;

    int vert_offset = 40;
    int counter = 1;

    // Draw entity list
    cv::putText(entity_list_roi, "Entity List:", entity_list_org, 1, 1.1, cv::Scalar(255,255, 255), 1, CV_AA);
    for(std::vector<EntityInfo>::const_iterator entity_it = entity_list_.begin(); entity_it != entity_list_.end(); ++entity_it){

        std::cout << "Entity ID: " << entity_it->id << std::endl;

        std::stringstream ss;
        ss << counter << ": " << (std::string)(entity_it->id).substr(0, 4);

        cv::putText(entity_list_roi, ss.str(), entity_list_org + cv::Point(0, vert_offset), 1, 1.1, cv::Scalar(255,255,255), 1, CV_AA);
        vert_offset += 20;
        counter++;
    }

    // ------------------------------------------------------

    entity_list_.clear();

    cv::imshow("Entity Live Viewer", output_img);
    key_pressed_ = cv::waitKey(10);

    processKeyPressed(key_pressed_);
}

// ----------------------------------------------------------------------------------------------------


void EntityLiveViewer::addEntity(const ed::EntityConstPtr& entity){
    ed::ErrorContext errc("EntityLiveViewer -> addEntity()");

    cv::Rect rgb_roi;

    EntityInfo e_info (entity);

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = entity->lastMeasurement();

    if (!msr)
        return;

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // get depth image
//    const cv::Mat& depth_image = msr->image()->getDepthImage();

    // mask color image with the entity segmentation
    cv::Mat entity_masked = ed::perception::maskImage(color_image, msr->imageMask(), rgb_roi);

    //copy the masked image to EntityInfo
    entity_masked(rgb_roi).copyTo(e_info.masked_roi);

//    tue::config::Reader r(e_info.data);

    entity_list_.push_back(e_info);
}


// ----------------------------------------------------------------------------------------------------


void EntityLiveViewer::processKeyPressed(int key){
    // return if no key was pressed
    if(key == -1) return;

    std::cout << "key pressed = " << key << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void EntityLiveViewer::myDrawMultiLineText(std::string InputParagraph, cv::Point Origin){
//    std::vector<std::string> LinesOfText = myParse(InputParagraph,"\n");
//    for (int i=0;i<LinesOfText.size(); ++i)
//        DrawText(CurrentLine[i], Origin.x, Origin.y + i*16);
}
