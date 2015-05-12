/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: May 2015
*/

#include "entity_live_viewer_server.h"
#include "../../plugins/shared_methods.h"


// ED data structures
#include <ed/error_context.h>
#include <ed/measurement.h>
#include <ed/io/filesystem/write.h>
#include <rgbd/Image.h>
#include <tue/config/reader.h>

// C++ IO
#include <iostream>
#include <boost/filesystem.hpp>

// ----------------------------------------------------------------------------------------------------


EntityLiveViewerServer::EntityLiveViewerServer(){

    window_margin_ = 20;
    preview_size_ = 400;
    max_age_ = 10;
    focused_idx_ = 0;
    viewer_freeze_ = false;
    module_name_ = "[Entity Live Viewer] ";
    model_name_ = "default";
    saved_measurements_ = 0;
    font_face_ = cv::FONT_HERSHEY_SIMPLEX;
    font_scale_ = 0.5;

    std::cout << module_name_ << "Ready!" << std::endl;
    std::cout << module_name_ << "How to use: " << std::endl;
    std::cout << module_name_ << "\t1 - 9 : Choose entity from the list " << std::endl;
    std::cout << module_name_ << "\tSPACE : Freeze the viewer" << std::endl;
    std::cout << module_name_ << "\tS : Store measurement" << std::endl;
    std::cout << module_name_ << "\tN : Change name used for the measurement" << std::endl;

    cv::namedWindow("Entity Live Viewer", cv::WINDOW_AUTOSIZE);

    // mock parameters
//    char *argv[] = {"program name", "arg1", "arg2", NULL};
//    int argc = sizeof(argv) / sizeof(char*) - 1;

//    QApplication app_local(argc, argv);
//    viewer_ui_->show();
//    viewer_ui_->setlabeltext("bla");
//    app_local.exec();
//    std::cout << module_name_  << "Viewer ready" << std::endl;
}

// ----------------------------------------------------------------------------------------------------


void EntityLiveViewerServer::updateViewer(){
    ed::ErrorContext errc("EntityLiveViewer -> updateViewer()");

    cv::Mat output_img = cv::Mat::zeros(700, 800, CV_8UC3);

    // set ROIs
    cv::Mat entity_preview_roi = output_img(cv::Rect(0, 0, preview_size_, preview_size_));
    cv::Mat entity_list_roi = output_img(cv::Rect(0, entity_preview_roi.rows + window_margin_,
                                                  entity_preview_roi.cols + window_margin_, output_img.rows - entity_preview_roi.rows - window_margin_));
    cv::Mat entity_info_roi = output_img(cv::Rect(preview_size_ + window_margin_, 0,
                                                  output_img.cols - entity_preview_roi.cols - window_margin_, output_img.rows));

    // paint background white
    entity_preview_roi.setTo(cv::Scalar(255,255,255));
    entity_info_roi.setTo(cv::Scalar(30,30,30));
    entity_list_roi.setTo(cv::Scalar(30,30,30));


    // ------------------ ENTITY IMAGE ----------------------

    // Draw entity masked image
    if (!entity_list_.empty() && focused_idx_ < entity_list_.size()){
        cv::Mat resized_img = cv::Mat::zeros(preview_size_, preview_size_, CV_8UC3);
        resized_img = ed::perception::resizeSameRatio(entity_list_[focused_idx_].masked_roi, preview_size_);
        resized_img.copyTo(entity_preview_roi);
    }


    // ------------------ ENTITY INFO -----------------------

    if (!entity_list_.empty() && focused_idx_ < entity_list_.size()){
        std::stringstream data;
        data << entity_list_[focused_idx_].data;

        putTextMultipleLines(data.str(), "\n", cv::Point(10,10), entity_info_roi);
    }


    // -------------------ENTITY LIST ------------------------

    int vert_offset = 40;
    int counter = 1;
    cv::Point model_name_org(10, 30);
    cv::Point entity_list_org(10, 60);

    // Draw model name
    std::stringstream model_name_info;
    model_name_info << "Model name: " << model_name_;
    cv::putText(entity_list_roi, model_name_info.str(), model_name_org, font_face_, font_scale_, cv::Scalar(255,200, 200), 1, CV_AA);
    model_name_info.str("");
    model_name_info << "Saved counter: " << saved_measurements_;
    cv::putText(entity_list_roi, model_name_info.str(), model_name_org + cv::Point(230, 0), font_face_, font_scale_, cv::Scalar(255,200, 200), 1, CV_AA);

    // Draw entity list title
    std::stringstream list_title;
    list_title << "Entity List (" << entity_list_.size() << "):";
    cv::putText(entity_list_roi, list_title.str(), entity_list_org, font_face_, font_scale_+0.1, cv::Scalar(255,255, 255), 1, CV_AA);

    // Draw entity list
    for(std::vector<EntityInfo>::const_iterator entity_it = entity_list_.begin(); entity_it != entity_list_.end(); ++entity_it){
        std::stringstream ss;
        ss << "   " << counter << ": " << (std::string)(entity_it->id).substr(0, 4);

        // show selected entity
        if (counter-1 == focused_idx_)
            ss << "  <--";

        // draw text
        cv::putText(entity_list_roi, ss.str(), entity_list_org + cv::Point(0, vert_offset), font_face_, font_scale_, cv::Scalar(255,255,255), 1, CV_AA);
        vert_offset += 20;
        counter++;
    }

    // ------------------------------------------------------

    if (!viewer_freeze_){
        // update age of the entities in the list, and remove old ones
        std::vector<EntityInfo>::iterator it = entity_list_.begin();
        while(it != entity_list_.end()) {
            if(it->last_updated > max_age_){
                it = entity_list_.erase(it);
            }else{
                it->last_updated++;
                ++it;
            }
        }
    }

    // show viewer
    cv::imshow("Entity Live Viewer", output_img);

    // process key presses
    char key_pressed_ = -1;
    key_pressed_ = cv::waitKey(10);
    processKeyPressed(key_pressed_);
}

// ----------------------------------------------------------------------------------------------------


void EntityLiveViewerServer::addEntity(const ed::EntityConstPtr& entity){
    ed::ErrorContext errc("EntityLiveViewer -> addEntity()");

    if (viewer_freeze_) return;

    // Get the last measurement from the entity
    ed::MeasurementConstPtr msr = entity->lastMeasurement();

    if (!msr) return;

    cv::Rect rgbd_roi;
    EntityInfo e_info (entity);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // get depth image
    //    const cv::Mat& depth_image = msr->image()->getDepthImage();

    // mask color image with the entity segmentation
    cv::Mat entity_masked = ed::perception::maskImage(color_image, msr->imageMask(), rgbd_roi);

    //copy the masked image to EntityInfo
    entity_masked(rgbd_roi).copyTo(e_info.masked_roi);

    // update the current entity info or create a new one
    bool exists = false;
    for(std::vector<EntityInfo>::iterator entity_it = entity_list_.begin(); entity_it != entity_list_.end(); ++entity_it){
        if (entity_it->id.compare(e_info.id) == 0){
            entity_it->last_updated = 0;
            e_info.color_img.copyTo(entity_it->color_img);
            e_info.depth_img.copyTo(entity_it->depth_img);
            e_info.masked_roi.copyTo(entity_it->masked_roi);
            entity_it->roi = rgbd_roi;
            entity_it->data = e_info.data;
            exists = true;
        }
    }

    if (!exists){
        entity_list_.push_back(e_info);
    }
}


// ----------------------------------------------------------------------------------------------------


void EntityLiveViewerServer::processKeyPressed(char key){
    ed::ErrorContext errc("EntityLiveViewer -> processKeyPressed()");

    // return if no key was pressed
    if(key == -1) return;

    std::cout << module_name_  << "Key pressed = " << key << std::endl;

    switch (key){
        // choose entity from the list
        case '1':if (entity_list_.size() >= 1) focused_idx_ = 0;
        break;
        case '2':if (entity_list_.size() >= 2) focused_idx_ = 1;
        break;
        case '3':if (entity_list_.size() >= 3) focused_idx_ = 2;
        break;
        case '4':if (entity_list_.size() >= 4) focused_idx_ = 3;
        break;
        case '5':if (entity_list_.size() >= 5) focused_idx_ = 4;
        break;
        case '6':if (entity_list_.size() >= 6) focused_idx_ = 5;
        break;
        case '7':if (entity_list_.size() >= 7) focused_idx_ = 6;
        break;
        case '8':if (entity_list_.size() >= 8) focused_idx_ = 7;
        break;
        case '9':if (entity_list_.size() >= 9) focused_idx_ = 8;
        break;

        // next and previous entity
        case '+':focused_idx_++;
        break;
        case '-':focused_idx_--;
        break;

        // toggle freeze viewer
        case ' ':   viewer_freeze_ = !viewer_freeze_;
                    std::cout << module_name_ << "Viewer paused: " << viewer_freeze_ << std::endl;
        break;

        // store measurement
        case 's':
            if (focused_idx_ <= entity_list_.size()){
                storeMeasurement(entity_list_[focused_idx_].entity_pt, model_name_);
            }else
                std::cout << module_name_ << "Select an entity from the list" << key << std::endl;
        break;

        // change model name
        case 'n': std::cout << module_name_ << "Model name: ";
                  std::cin >> model_name_;
                  saved_measurements_ = 0;
        break;

    }
}


// ----------------------------------------------------------------------------------------------------


void EntityLiveViewerServer::putTextMultipleLines(std::string text, std::string delimiter, cv::Point origin, cv::Mat& image_out){
    ed::ErrorContext errc("EntityLiveViewer -> putTextMultipleLines()");

    int pos = 0;
    std::string token;
    cv::Point line_spacing(0, 15);
    int counter = 1;

    // split the string by the delimiter
    while ((pos = text.find(delimiter)) != std::string::npos) {
        token = text.substr(0, pos);
        text.erase(0, pos + delimiter.length());

        cv::putText(image_out, token, origin + line_spacing*counter, font_face_, font_scale_-0.1, cv::Scalar(255,255, 255), 1, CV_AA);
//        std::cout << token << std::endl;
        counter++;
    }

}


// ----------------------------------------------------------------------------------------------------

void EntityLiveViewerServer::storeMeasurement(const ed::EntityConstPtr& entity, const std::string& model_name){
    ed::ErrorContext errc("EntityLiveViewer -> storeMeasurement()");

    if (entity){
        char const* home = getenv("HOME");
        if (home)
        {
            boost::filesystem::path dir(std::string(home) + "/.ed/measurements/" + model_name);
            boost::filesystem::create_directories(dir);

            std::string filename = dir.string() + "/" + ed::Entity::generateID().str();
            ed::write(filename, *entity);

            std::cout << module_name_ << "Writing entity info to '" << filename << "'." << std::endl;
            saved_measurements_++;
        }
    } else
        std::cout << module_name_ << "Entity does not exist!" << std::endl;
}
