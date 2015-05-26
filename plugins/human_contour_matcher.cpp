/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: July 2015
*/

#include "human_contour_matcher.h"

#include "ed/measurement.h"
#include <ed/entity.h>
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include "shared_methods.h"

#include <boost/filesystem.hpp>

// ----------------------------------------------------------------------------------------------------

HumanContourMatcher::HumanContourMatcher() :
    ed::perception::Module("human_contour_matcher"),
    human_classifier_("human_contour_matcher"), init_success_(false)
{
}

// ----------------------------------------------------------------------------------------------------

HumanContourMatcher::~HumanContourMatcher()
{
}

// ----------------------------------------------------------------------------------------------------

void HumanContourMatcher::configure(tue::Configuration config) {

    if (!config.value("head_template_front_path", template_front_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'head_template_front_path' not found. Using default: " << template_front_path_ << std::endl;

    if (!config.value("head_template_left_path", template_left_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'head_template_left_path' not found. Using default: " << template_left_path_ << std::endl;

    if (!config.value("head_template_right_path", template_right_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'head_template_right_path' not found. Using default: " << template_right_path_ << std::endl;

    if (!config.value("debug_folder", debug_folder_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_folder' not found. Using default: " << debug_folder_ << std::endl;

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    if (!config.value("max_match_iterations", match_iterations_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'max_match_iterations' not found. Using default: " << match_iterations_ << std::endl;

    if (!config.value("dt_line_width", dt_line_width_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'dt_line_width' not found. Using default: " << dt_line_width_ << std::endl;

    if (!config.value("max_template_err", max_template_err_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'max_template_err' not found. Using default: " << max_template_err_ << std::endl;

    if (!config.value("dt_border_size", border_size_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'dt_border_size' not found. Using default: " << border_size_ << std::endl;

    if (!config.value("dt_slices_num", num_slices_matching_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'dt_slices_num' not found. Using default: " << num_slices_matching_ << std::endl;

    if (!config.value("dt_slices_num", num_slices_matching_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'dt_slices_num' not found. Using default: " << num_slices_matching_ << std::endl;

    if (!config.value("type_positive_score", type_positive_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_positive_score' not found. Using default: " << type_positive_score_ << std::endl;

    if (!config.value("type_negative_score", type_negative_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_negative_score' not found. Using default: " << type_negative_score_ << std::endl;

    if (!config.value("type_unknown_score", type_unknown_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_unknown_score' not found. Using default: " << type_unknown_score_ << std::endl;

    template_front_path_ = module_path_ + template_front_path_;
    template_left_path_ = module_path_ + template_left_path_;
    template_right_path_ = module_path_ + template_right_path_;

    // initialzie human_classifier
    if(!human_classifier_.Initializations(debug_mode_,
                                          debug_folder_,
                                          template_front_path_,
                                          template_left_path_,
                                          template_right_path_,
                                          match_iterations_,
                                          dt_line_width_,
                                          max_template_err_,
                                          border_size_,
                                          num_slices_matching_)){
        std::cout << "[" << module_name_ << "] " << "Initialization incomplete!" << std::endl;
    }

    init_success_ = true;

    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
}


// ----------------------------------------------------------------------------------------------------


void HumanContourMatcher::loadConfig(const std::string& config_path) {

    module_name_ = "human_contour_matcher";
    module_path_ = config_path;

    // default values in case configure(...) is not called!
    debug_mode_ = false;
    debug_folder_ = "/tmp/human_classifier/";
    template_front_path_ = "/head_templates/template_front_4_trimmed.png";
    template_left_path_ = "/head_templates/left_close.png";
    template_right_path_ = "/head_templates/right_close.png";
    match_iterations_ = 30;
    dt_line_width_ = 1;
    max_template_err_ = 15;
    border_size_ = 20;
    num_slices_matching_ = 7;
    type_positive_score_ = 0.9;
    type_negative_score_ = 0.4;
    type_unknown_score_ = 0.05;
}


// ----------------------------------------------------------------------------------------------------


void HumanContourMatcher::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const
{
    ed::ErrorContext errc("Processing entity in HumanContourMatcher");

    // if initialization failed, return
    if (!init_success_)
        return;

    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& result = output.data;

    float classification_error = 0;
    float classification_deviation = 0;
    std::string classification_stance;
    bool is_human = false;

    // ---------- Prepare measurement ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    int min_x, max_x, min_y, max_y;

    // create a view
//    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // get color image
    const cv::Mat& depth_image = msr->image()->getDepthImage();

    // crop it to match the view
//    cv::Mat cropped_image(color_image(cv::Rect(0,0,view.getWidth(), view.getHeight())));

    // initialize bounding box points
    max_x = 0;
    max_y = 0;
    min_x = depth_image.cols;
    min_y = depth_image.rows;

    cv::Mat depth_mask = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_8UC1);
    // Iterate over all points in the mask
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(depth_image.cols); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // paint a mask
        depth_mask.at<unsigned char>(*it) = 255;

        // update the boundary coordinates
        if (min_x > p_2d.x) min_x = p_2d.x;
        if (max_x < p_2d.x) max_x = p_2d.x;
        if (min_y > p_2d.y) min_y = p_2d.y;
        if (max_y < p_2d.y) max_y = p_2d.y;
    }

    cv::Rect bouding_box (min_x, min_y, max_x - min_x, max_y - min_y);

    // create a copy of the depth image region of interest, masked
    cv::Mat masked_depth_image(depth_image);
    masked_depth_image.copyTo(masked_depth_image, depth_mask);
    masked_depth_image = masked_depth_image(bouding_box);

    // get entity average depth
    float avg_depth = ed::perception::getAverageDepth(masked_depth_image);

    // call classifier
    is_human = human_classifier_.Classify(depth_image, color_image, depth_mask, avg_depth, classification_error, classification_deviation, classification_stance);


    // ----------------------- Assert results -----------------------

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("human_contour_matcher");

    result.setValue("label", "human_shape");

    // only assert something if error is above 0, otherwise matching could not be performed
    if (classification_error > 0){
        result.setValue("stance", classification_stance);
        result.setValue("error", classification_error);
        result.setValue("deviation", classification_deviation);
    }

    if(is_human){
        // classified as human
        result.setValue("score", type_positive_score_);

//        output.type_update.setScore("crowd", type_positive_score_);
        output.type_update.setScore("human", type_positive_score_);

    }else if (!is_human && classification_error > 0){
        // not classified as human but matching was possible
        result.setValue("score", type_negative_score_);

//        output.type_update.setScore("crowd", type_negative_score_);
        output.type_update.setScore("human", type_negative_score_);

    }else if (!is_human && classification_error == 0){
        // not classified as human and matching was not possible
        result.setValue("score", 0);

//        output.type_update.setScore("crowd", type_negative_score_);
        output.type_update.setScore("human", type_negative_score_);
    }


    output.type_update.setUnknownScore(type_unknown_score_);

    result.endGroup();  // close human_contour_matcher group
    result.endGroup();  // close perception_result group
}

ED_REGISTER_PERCEPTION_MODULE(HumanContourMatcher)
