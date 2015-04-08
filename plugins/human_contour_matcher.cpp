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

    template_front_path_ = module_path_ + template_front_path_;

    if (!config.value("head_template_left_path", template_left_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'head_template_left_path' not found. Using default: " << template_left_path_ << std::endl;

    template_left_path_ = module_path_ + template_left_path_;

    if (!config.value("head_template_right_path", template_right_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'head_template_right_path' not found. Using default: " << template_right_path_ << std::endl;

    template_right_path_ = module_path_ + template_right_path_;

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

}


// ----------------------------------------------------------------------------------------------------


void HumanContourMatcher::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const
{
    ed::ErrorContext errc("Processing entity in HumanContourMatcher");

    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& result = output.data;

    // if initialization failed, return
    if (!init_success_)
        return;

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    float depth_sum = 0;
    float avg_depht;
    float classification_error = 0;
    float classification_deviation = 0;
    std::string classification_stance;
    uint point_counter = 0;
    bool is_human = false;

    // Get the depth image from the measurement
    const cv::Mat& depth_image = msr->image()->getDepthImage();
    const cv::Mat& color_image = msr->image()->getRGBImage();

    cv::Mat mask_cv = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_8UC1);

    // Iterate over all points in the mask. You must specify the width of the image on which you
    // want to apply the mask (see the begin(...) method).
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(depth_image.cols); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // TODO dont creat a Mat mask, just give human_classifier_.Classify a vector of 2D points!
        // paint a mask
        mask_cv.at<unsigned char>(p_2d) = 255;

        // calculate measurement average depth
        depth_sum += depth_image.at<float>(p_2d);
        point_counter++;
    }

    avg_depht = depth_sum/(float)point_counter;

    is_human = human_classifier_.Classify(depth_image, color_image, mask_cv, avg_depht, classification_error, classification_deviation, classification_stance);

    // ----------------------- assert results -----------------------

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    // assert the results to the entity
    result.writeGroup(module_name_);
    result.setValue("label", "human_shape");

    if (classification_error > 0){
        result.setValue("stance", classification_stance);
        result.setValue("error", classification_error);
        result.setValue("deviation", classification_deviation);
    }

    output.type_update.setUnknownScore(0.1); // TODO: magic number

    if(is_human)
    {
        result.setValue("score", 1.0);
        output.type_update.setScore("human", 0.8);
    }else{
        result.setValue("score", 0.0);
//        output.type_update.setScore("human", 0.2);
    }

    result.endGroup();  // close human_contour_matcher group
    result.endGroup();  // close perception_result group
}

ED_REGISTER_PERCEPTION_MODULE(HumanContourMatcher)
