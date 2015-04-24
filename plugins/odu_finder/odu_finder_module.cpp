/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "odu_finder_module.h"
#include "odu_finder.h"

#include "ed/measurement.h"
#include <ed/entity.h>
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include "../shared_methods.h"


// ----------------------------------------------------------------------------------------------------

ODUFinderModule::ODUFinderModule() :
    ed::perception::Module("odu_finder"),
    init_success_(false)
{
}

// ----------------------------------------------------------------------------------------------------

ODUFinderModule::~ODUFinderModule()
{
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::configure(tue::Configuration config) {

    if (!config.value("database_path", database_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'database_path' not found. Using default: " << database_path_ << std::endl;

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    if (!config.value("type_unknown_score", type_unknown_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_unknown_score' not found. Using default: " << type_unknown_score_ << std::endl;

    if (!config.value("debug_folder", debug_folder_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_folder' not found. Using default: " << debug_folder_ << std::endl;

    database_path_ = module_path_ + database_path_;

    if (debug_mode_){
        std::cout << "[" << module_name_ << "] " << "Debug mode enabled. Debug folder: " << debug_folder_ << std::endl;
        ed::perception::cleanDebugFolder(debug_folder_);
    }

    // creat odu finder instance
    odu_finder_ = new odu_finder::ODUFinder(database_path_, debug_mode_);

    if (odu_finder_->get_n_models_loaded() == 0)
         std::cout << "[" << module_name_ << "] " << "No models were loaded!" << std::endl;
    else
        std::cout << "[" << module_name_ << "] " << "Loaded information for " << odu_finder_->get_n_models_loaded() << " models" << std::endl;

    init_success_ = true;

    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::loadConfig(const std::string& config_path)
{
    module_name_ = "odu_finder";
    module_path_ = config_path;
    database_path_ = "/database";

    // default values in case configure(...) is not called!
    type_unknown_score_ = 0.05;
    debug_mode_ = false;
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const
{
    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& result = output.data;

    ed::ErrorContext errc("Processing entity in ODUFinderModule");

    output.type_update.setUnknownScore(type_unknown_score_);

    if (!init_success_)
        return;

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    // ----------------------- PREPARE IMAGE -----------------------

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // Created masked image
    cv::Rect rgb_roi;
    cv::Mat masked_color_image = ed::perception::maskImage(color_image, msr->imageMask(), rgb_roi);

    // Create cropped masked color image
    cv::Mat cropped_image = color_image(rgb_roi);

    // convert to grayscale and increase contrast
    cv::Mat croped_mono_image;
    cv::cvtColor(cropped_image, croped_mono_image, CV_BGR2GRAY);
    cv::equalizeHist(croped_mono_image, croped_mono_image);

    // ----------------------- PROCESS IMAGE -----------------------

    IplImage img(croped_mono_image);
    std::map<std::string, float> results;

    {
        boost::lock_guard<boost::mutex> lg(mutex_update_);
        results = odu_finder_->process_image(&img);
    }

    // ----------------------- ASSERT RESULTS -----------------------


    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("odu_finder");

    output.type_update.setUnknownScore(type_unknown_score_);

    // if an hypothesis is found, assert it
    if (!results.empty())
    {
        for(std::map<std::string, float>::const_iterator it = results.begin(); it != results.end(); ++it){
            output.type_update.setScore(it->first, it->second);
        }
    }

    result.endGroup();  // close odu_finder group
    result.endGroup();  // close perception_result group

    if (debug_mode_){
        cv::imwrite(debug_folder_ + ed::Entity::generateID().str() + "_odu_finder_module.png", croped_mono_image);
    }
}

ED_REGISTER_PERCEPTION_MODULE(ODUFinderModule)
