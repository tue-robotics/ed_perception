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

ODUFinderModule::ODUFinderModule() : ed::perception::Module("odu_finder"), initialized_(false), odu_finder_(NULL)
{
    this->registerPropertyServed("type");
}

// ----------------------------------------------------------------------------------------------------

ODUFinderModule::~ODUFinderModule()
{
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
              ed::perception::ClassificationOutput& output) const
{
    ed::ErrorContext errc("Processing entity in ODUFinderModule");

    if (!odu_finder_)
        return;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Prepare image

    cv::Mat cropped_mono_image;
    if (!extractImage(e, cropped_mono_image))
        return;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Process image

    IplImage img(cropped_mono_image);
    std::map<std::string, float> results;

    {
        boost::lock_guard<boost::mutex> lg(mutex_update_);
        results = odu_finder_->process_image(&img);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set results

    tue::Configuration& result = output.data;

    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("odu_finder");

//    output.type_update.setUnknownScore(type_unknown_score_);

    double total_score = 0;

    // if an hypothesis is found, assert it
    if (!results.empty())
    {
        for(std::map<std::string, float>::const_iterator it = results.begin(); it != results.end(); ++it)
        {
            double score = it->second * score_factor_;
            output.likelihood.setScore(it->first, score);
            total_score += score;
        }
    }

    if (total_score < 1.0)
        output.likelihood.setUnknownScore(1.0 - total_score);
    else
        output.likelihood.setUnknownScore(0);

    result.endGroup();  // close odu_finder group
    result.endGroup();  // close perception_result group

    if (debug_mode_){
        cv::imwrite(debug_folder_ + ed::Entity::generateID().str() + "_odu_finder_module.png", cropped_mono_image);
    }
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value)
{
    if (!odu_finder_)
        return;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Prepare image

    cv::Mat cropped_mono_image;
    if (!extractImage(e, cropped_mono_image))
        return;

    vt::Document doc;

    odu_finder_->add_image_to_database();

    // TODO
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::loadRecognitionData(const std::string& path)
{
    // creat odu finder instance
    delete odu_finder_;
    odu_finder_ = new odu_finder::ODUFinder(path, debug_mode_);
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::saveRecognitionData(const std::string& path) const
{
    if (odu_finder_)
        odu_finder_->save_database(path);
}

// ----------------------------------------------------------------------------------------------------

bool ODUFinderModule::extractImage(const ed::Entity& e, cv::Mat& img) const
{
    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e.lastMeasurement();
    if (!msr)
        return false;

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // Created masked image
    cv::Rect rgb_roi;
    ed::perception::maskImage(color_image, msr->imageMask(), rgb_roi);

    // Create cropped masked color image
    cv::Mat cropped_image = color_image(rgb_roi);

    // convert to grayscale and increase contrast
    cv::cvtColor(cropped_image, img, CV_BGR2GRAY);
    cv::equalizeHist(croped_mono_image, img);
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::configure(tue::Configuration config) {

    if (!config.value("database_path", database_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'database_path' not found. Using default: " << database_path_ << std::endl;

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    if (!config.value("score_factor", score_factor_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'score_factor' not found. Using default: " << score_factor_ << std::endl;

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

    initialized_ = true;

    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::loadConfig(const std::string& config_path)
{
    module_name_ = "odu_finder";
    module_path_ = config_path;
    database_path_ = "/database";

    // default values in case configure(...) is not called!
    score_factor_ = 0.1;
    debug_mode_ = false;
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const
{
    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& result = output.data;

    ed::ErrorContext errc("Processing entity in ODUFinderModule");

//    output.type_update.setUnknownScore(type_unknown_score_);

    if (!initialized_)
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

//    output.type_update.setUnknownScore(type_unknown_score_);

    double total_score = 0;

    // if an hypothesis is found, assert it
    if (!results.empty())
    {
        for(std::map<std::string, float>::const_iterator it = results.begin(); it != results.end(); ++it)
        {
            double score = it->second * score_factor_;
            output.type_update.setScore(it->first, score);
            total_score += score;
        }
    }

    if (total_score < 1.0)
        output.type_update.setUnknownScore(1.0 - total_score);
    else
        output.type_update.setUnknownScore(0);

    result.endGroup();  // close odu_finder group
    result.endGroup();  // close perception_result group

    if (debug_mode_){
        cv::imwrite(debug_folder_ + ed::Entity::generateID().str() + "_odu_finder_module.png", croped_mono_image);
    }
}

ED_REGISTER_PERCEPTION_MODULE(ODUFinderModule)
