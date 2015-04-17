/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "odu_finder_module.h"

#include "ed/measurement.h"
#include <ed/entity.h>
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include "odu_finder.h"

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

    database_path_ = module_path_ + database_path_;

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    // creat odu finder instance
    odu_finder_ = new odu_finder::ODUFinder(database_path_, debug_mode_);

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
    debug_mode_ = false;
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const
{
    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& result = output.data;

    ed::ErrorContext errc("Processing entity in ODUFinderModule");

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
    cv::Mat cropped_image = masked_color_image(rgb_roi);

    // convert to grayscale and increase contrast
    cv::Mat masked_mono_image;
    cv::cvtColor(cropped_image, masked_mono_image, CV_BGR2GRAY);
    cv::equalizeHist(masked_mono_image, masked_mono_image);

    // ----------------------- PROCESS IMAGE -----------------------

    IplImage img(masked_mono_image);
    std::map<std::string, float> results;

    {
        boost::lock_guard<boost::mutex> lg(mutex_update_);
        results = odu_finder_->process_image(&img);
    }


    // ----------------------- SAVE RESULTS -----------------------


    // create group if it doesnt exist
    if (!result.readGroup("perception_result", tue::OPTIONAL))
    {
        result.writeGroup("perception_result");
    }

    result.writeGroup("odu_finder");

    output.type_update.setUnknownScore(0.5);

    // if an hypothesis is found, assert it
    if (!results.empty())
    {
        result.writeArray("hypothesis");
        for(std::map<std::string, float>::const_iterator it = results.begin(); it != results.end(); ++it)
        {
            result.addArrayItem();
            result.setValue("name", it->first);
            result.setValue("score", it->second);
            result.endArrayItem();

            output.type_update.setScore(it->first, it->second);
        }
        result.endArray();
    }

    result.endGroup();  // close odu_finder group
    result.endGroup();  // close perception_result group
}

// ----------------------------------------------------------------------------------------------------

void ODUFinderModule::optimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized, cv::Rect& bounding_box) const{

    std::vector<std::vector<cv::Point> > hull;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Rect> bounding_boxes;
    cv::Mat tempMat;

    cv::findContours(mask_orig, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    tempMat = cv::Mat::zeros(mask_orig.size(), CV_8UC1);

    for (uint i = 0; i < contours.size(); i++){
        hull.push_back(std::vector<cv::Point>());
        cv::convexHull(cv::Mat(contours[i]), hull.back(), false);

        bounding_boxes.push_back(cv::boundingRect(hull.back()));

        cv::drawContours(tempMat, hull, -1, cv::Scalar(255), CV_FILLED);
    }

//    cv::imwrite("/tmp/odu/opt.png", mask_optimized);

    contours.clear();
    mask_optimized = cv::Mat(tempMat);

    cv::findContours(tempMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    if (contours.size()>0) bounding_box = cv::boundingRect(contours[0]);

//    cv::imwrite("/tmp/odu/bounding.png", mask_optimized(bounding_box));
}

ED_REGISTER_PERCEPTION_MODULE(ODUFinderModule)
