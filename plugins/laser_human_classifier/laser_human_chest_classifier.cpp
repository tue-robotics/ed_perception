/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: March 2015
*/

#include "laser_human_chest_classifier.h"

#include "ed/measurement.h"
#include <ed/entity.h>
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <boost/filesystem.hpp>

#include <iostream>


// ----------------------------------------------------------------------------------------------------

LaserHumanChestClassifier::LaserHumanChestClassifier() :
    PerceptionModule("laser_human_chest_classifier"),
    init_success_(false)
{

}


// ----------------------------------------------------------------------------------------------------


LaserHumanChestClassifier::~LaserHumanChestClassifier()
{
}


// ----------------------------------------------------------------------------------------------------

void LaserHumanChestClassifier::configure(tue::Configuration config) {

    bool file_loaded = true;

    if (!config.value("hypotheses_filename", hypotheses_filename_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'hypotheses_filename' not found. Using default: " << hypotheses_filename_ << std::endl;

    if (!config.value("num_hypotheses", num_hypotheses_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'num_hypotheses' not found. Using default: " << num_hypotheses_ << std::endl;

    hypotheses_filename_ = module_path_ + hypotheses_filename_;

    // try to load the hypothesis file
    try {
        f_hypotheses = fopen(hypotheses_filename_.c_str(), "r");
        if (f_hypotheses == NULL){
            throw -1;
        }
    } catch (int e) {
        std::cout << "[" << module_name_ << "] " << "Could not load hypothesis file! (" << hypotheses_filename_ << ")" << std::endl;
        file_loaded = false;
    }

    // initialize PeopleDetector
    if (file_loaded){
        pd.load(f_hypotheses, num_hypotheses_);
        init_success_ = true;
        std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
    }
}


// ----------------------------------------------------------------------------------------------------


void LaserHumanChestClassifier::loadConfig(const std::string& config_path) {

    module_name_ = "laser_human_detector";
    module_path_ = config_path;

}


// ----------------------------------------------------------------------------------------------------


void LaserHumanChestClassifier::process(ed::EntityConstPtr e, tue::Configuration& result) const
{
    ed::ErrorContext errc("Processing entity in LaserHumanDetector");

    if (!init_success_)
        return;

}


// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PERCEPTION_MODULE(LaserHumanChestClassifier)
