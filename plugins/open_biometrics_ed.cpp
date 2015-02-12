/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "open_biometrics_ed.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <boost/filesystem.hpp>

// #include <openbr/openbr_plugin.h>

// ----------------------------------------------------------------------------------------------------

OpenBrEd::OpenBrEd() :
    PerceptionModule("face_detector"),
    init_success_(false)
{
}


// ----------------------------------------------------------------------------------------------------

OpenBrEd::~OpenBrEd()
{
}


// ----------------------------------------------------------------------------------------------------

void OpenBrEd::configure(tue::Configuration config) {

    if (debug_mode_){
        // clean the debug folder if debugging is active
        try {
            boost::filesystem::path dir(debug_folder_);
            boost::filesystem::remove_all(dir);
            boost::filesystem::create_directories(dir);
        } catch(const boost::filesystem::filesystem_error& e){
           if(e.code() == boost::system::errc::permission_denied)
               std::cout << "[" << module_name_ << "] " << "boost::filesystem permission denied" << std::endl;
           else
               std::cout << "[" << module_name_ << "] " << "boost::filesystem failed with error: " << e.code().message() << std::endl;
        }

        // create debug window
        cv::namedWindow("Open Biometrics ED Output", CV_WINDOW_AUTOSIZE);
    }
}


// ----------------------------------------------------------------------------------------------------


void OpenBrEd::loadConfig(const std::string& config_path) {

    module_name_ = "open_br_ed";
    module_path_ = config_path;
    debug_folder_ = "/tmp/face_detector/";
    debug_mode_ = false;
}


ED_REGISTER_PERCEPTION_MODULE(OpenBrEd)
