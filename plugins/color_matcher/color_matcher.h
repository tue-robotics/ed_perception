/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: July 2015
*/

#ifndef ED_PERCEPTION_COLOR_MATCHER_H_
#define ED_PERCEPTION_COLOR_MATCHER_H_

// OpenCV
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>

// Color name table
#include "color_name_table.h"

#include <ed/perception/module.h>

using namespace ColorNames;


class ColorMatcher : public ed::perception::Module
{

public:

    ColorMatcher();

    virtual ~ColorMatcher();

    void configure(tue::Configuration config);

    void loadModel(const std::string& model_name, const std::string& model_path);

    void loadConfig(const std::string& config_path);

    void process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const;

private:

    ColorNameTable& color_table_;

    // module configuration
    bool init_success_;
    bool debug_mode_;
    std::string module_name_;

    std::string debug_folder_;
    std::string module_path_;
    std::string color_table_path_;

    double type_unknown_score_;

    // Module methods
    std::map<std::string, double> getImageColorProbability(const cv::Mat& img, const ed::ImageMask& mask, cv::Mat &histogram) const;

    std::string getHighestProbColor(std::map<std::string, double>& map) const;

    // Object colors
    std::map<std::string, std::vector<std::map<std::string, double> > > models_colors_;

    void cleanDebugFolder(const std::string& folder) const;

    bool loadLearning(std::string path, std::string model_name);

    void getHypothesis(const std::map<std::string, double>& curr_hist, std::map<std::string, double>& hypothesis) const;
};


#endif

