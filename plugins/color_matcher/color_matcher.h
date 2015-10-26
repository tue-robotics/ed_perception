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

typedef std::vector<float> ColorHistogram;

class ColorMatcher : public ed::perception::Module
{

public:

    ColorMatcher();

    virtual ~ColorMatcher();

    void configure(tue::Configuration config);

    void loadModel(const std::string& model_name, const std::string& model_path) {}

    void loadConfig(const std::string& config_path) {}

    void process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const {}


    // New interface

    void classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
                  ed::perception::ClassificationOutput& output) const;

    void addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value);

    // Does nothing: training algorithm is online and doesn't need batch training
    void train() {}

    void loadRecognitionData(const std::string& path);

    void saveRecognitionData(const std::string& path) const;

private:

    ColorNameTable color_table_;

    std::map<std::string, ColorHistogram> models_;

    void calculateHistogram(const ed::Entity& e, ColorHistogram& histogram) const;




    // module configuration
    bool init_success_;
    bool debug_mode_;
    std::string module_name_;

    std::string debug_folder_;
    std::string module_path_;
    std::string color_table_path_;

    double type_unknown_score_;
};


#endif

