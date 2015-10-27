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

struct ColorModel
{
    ColorHistogram min;
    ColorHistogram max;
};

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

    std::map<std::string, ColorModel> models_;

    void calculateHistogram(const ed::Entity& e, ColorHistogram& histogram) const;


    // Parameters

    double color_margin_;

};


#endif

