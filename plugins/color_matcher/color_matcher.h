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

    void configureTraining(tue::Configuration config);

    void configureClassification(tue::Configuration config);

    void classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
                  ed::perception::ClassificationOutput& output) const;

    void addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value);

    void loadRecognitionData(const std::string& path);

    void saveRecognitionData(const std::string& path) const;

private:

    ColorNameTable color_table_;

    std::map<std::string, ColorModel> models_;

    void calculateHistogram(const ed::Entity& e, ColorHistogram& histogram) const;


    // Classification parameters

    double color_margin_;

    void initialize();


};


#endif

