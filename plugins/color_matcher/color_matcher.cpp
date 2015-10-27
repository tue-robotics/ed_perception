/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "color_matcher.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <tue/config/write.h>
#include <tue/config/writer.h>
#include <tue/config/read.h>
#include <tue/config/reader.h>

#include <tue/filesystem/path.h>

#include <ros/package.h>

// ---------------------------------------------------------------------------------------------------

ColorMatcher::ColorMatcher() : ed::perception::Module("color_matcher")
{
    this->registerPropertyServed("type");
}

// ---------------------------------------------------------------------------------------------------

ColorMatcher::~ColorMatcher()
{
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::classify(const ed::Entity& e, const std::string& property,
                            const ed::perception::CategoricalDistribution& prior,
                            ed::perception::ClassificationOutput& output) const
{
    if (property != "type")
        return;

    tue::Configuration& result = output.data;

    ColorHistogram color_histogram;
    calculateHistogram(e, color_histogram);

    for(std::map<std::string, ColorModel>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& model_name = it->first;
        const ColorModel& model = it->second;

        output.likelihood.setScore(model_name, 1);
        for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
        {
            float v = color_histogram[i];
            if (v < model.min[i] - color_margin_ || v > model.max[i] + color_margin_)
                output.likelihood.setScore(model_name, 0);
        }
    }

    // Represent data (for debugging)
    result.writeArray("colors");
    for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
    {
        result.addArrayItem();
        result.setValue("color", "color");
        result.setValue("value", color_histogram[i]);
        result.endArrayItem();
    }
    result.endArray();
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value)
{
    if (property != "type")
        return;

    ColorHistogram color_histogram;
    calculateHistogram(e, color_histogram);

    std::map<std::string, ColorModel>::iterator it = models_.find(value);
    if (it == models_.end())
    {
        // No other training instances for this type
        ColorModel& model = models_[value];
        model.min = color_histogram;
        model.max = color_histogram;
    }
    else
    {
        ColorModel& model = it->second;

        // Determine maximum for each histogram bin (i.e., determine maximum for each color)
        for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
        {
            model.min[i] = std::min(model.min[i], color_histogram[i]);
            model.max[i] = std::max(model.max[i], color_histogram[i]);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::loadRecognitionData(const std::string& path_str)
{
    tue::filesystem::Path path(path_str + "/models.yaml");
    if (!path.exists())
        return;

    tue::config::DataPointer data = tue::config::fromFile(path.string());

    tue::config::Reader r(data);

    if (!r.readArray("models"))
        return;

    while(r.nextArrayItem())
    {
        std::string model_name;
        if (!r.value("name", model_name))
            continue;

        ColorModel& model = models_[model_name];
        model.min.resize(ColorNameTable::NUM_COLORS, 0);
        model.max.resize(ColorNameTable::NUM_COLORS, 0);

        if (r.readArray("colors"))
        {
            int i = 0;
            while(r.nextArrayItem())
            {
                r.value("min", model.min[i]);
                r.value("max", model.max[i]);
                ++i;
            }

            r.endArray();
        }
    }

    color_margin_ = 0;
    r.value("color_margin", color_margin_);

    r.endArray(); // models
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::saveRecognitionData(const std::string& path) const
{
    tue::config::Writer w;

    w.writeArray("models");
    for(std::map<std::string, ColorModel>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& model_name = it->first;
        const ColorModel& model = it->second;

        w.addArrayItem();
        w.setValue("name", model_name);

        w.writeArray("colors");
        for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
        {
            w.addArrayItem();
            w.setValue("min", model.min[i]);
            w.setValue("max", model.max[i]);
            w.setValue("color", ColorNameTable::intToColorName(i));
            w.endArrayItem();
        }
        w.endArray();

        w.endArrayItem();
    }
    w.endArray();

    w.setValue("color_margin", color_margin_);

    tue::config::toFile(path + "/models.yaml", w.data(), tue::config::YAML, 4);
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::calculateHistogram(const ed::Entity& e, ColorHistogram& histogram) const
{
    ed::MeasurementConstPtr msr = e.lastMeasurement();
    if (!msr)
        return;

    histogram.resize(ColorNameTable::NUM_COLORS);
    histogram.assign(ColorNameTable::NUM_COLORS, 0);

    // get color image
    const cv::Mat& img = msr->image()->getRGBImage();

    int pixel_count = 0;

    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(img.cols); it != msr->imageMask().end(); ++it)
    {
        ++pixel_count;
        const cv::Point2i& p = *it;

        // Calculate prob distribution
        const cv::Vec3b& bgr = img.at<cv::Vec3b>(p);

        const float* probs = color_table_.rgbToDistribution(bgr[2], bgr[1], bgr[0]);

        for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
            histogram[i] += probs[i];
    }

    // normalize histogram
    for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
        histogram[i] /= pixel_count;
}

// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

void ColorMatcher::configure(tue::Configuration config)
{
    std::string color_table_path_ = ros::package::getPath("ed_perception") + "/data/color_names.txt";

    if (!color_table_.readFromFile(color_table_path_)){
        config.addError("Failed loading color names from '" + color_table_path_ + "'.");
        return;
    }

    config.value("color_margin", color_margin_);
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PERCEPTION_MODULE(ColorMatcher)
