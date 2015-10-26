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

// ---------------------------------------------------------------------------------------------------

ColorMatcher::ColorMatcher() :
    ed::perception::Module("color_matcher"),
    init_success_(false)
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
    if (!init_success_)
        return;

    if (property != "type")
        return;

    tue::Configuration& result = output.data;

    ColorHistogram color_histogram;
    calculateHistogram(e, color_histogram);

    float color_threshold = 0.1;

    for(std::map<std::string, ColorHistogram>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& model_name = it->first;
        const ColorHistogram& model_color_histogram = it->second;

        output.likelihood.setScore(model_name, 1);
        for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
        {
            if (model_color_histogram[i] < color_threshold && color_histogram[i] > color_threshold)
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

    std::map<std::string, ColorHistogram>::iterator it = models_.find(value);
    if (it == models_.end())
    {
        // No other training instances for this type
        models_[value] = color_histogram;
    }
    else
    {
        ColorHistogram& model_color_histogram = it->second;

        // Determine maximum for each histogram bin (i.e., determine maximum for each color)
        for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
            model_color_histogram[i] = std::max(model_color_histogram[i], color_histogram[i]);
    }
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::loadRecognitionData(const std::string& path)
{
    tue::config::DataPointer data = tue::config::fromFile(path);

    tue::config::Reader r(data);

    if (!r.readArray("models"))
        return;

    while(r.nextArrayItem())
    {
        std::string model_name;
        if (!r.value("name", model_name))
            continue;

        ColorHistogram& model_color_histogram = models_[model_name];
        model_color_histogram.resize(ColorNameTable::NUM_COLORS, 0);

        if (r.readArray("colors"))
        {
            int i = 0;
            while(r.nextArrayItem())
            {
                float value;
                if (r.value("value", value))
                    model_color_histogram[i] = value;
                ++i;
            }

            r.endArray();
        }
    }

    r.endArray(); // models
}

// ----------------------------------------------------------------------------------------------------

void ColorMatcher::saveRecognitionData(const std::string& path) const
{
    tue::config::Writer w;

    w.writeArray("models");
    for(std::map<std::string, ColorHistogram>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& model_name = it->first;
        const ColorHistogram& model_color_histogram = it->second;

        w.addArrayItem();
        w.setValue("name", model_name);

        w.writeArray("colors");
        for(unsigned int i = 0; i < ColorNameTable::NUM_COLORS; ++i)
        {
            w.addArrayItem();
            w.setValue("value", model_color_histogram[i]);
            w.endArrayItem();
        }
        w.endArray();

        w.endArrayItem();
    }
    w.endArray();

    tue::config::toFile(path, w.data(), tue::config::YAML, 4);
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

    if (!config.value("color_table", color_table_path_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'color_table' not found. Using default: " << color_table_path_ << std::endl;

    color_table_path_ = module_path_ + "/" + color_table_path_;

    if (!config.value("debug_mode", debug_mode_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_mode' not found. Using default: " << debug_mode_ << std::endl;

    if (!config.value("debug_folder", debug_folder_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'debug_folder' not found. Using default: " << debug_folder_ << std::endl;

    if (!config.value("type_unknown_score", type_unknown_score_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'type_unknown_score' not found. Using default: " << type_unknown_score_ << std::endl;

//    if (debug_mode_)
//        ed::perception::cleanDebugFolder(debug_folder_);

    std::cout << "[" << module_name_ << "] " << "Loading color names..." << std::endl;

    if (!color_table_.readFromFile(color_table_path_)){
        std::cout << "[" << module_name_ << "] " << "Failed loading color names from " << color_table_path_ << std::endl;
        return;
    }

    init_success_ = true;

    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PERCEPTION_MODULE(ColorMatcher)
