/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#include "size_matcher.h"

#include "ed/measurement.h"
#include "ed/entity.h"
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <tue/config/write.h>
#include <tue/config/writer.h>
#include <tue/config/read.h>
#include <tue/config/reader.h>

#include <tue/filesystem/path.h>

// ----------------------------------------------------------------------------------------------------

namespace
{

bool calculateSize(const ed::Entity& e, double& width, double& height)
{
    ed::MeasurementConstPtr msr = e.lastMeasurement();
    if (!msr)
        return false;

    // get depth image
    const cv::Mat& depth = msr->image()->getDepthImage();

    geo::Vec2 p_min( 1e6,  1e6);
    geo::Vec2 p_max(-1e6, -1e6);

    bool has_points = false;

    rgbd::View view(*msr->image(), depth.cols);
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(depth.cols); it != msr->imageMask().end(); ++it)
    {
        const cv::Point2i& p_2d = *it;
        float d = depth.at<float>(p_2d);

        if (d == 0 || d != d)
            continue;

        geo::Vec3 p = view.getRasterizer().project2Dto3D(p_2d.x, p_2d.y) * d;

        p_min.x = std::min(p_min.x, p.x);
        p_min.y = std::min(p_min.y, p.y);
        p_max.x = std::max(p_max.x, p.x);
        p_max.y = std::max(p_max.y, p.y);

        has_points = true;
    }

    if (!has_points)
        return false;

    width = p_max.x - p_min.x;
    height = p_max.y - p_min.y;

    return true;
}

}

// ----------------------------------------------------------------------------------------------------

SizeMatcher::SizeMatcher() : Module("size_matcher")
{
    this->registerPropertyServed("type");
}

// ----------------------------------------------------------------------------------------------------

SizeMatcher::~SizeMatcher(){}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
              ed::perception::ClassificationOutput& output) const
{
    if (property != "type")
        return;

    double width, height;
    if (!calculateSize(e, width, height))
        return;

    double margin = 0.01;

    for(std::map<std::string, SizeModel>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& model_name = it->first;
        const SizeModel& model = it->second;

        if (model.width_min - margin < width && width < model.width_max + margin
                && model.height_min - margin < height && height < model.height_max + margin)
        {
            double score = 1.0 / (model.width_max - model.width_min) + 1.0 / (model.height_max - model.height_min);
//            double score = 1;
            output.likelihood.setScore(model_name, score);
        }
        else
        {
            output.likelihood.setScore(model_name, 0);
        }
    }

    // Represent data (for debugging)
    tue::Configuration& result = output.data;
    result.setValue("width", width);
    result.setValue("height", height);
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value)
{
    if (property != "type")
        return;

    double width, height;
    if (!calculateSize(e, width, height))
        return;

    std::map<std::string, SizeModel>::iterator it = models_.find(value);
    if (it == models_.end())
    {
        // No other training instances for this type
        SizeModel& model = models_[value];
        model.width_min = width;
        model.width_max = width;
        model.height_min = height;
        model.height_max = height;
    }
    else
    {
        SizeModel& model = it->second;
        model.width_min = std::min(model.width_min, width);
        model.width_max = std::max(model.width_max, width);
        model.height_min = std::min(model.height_min, height);
        model.height_max = std::max(model.height_max, height);
    }
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::loadRecognitionData(const std::string& path_str)
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

        SizeModel& model = models_[model_name];

        r.value("width_min", model.width_min);
        r.value("width_max", model.width_max);
        r.value("height_min", model.height_min);
        r.value("height_max", model.height_max);
    }

    r.endArray(); // models
}

// ----------------------------------------------------------------------------------------------------

void SizeMatcher::saveRecognitionData(const std::string& path) const
{
    tue::config::Writer w;

    w.writeArray("models");
    for(std::map<std::string, SizeModel>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const std::string& model_name = it->first;
        const SizeModel& model = it->second;

        w.addArrayItem();
        w.setValue("name", model_name);
        w.setValue("width_min", model.width_min);
        w.setValue("width_max", model.width_max);
        w.setValue("height_min", model.height_min);
        w.setValue("height_max", model.height_max);
        w.endArrayItem();
    }
    w.endArray();

    tue::config::toFile(path + "/models.yaml", w.data(), tue::config::YAML, 4);
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PERCEPTION_MODULE(SizeMatcher)

