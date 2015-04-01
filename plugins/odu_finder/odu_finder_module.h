/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: July 2015
*/

#ifndef ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_
#define ED_PERCEPTION_HUMAN_CONTOUR_MATCHER_H_

#include <ed/perception/module.h>
#include <boost/thread.hpp>

#include <opencv2/core/core.hpp>

namespace odu_finder
{
    class ODUFinder;
}

class ODUFinderModule : public ed::perception::Module
{

public:

    ODUFinderModule();

    virtual ~ODUFinderModule();

    void loadConfig(const std::string& config_path);

    void configure(tue::Configuration config);

    void process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const;

private:
    bool init_success_;
    bool debug_mode_;

    std::string database_path_;
    std::string module_path_;
    std::string module_name_;

    void optimizeContourHull(const cv::Mat& mask_orig, cv::Mat& mask_optimized, cv::Rect &bounding_box) const;

    odu_finder::ODUFinder* odu_finder_;

protected:

    mutable boost::mutex mutex_update_;
};

#endif
