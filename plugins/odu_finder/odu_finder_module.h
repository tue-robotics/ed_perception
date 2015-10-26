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


    // New interface

    ed::perception::CategoricalDistribution classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior) const {}

    void addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value) {}

    void train() {}

    void loadRecognitionData(const std::string& path) {}

    void saveRecognitionData(const std::string& path) const {}

private:
    bool init_success_;
    bool debug_mode_;
    std::string debug_folder_;

    double score_factor_;

    std::string database_path_;
    std::string module_path_;
    std::string module_name_;

    odu_finder::ODUFinder* odu_finder_;

protected:

    mutable boost::mutex mutex_update_;
};

#endif
