#ifndef _AGGREGATOR_H_
#define _AGGREGATOR_H_

#include "ed/perception/module.h"

namespace ed
{
namespace perception
{

class Aggregator : public Module
{

public:

    Aggregator();

    ~Aggregator();

    void configure(tue::Configuration config);

    void classify(const Entity& e, const std::string& property, const CategoricalDistribution& prior, ClassificationOutput& output) const;

    void addTrainingInstance(const Entity& e, const std::string& property, const std::string& value);

    void train();

    void loadRecognitionData(const std::string& path);

    void saveRecognitionData(const std::string& path) const;

    void addModule(const boost::shared_ptr<Module>& m);


    void loadRecognitionData() { loadRecognitionData(classification_model_path_); }

    void saveRecognitionData() { saveRecognitionData(classification_model_path_); }


    void process(const WorkerInput& input, WorkerOutput& output) const {}

private:

    std::vector<std::string> plugin_paths_;

    std::vector<boost::shared_ptr<Module> > modules_;

    std::vector<class_loader::ClassLoader*> perception_loaders_;

    std::string classification_model_path_;

};

} // end namespace ed

} // end namespace perception

#endif
