#ifndef _AGGREGATOR_H_
#define _AGGREGATOR_H_

#include "ed/perception/module.h"

namespace ed
{
namespace perception
{

class Aggregator
{

public:

    Aggregator();

    ~Aggregator();

    void configure(tue::Configuration config, bool for_training);

    void classify(const Entity& e, const std::string& property, const CategoricalDistribution& prior, ClassificationOutput& output) const;

    void addTrainingInstance(const Entity& e, const std::string& property, const std::string& value);

    void train();

    void loadRecognitionData();

    void saveRecognitionData() const;

    void addModule(const boost::shared_ptr<Module>& m);

private:

    std::vector<std::string> plugin_paths_;

    std::vector<boost::shared_ptr<Module> > modules_;

    std::vector<class_loader::ClassLoader*> perception_loaders_;

    std::string classification_model_path_;

};

} // end namespace ed

} // end namespace perception

#endif
