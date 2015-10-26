#ifndef ED_PERCEPTION_MODULE_H_
#define ED_PERCEPTION_MODULE_H_

#include <class_loader/class_loader.h>
#define ED_REGISTER_PERCEPTION_MODULE(Derived)  CLASS_LOADER_REGISTER_CLASS(Derived, ed::perception::Module)

#include "ed/perception/worker_input.h"
#include "ed/perception/worker_output.h"

#include <set>

namespace ed
{
namespace perception
{

struct ClassificationOutput
{
    ClassificationOutput(tue::Configuration& data_) : data(data_) {}

    CategoricalDistribution likelihood;
    tue::Configuration& data;
};

class Module
{

public:

    Module(const std::string& name) : name_(name) {}

    virtual ~Module() {}

    virtual void process(const WorkerInput& input, WorkerOutput& output) const = 0;

    virtual void loadModel(const std::string& model_name, const std::string& model_path) {}

    virtual void loadConfig(const std::string& config_path) {}

    virtual void configure(tue::Configuration config) {}


    // New interface

    // Return the value distribution for the given entity and property
    virtual void classify(const Entity& e, const std::string& property, const CategoricalDistribution& prior, ClassificationOutput& output) const = 0;

    // Add a training instance for the given entity, property and property value
    virtual void addTrainingInstance(const Entity& e, const std::string& property, const std::string& value) = 0;

    // Optional: if the module uses batch training, call this function if all training instances have been added
    virtual void train() {}

    // Load the data needed to classify from disk, using the given path. The function train() should not have to be called after, i.e.,
    // after calling this method, the module should be ready to classify.
    virtual void loadRecognitionData(const std::string& path) = 0;

    // Save the data needed to classify to the given path
    virtual void saveRecognitionData(const std::string& path) const = 0;

    // Returns the name of this module
    const std::string& name() const { return name_; }

    // Returns the properties this module serves
    const std::set<std::string>& properties_served() const { return properties_served_; }

    // Returns true if this module serves the given property
    bool serves_property(const std::string& property)
    {
        return properties_served_.find(property) != properties_served_.end();
    }

protected:

    // For internal use of the module: register which properties this module serves
    void registerPropertyServed(const std::string& property)
    {
        properties_served_.insert(property);
    }


private:

    std::string name_;

    std::set<std::string> properties_served_;

};

boost::shared_ptr<Module> loadPerceptionModule(class_loader::ClassLoader* loader, const std::string& model_list_name = std::string());

bool loadModelList(std::string& model_list_path, std::vector<std::string>& model_list);

} // end namespace ed

} // end namespace perception

#endif
