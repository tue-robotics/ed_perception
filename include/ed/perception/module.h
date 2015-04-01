#ifndef ED_PERCEPTION_MODULE_H_
#define ED_PERCEPTION_MODULE_H_

#include <class_loader/class_loader.h>
#define ED_REGISTER_PERCEPTION_WORKER(Derived)  CLASS_LOADER_REGISTER_CLASS(Derived, ed::perception::Worker)

#include "ed/perception/worker_input.h"
#include "ed/perception/worker_output.h"

namespace ed
{
namespace perception
{

class Module
{

public:

    Module(const std::string& name) : name_(name) {}

    virtual ~Module() {}

    virtual void process(const WorkerInput& input, WorkerOutput& output) = 0;

    virtual void loadModel(const std::string& model_name, const std::string& model_path) {}

    virtual void loadConfig(const std::string& config_path) {}

    virtual void configure(tue::Configuration config) {}

    const std::string& name() const { return name_; }

private:

    std::string name_;

};

boost::shared_ptr<Module> loadPerceptionModule(class_loader::ClassLoader* loader, const std::string& model_list_name = std::string());

bool loadModelList(std::string& model_list_path, std::vector<std::string>& model_list);

} // end namespace ed

} // end namespace perception

#endif
