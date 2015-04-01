#ifndef ED_PERCEPTION_PLUGIN_H_
#define ED_PERCEPTION_PLUGIN_H_

#include <ed/plugin.h>

#include "ed/perception/module.h"
#include "ed/perception/worker.h"

namespace ed
{

namespace perception
{

class PerceptionPlugin : public ed::Plugin
{

public:

    PerceptionPlugin();

    virtual ~PerceptionPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    std::vector<std::string> plugin_paths_;

    std::vector<class_loader::ClassLoader*> perception_loaders_;

    std::vector<boost::shared_ptr<Module> > perception_modules_;

    std::map<UUID, boost::shared_ptr<Worker> > workers_;

};

}

}

#endif
