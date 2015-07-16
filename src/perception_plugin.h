#ifndef ED_PERCEPTION_PLUGIN_H_
#define ED_PERCEPTION_PLUGIN_H_

#include <ed/plugin.h>

#include "ed/perception/module.h"
#include "ed/perception/worker.h"

// Service
#include <ed_perception/Classify.h>
#include <ros/service_server.h>
#include <ros/callback_queue.h>

// Entity live viewer
#include "../tools/entity_live_viewer/entity_live_viewer_cv.h"

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

    const std::vector<boost::shared_ptr<Module> >& perception_modules() const { return perception_modules_; }

    const std::vector<std::string>& model_list() const { return model_list_; }

    double unknown_probability_prior() const { return unknown_probability_prior_; }

private:

    // List of possible object types
    std::vector<std::string> model_list_;

    std::vector<std::string> plugin_paths_;

    std::vector<class_loader::ClassLoader*> perception_loaders_;

    std::vector<boost::shared_ptr<Module> > perception_modules_;

    std::map<UUID, boost::shared_ptr<Worker> > workers_;

    double type_persistence_;

    double unknown_probability_prior_;

    EntityLiveViewerCV* entity_viewer_;
    bool enable_live_viewer_;


    // SERVICE

    bool continuous_;

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_classify_;

    bool srvClassify(ed_perception::Classify::Request& req, ed_perception::Classify::Response& res);

};

}

}

#endif
