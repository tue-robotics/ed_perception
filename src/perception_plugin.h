#ifndef ED_PERCEPTION_PLUGIN_H_
#define ED_PERCEPTION_PLUGIN_H_

#include <ed/plugin.h>

#include "ed/perception/aggregator.h"

// Service
#include <ed_perception/Classify.h>
#include <ros/service_server.h>
#include <ros/callback_queue.h>

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

    Aggregator aggregator_;

    std::string perception_models_path_;

    // SERVICE

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_classify_;

    bool srvClassify(ed_perception::Classify::Request& req, ed_perception::Classify::Response& res);

};

}

}

#endif
