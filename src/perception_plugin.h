#ifndef ED_PERCEPTION_PLUGIN_H_
#define ED_PERCEPTION_PLUGIN_H_

#include <ed/plugin.h>

#include "ed/perception/aggregator.h"

// Service
#include <ed_perception/Classify.h>
#include <ed_perception/AddTrainingInstance.h>
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

    bool configureClassifier(const std::string& perception_models_path, std::string& error);

    // SERVICE

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_classify_;

    bool srvClassify(ed_perception::Classify::Request& req, ed_perception::Classify::Response& res);

    ros::ServiceServer srv_add_training_instance_;

    bool srvAddTrainingInstance(ed_perception::AddTrainingInstance::Request& req,
                                ed_perception::AddTrainingInstance::Response& res);


};

}

}

#endif
