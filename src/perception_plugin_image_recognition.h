#ifndef ED_PERCEPTION_PLUGIN_TENSORFLOW_H_
#define ED_PERCEPTION_PLUGIN_TENSORFLOW_H_

#include <ed/plugin.h>

// Service
#include <ed_perception/Classify.h>
#include <ros/service_server.h>
#include <ros/service_client.h>
#include <ros/callback_queue.h>

namespace ed
{

namespace perception
{

class PerceptionPluginImageRecognition : public ed::Plugin
{

public:

    PerceptionPluginImageRecognition();

    virtual ~PerceptionPluginImageRecognition();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    std::string perception_models_path_;

    // SERVICE

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;

    ros::CallbackQueue cb_queue_;

    ros::ServiceServer srv_classify_;

    bool srvClassify(ed_perception::Classify::Request& req, ed_perception::Classify::Response& res);

    /** Service client to pass on services */
    ros::ServiceClient srv_client_;

};

}

}

#endif
