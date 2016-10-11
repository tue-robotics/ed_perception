#include "perception_plugin_tensorflow.h"

#include <iostream>

#include <ros/package.h>
#include <ros/node_handle.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/error_context.h>
#include <ed/measurement.h>

#include <rgbd/Image.h>
#include <rgbd/ros/conversions.h>

#include <tue/filesystem/path.h>

#include <object_recognition_srvs/Recognize.h>


namespace ed
{

namespace perception
{

// ----------------------------------------------------------------------------------------------------

PerceptionPluginTensorflow::PerceptionPluginTensorflow()
{
}

// ----------------------------------------------------------------------------------------------------

PerceptionPluginTensorflow::~PerceptionPluginTensorflow()
{
}

// ----------------------------------------------------------------------------------------------------

void PerceptionPluginTensorflow::initialize(ed::InitData& init)
{
    // Initialize service
    ros::NodeHandle nh_private("~");
    nh_private.setCallbackQueue(&cb_queue_);
    srv_classify_ = nh_private.advertiseService("classify", &PerceptionPluginTensorflow::srvClassify, this);

    ros::NodeHandle nh;
    srv_client_ = nh.serviceClient<object_recognition_srvs::Recognize>("recognize");
}

// ----------------------------------------------------------------------------------------------------

void PerceptionPluginTensorflow::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    world_ = &data.world;
    update_req_ = &req;

    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool PerceptionPluginTensorflow::srvClassify(ed_perception::Classify::Request& req, ed_perception::Classify::Response& res)
{

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Classify

    for(std::vector<std::string>::const_iterator it = req.ids.begin(); it != req.ids.end(); ++it)
    {
        ed::EntityConstPtr e = world_->getEntity(*it);

        // Check if the entity exists
        if (!e)
        {
            res.error_msg += "Entity '" + *it + "' does not exist.\n";
            ROS_ERROR_STREAM(res.error_msg);
            continue;
        }

        // Check if the entity has a measurement associated with it
        if (!e->bestMeasurement())
        {
            res.error_msg += "Entity '" + *it + "' does not have a measurement.\n";
            ROS_ERROR_STREAM(res.error_msg);
            continue;
        }
        MeasurementConstPtr meas_ptr = e->bestMeasurement();

        // Create the classificationrequest and call the service
        object_recognition_srvs::Recognize client_srv;
        rgbd::convert(meas_ptr->image()->getRGBImage(), client_srv.request.image);
        srv_client_.call(client_srv);

        std::string label;
        if (client_srv.response.recognitions.size() > 0)
        {
            label = client_srv.response.recognitions[0].label;
            ROS_INFO_STREAM("Entity " + *it + ": " + label);
        }
        else
        {
            ROS_WARN_STREAM("No classification for entity " + *it);
            continue;
        }

        // Update the world model
        update_req_->setType(e->id(), label);
//        update_req_->addData(e->id(), data.data()); // What does this do???

        // Add the result to the response
        res.expected_values.push_back(label);
        res.expected_value_probabilities.push_back(1.0);  // ToDo: does this make any sense?

    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace perception

} // end namespace ed

ED_REGISTER_PLUGIN(ed::perception::PerceptionPluginTensorflow)
