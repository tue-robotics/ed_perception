#include "perception_plugin_image_recognition.h"

#include <iostream>

#include <ros/package.h>
#include <ros/node_handle.h>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/error_context.h>
#include <ed/measurement.h>

#include "../plugins/shared_methods.h"

#include <rgbd/Image.h>
#include <rgbd/ros/conversions.h>

#include <tue/filesystem/path.h>

#include <image_recognition_msgs/Recognize.h>


namespace ed
{

namespace perception
{

// ----------------------------------------------------------------------------------------------------

PerceptionPluginImageRecognition::PerceptionPluginImageRecognition()
{
}

// ----------------------------------------------------------------------------------------------------

PerceptionPluginImageRecognition::~PerceptionPluginImageRecognition()
{
}

// ----------------------------------------------------------------------------------------------------

void PerceptionPluginImageRecognition::initialize(ed::InitData& init)
{
    // Initialize service
    ros::NodeHandle nh_private("~");
    nh_private.setCallbackQueue(&cb_queue_);
    srv_classify_ = nh_private.advertiseService("classify", &PerceptionPluginImageRecognition::srvClassify, this);

    ros::NodeHandle nh;
    srv_client_ = nh.serviceClient<image_recognition_msgs::Recognize>("object_recognition/recognize");
}

// ----------------------------------------------------------------------------------------------------

void PerceptionPluginImageRecognition::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    world_ = &data.world;
    update_req_ = &req;

    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool PerceptionPluginImageRecognition::srvClassify(ed_perception::Classify::Request& req, ed_perception::Classify::Response& res)
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
        image_recognition_msgs::Recognize client_srv;
        cv::Mat image = meas_ptr->image()->getRGBImage();

        // Get the part that is masked
        ed::ImageMask mask = meas_ptr->imageMask();

        cv::Point p_min(image.cols, image.rows);
        cv::Point p_max(0, 0);

        for(ed::ImageMask::const_iterator it2 = mask.begin(image.cols); it2 != mask.end(); ++it2)
        {
            const cv::Point2i& p = *it2;
            p_min.x = std::min(p_min.x, p.x);
            p_min.y = std::min(p_min.y, p.y);
            p_max.x = std::max(p_max.x, p.x);
            p_max.y = std::max(p_max.y, p.y);
        }

        cv::Rect roi = cv::Rect(std::min(p_min.x + 5, image.cols),
                                std::min(p_min.y + 5, image.rows),
                                std::max(p_max.x - p_min.x - 5, 0),
                                std::max(p_max.y - p_min.y - 5, 0));
        cv::Mat cropped_image = image(roi);

        // Convert it to the image request and call the service
        rgbd::convert(cropped_image, client_srv.request.image);
        if (!srv_client_.call(client_srv))
        {
            ROS_ERROR("Service call failed");
            continue;
        }

        // If we have recognitions and the highest probability is above a certain
        // threshold: update the world model
        double best_probability = 0; // We just always update with our best guess
        std::string label;
        if (client_srv.response.recognitions.size() > 0)
        {
            const image_recognition_msgs::Recognition& r = client_srv.response.recognitions[0];  // Assuming that the first recognition is the best one!
            for ( int i = 0; i < r.categorical_distribution.probabilities.size(); i++ )
            {
                const image_recognition_msgs::CategoryProbability& p = r.categorical_distribution.probabilities[i];

                if ( p.probability > best_probability )
                {
                    best_probability = p.probability;
                    label = p.label;
                }
            }
        }
        else
        {
            ROS_ERROR_STREAM("No classification for entity " + *it);
            continue;
        }

        // Add the result to the response
        if (best_probability > req.unknown_probability)
        {
          update_req_->setType(e->id(), label);
        }

        // For some reason we defined the interface this way but this is much too much info for the client ..
        // I am now setting all these things because the client expects this for some reason ..
        // posteriors = [dict(zip(distr.values, distr.probabilities)) for distr in res.posteriors]
        // return [ClassificationResult(_id, exp_val, exp_prob, distr) for _id, exp_val, exp_prob, distr in zip(res.ids, res.expected_values, res.expected_value_probabilities, posteriors) if exp_val in types]

        ed_perception::CategoricalDistribution posterior;
        for (unsigned int i = 0; i < client_srv.response.recognitions[0].categorical_distribution.probabilities.size(); ++i) // Assuming that there is only one recognition!
        {
            posterior.values.push_back(client_srv.response.recognitions[0].categorical_distribution.probabilities[i].label);
            posterior.probabilities.push_back(client_srv.response.recognitions[0].categorical_distribution.probabilities[i].probability);
        }

        res.ids.push_back(e->id().str());
        res.expected_values.push_back(label);
        res.expected_value_probabilities.push_back(best_probability);
        res.posteriors.push_back(posterior);
    }

    ROS_DEBUG_STREAM("response: return true: " << res << "");

    return true;
}

// ----------------------------------------------------------------------------------------------------

} // end namespace perception

} // end namespace ed

ED_REGISTER_PLUGIN(ed::perception::PerceptionPluginImageRecognition)
