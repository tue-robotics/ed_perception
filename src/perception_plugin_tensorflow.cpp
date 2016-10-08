#include "perception_plugin_tensorflow.h"

#include <iostream>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/error_context.h>

#include <tue/filesystem/path.h>

#include <ros/package.h>
#include <ros/node_handle.h>

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
//    tue::Configuration& config = init.config;
//    aggregator_.configure(config, false);

    // Initialize service
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);
    srv_classify_ = nh.advertiseService("classify", &PerceptionPlugin::srvClassify, this);
    srv_add_training_instance_ = nh.advertiseService("add_training_instance", &PerceptionPlugin::srvAddTrainingInstance, this);
}

// ----------------------------------------------------------------------------------------------------

void PerceptionPluginTensorflow::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    world_ = &data.world;
    update_req_ = &req;

    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

bool PerceptionPluginTensorflow::configureClassifier(const std::string& perception_models_path, std::string& error)
{
    if (perception_models_path_ != perception_models_path)
    {
        std::string config_filename = perception_models_path + "/parameters.yaml";
        tue::Configuration config;
        config.loadFromYAMLFile(config_filename);

        if (!config.hasError())
            aggregator_.configure(config, false);

        if (config.hasError())
        {
            error = "Could not load configuration file '" + config_filename + "':\n\n" + config.error();
            return false;
        }

        perception_models_path_ = perception_models_path;
    }
    else if (perception_models_path.empty())
    {
        error = "Please provide perception model path.";
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool PerceptionPluginTensorflow::srvClassify(ed_perception::Classify::Request& req, ed_perception::Classify::Response& res)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure classifier

    if (!configureClassifier(req.perception_models_path, res.error_msg))
    {
        ROS_ERROR_STREAM(res.error_msg);
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Convert prior msg to distribution

    CategoricalDistribution prior;
    for(unsigned int i = 0; i < req.prior.values.size(); ++i)
        prior.setScore(req.prior.values[i], req.prior.probabilities[i]);
    prior.setUnknownScore(req.prior.unknown_probability);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Classify

    for(std::vector<std::string>::const_iterator it = req.ids.begin(); it != req.ids.end(); ++it)
    {
        ed::EntityConstPtr e = world_->getEntity(*it);
        if (!e)
        {
            res.error_msg += "Entity '" + *it + "' does not exist.\n";
            ROS_ERROR_STREAM(res.error_msg);
            continue;
        }

        if (!e->bestMeasurement())
        {
            res.error_msg += "Entity '" + *it + "' does not have a measurement.\n";
            ROS_ERROR_STREAM(res.error_msg);
            continue;
        }

        tue::Configuration data;
        ed::perception::ClassificationOutput output(data);
        aggregator_.classify(*e, req.property, prior, output);

        // TODO: create posterior! (is now just likelihood)
        ed::perception::CategoricalDistribution posterior = output.likelihood;
        posterior.normalize();

        // - - - - - - - - - - - - - - - - - - - - - -
        // Write result to message

        res.ids.push_back(e->id().str());

        res.posteriors.push_back(ed_perception::CategoricalDistribution());
        ed_perception::CategoricalDistribution& dist_msg = res.posteriors.back();

        for(std::map<std::string, double>::const_iterator it2 = posterior.values().begin(); it2 != posterior.values().end(); ++it2)
        {
            dist_msg.values.push_back(it2->first);
            dist_msg.probabilities.push_back(it2->second);
        }

        dist_msg.unknown_probability = posterior.getUnknownScore();


        std::string max_value;
        double max_prob;
        if (posterior.getMaximum(max_value, max_prob) && max_prob > posterior.getUnknownScore())
        {
            res.expected_values.push_back(max_value);
            res.expected_value_probabilities.push_back(max_prob);

            // If the classification property is 'type', set the type of the entity in the world model
            if (req.property == "type")
            {
                update_req_->setType(e->id(), max_value);
                update_req_->addData(e->id(), data.data());
            }
        }
        else
        {
            res.expected_values.push_back("");
            res.expected_value_probabilities.push_back(posterior.getUnknownScore());
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool PerceptionPluginTensorflow::srvAddTrainingInstance(ed_perception::AddTrainingInstance::Request& req,
                                              ed_perception::AddTrainingInstance::Response& res)
{
    std::cout << "PerceptionPlugin::srvAddTrainingInstance" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure classifier

    if (!configureClassifier(req.perception_models_path, res.error_msg))
        return true;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check if entity and its measurement exist

    ed::EntityConstPtr e = world_->getEntity(req.id);
    if (!e)
    {
        res.error_msg += "Entity '" + req.id + "' does not exist.\n";
        return true;
    }

    if (!e->bestMeasurement())
    {
        res.error_msg += "Entity '" + req.id + "' does not have a measurement.\n";
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Add training instance

    aggregator_.addTrainingInstance(*e, req.property, req.value);

    return true;
}

// ----------------------------------------------------------------------------------------------------


} // end namespace perception

} // end namespace ed

ED_REGISTER_PLUGIN(ed::perception::PerceptionPluginTensorflow)
