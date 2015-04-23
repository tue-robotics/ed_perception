#include "perception_plugin.h"

#include <iostream>

#include <ed/world_model.h>
#include <ed/entity.h>
#include <ed/update_request.h>
#include <ed/error_context.h>

#include <tue/filesystem/path.h>

#include <ros/package.h>

// ----------------------------------------------------------------------------------------------------

namespace
{

bool getEnvironmentVariable(const std::string& var, std::string& value)
{
     const char * val = ::getenv(var.c_str());
     if ( val == 0 )
         return false;

     value = val;
     return true;
}

// ----------------------------------------------------------------------------------------------------

bool loadModelList(std::string& model_list_path, std::vector<std::string>& model_list){
    tue::Configuration conf;
    std::string model_name;

    if (!conf.loadFromYAMLFile(model_list_path)) // read YAML configuration
        return false;

    if (!conf.readArray("models"))           // read Model group
    {
        std::cout << "[ED PERCEPTION] While reading file " << model_list_path << ": could not find 'models' group." << std::endl;
        return false;
    }

    while(conf.nextArrayItem())
    {
        if(conf.value("name", model_name))
            model_list.push_back(model_name);
    }

    return true;
}

}

// ----------------------------------------------------------------------------------------------------

namespace ed
{

namespace perception
{

// ----------------------------------------------------------------------------------------------------

PerceptionPlugin::PerceptionPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

PerceptionPlugin::~PerceptionPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void PerceptionPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    std::string model_list_name = "";

    // Get the plugin paths
    std::string ed_plugin_paths;
    if (getEnvironmentVariable("ED_PLUGIN_PATH", ed_plugin_paths))
    {
        std::stringstream ss(ed_plugin_paths);
        std::string item;
        while (std::getline(ss, item, ':'))
            plugin_paths_.push_back(item);
    }
    else
    {
        config.addError("Environment variable ED_PLUGIN_PATH not set.");
        return;
    }

    // read model list name to be used
    if (!config.value("model_list", model_list_name))
    {
//        std::cout << "Could not find model list name. Using all models." << std::endl;
        return;
    }

    config.value("type_persistence", type_persistence_);

    std::string object_models_path = ros::package::getPath("ed_object_models");
    std::string model_list_path = object_models_path + "/configs/model_lists/" + model_list_name;

    model_list_.clear();
    if (!loadModelList(model_list_path, model_list_))
    {
        init.config.addError("Error reading file '" + model_list_path + "'");
        return;
    }

    if (config.readArray("modules"))
    {
        while(config.nextArrayItem())
        {
            int enabled = 1;
            if (config.value("enabled", enabled, tue::OPTIONAL) && enabled == 0)
                continue;

            std::string lib;
            if (!config.value("lib", lib))
                continue;

            std::string lib_file;
            for(std::vector<std::string>::const_iterator it = plugin_paths_.begin(); it != plugin_paths_.end(); ++it)
            {
                std::string lib_file_test = *it + "/" + lib;
                if (tue::filesystem::Path(lib_file_test).exists())
                {
                    lib_file = lib_file_test;
                    break;
                }
            }

            if (lib_file.empty())
            {
                config.addError("Perception plugin '" + lib + "' could not be found.");
                return;
            }

            // Load the library
            class_loader::ClassLoader* class_loader = new class_loader::ClassLoader(lib_file);
            perception_loaders_.push_back(class_loader);

            // Create perception module
            boost::shared_ptr<Module> perception_module = ed::perception::loadPerceptionModule(class_loader, model_list_name);

            if (perception_module)
            {
                // Configure the module if there is a 'parameters' group in the config
                if (config.readGroup("parameters"))
                {
                    ed::ErrorContext errc("Configuring perception module", perception_module->name().c_str());
                    perception_module->configure(config.limitScope());
                    config.endGroup();
                }

                // Add the perception module to the aggregator
                perception_modules_.push_back(perception_module);
            }
            else
            {
                config.addError("No valid perception module could be found in '" + lib_file + "'.");
            }

        }

        config.endArray();
    }
}

// ----------------------------------------------------------------------------------------------------

void PerceptionPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    // Don't update if there are no perception modules
    if (perception_modules_.empty())
        return;

    int n = 0;
    for(ed::WorldModel::const_iterator it = data.world.begin(); it != data.world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (e->shape() || e->convexHull().chull.empty())
            continue;

        ++n;

        const ed::UUID& id = e->id();

//        if (e->type() != "")
//        {
//            std::map<ed::UUID, std::string>::iterator it2 = previous_entity_types_.find(e->id());
//            if (it2 == previous_entity_types_.end() || it2->second != e->type())
//            {
//                // The entity received a type it did not have before. Therefore, update it and fit
//                // a shape model
//                it->second = updateEntityType(e, e->type(), fit_shape_);

//                previous_entity_types_[id] = e->type();

//                // We do not have to start a perception worker since we already have a type
//                continue;
//            }
//        }

        std::map<UUID, boost::shared_ptr<Worker> >::iterator it_worker = workers_.find(id);
        if (it_worker == workers_.end())
        {
            // No worker active for this entity, so create one

            // create worker and add measurements
            boost::shared_ptr<Worker> worker(new Worker(model_list_, type_persistence_));
            worker->setEntity(e);
            worker->setPerceptionModules(perception_modules_);

            workers_[id] = worker;
            worker->start();
        }
        else
        {
            // Already a worker active
            boost::shared_ptr<Worker> worker = it_worker->second;

            // Check if it is idle, but has done work before
            if (worker->isIdle() && worker->t_last_processing > 0)
            {
                // Worker has already done work and finished. Check if we want to run it again

                // Get the latest measurements since the last measurement processed by the worker
                std::vector<MeasurementConstPtr> measurements;
                e->measurements(measurements, worker->t_last_processing);

                if (!measurements.empty())
                {
                    // There are new measurements, so run the worker again
                    worker->setEntity(e);
                    worker->start();
                }
            }
            // Check if it just finished processing
            else if (worker->isDone())
            {
                const CategoricalDistribution& type_dist = worker->getTypeDistribution();

                std::string expected_type;
                double type_score;

                // Set type, if score is high enough
                if (type_dist.getMaximum(expected_type, type_score) && type_score > type_dist.getUnknownScore())
                    req.setType(e->id(), expected_type);

                // Update the entity with the results from the worker
                if (!worker->getResult().empty())
                    req.addData(e->id(), worker->getResult());

                // Set worker to idle. This way, the result is not checked again on the next iteration
                worker->setIdle();

                worker->t_last_processing = worker->timestamp();
            }
        }
    }
    
    // Filter idle workers of deleted entities
    for(std::map<UUID, boost::shared_ptr<Worker> >::iterator it = workers_.begin(); it != workers_.end();)
    {
        const boost::shared_ptr<Worker>& worker = it->second;

        if (!data.world.getEntity(it->first))
        {
            if (worker->isRunning())
            {
                worker->signalStop();
                ++it;
            }
            else
            {
                workers_.erase(it++);
            }
        }
        else
        {
            ed::EntityConstPtr e = data.world.getEntity(it->first);
            if (e->convexHull().chull.empty())
                std::cout << "WARNING: Entity " << e->id() << " has empty convex hull!" << std::endl;

            ++it;
        }
    }

//    std::cout << "Num entities: " << n << ", num workers = " << workers_.size() << std::endl;
}

}

}

ED_REGISTER_PLUGIN(ed::perception::PerceptionPlugin)
