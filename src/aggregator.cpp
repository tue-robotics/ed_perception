#include "ed/perception/aggregator.h"

#include <tue/filesystem/path.h>
#include <ed/error_context.h>

namespace ed
{
namespace perception
{

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

}

// ----------------------------------------------------------------------------------------------------

Aggregator::Aggregator()
{
}

// ----------------------------------------------------------------------------------------------------

Aggregator::~Aggregator()
{
}

// ----------------------------------------------------------------------------------------------------

void Aggregator::configure(tue::Configuration config, bool for_training)
{
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

            // - - - - - - - - - - - - - - - - - - - - - - - - - - -
            // Create perception module

            class_loader->loadLibrary();
            std::vector<std::string> classes = class_loader->getAvailableClasses<Module>();

            if (classes.empty())
            {
                config.addError("Could not find any perception modules in '" + class_loader->getLibraryPath() + "'.");
                continue;
            }

            if (classes.size() > 1)
            {
                config.addError("Multiple perception modules registered in '" + class_loader->getLibraryPath() + "'.");
                continue;
            }

            boost::shared_ptr<Module> perception_module = class_loader->createInstance<Module>(classes.front());

            if (!perception_module)
            {
                config.addError("Loading perception module failed: '" + lib_file + "'.");
                continue;
            }

            // If module is going to be used for training, load training config
            if (for_training)
            {
                ed::ErrorContext errc("Configuring perception module", perception_module->name().c_str());

                if (config.readGroup("parameters"))
                {
                    perception_module->configureTraining(config.limitScope());
                    config.endGroup();
                }
                else
                {
                    tue::Configuration tmp_config;
                    perception_module->configureTraining(tmp_config);
                    if (tmp_config.hasError())
                        config.addError(tmp_config.error());
                }
            }

            // Add the perception module to the aggregator
            addModule(perception_module);
        }
        config.endArray();
    }

    config.value("classification_model_path", classification_model_path_);
}

// ----------------------------------------------------------------------------------------------------

void Aggregator::classify(const Entity& e, const std::string& property, const CategoricalDistribution& prior, ClassificationOutput& output) const
{
    std::vector<CategoricalDistribution> sub_distribs;

    for(std::vector<boost::shared_ptr<Module> >::const_iterator it = modules_.begin(); it != modules_.end(); ++it)
    {
        const Module& m = **it;
        if (!m.serves_property(property))
            continue;

        output.data.writeGroup(m.name());

        ClassificationOutput sub_output(output.data);
        sub_output.likelihood.setUnknownScore(0.01); // TODO
        m.classify(e, property, prior, sub_output);

        output.data.endGroup();

        sub_output.likelihood.normalize();
        sub_distribs.push_back(sub_output.likelihood);

//        output.likelihood.update(sub_output.likelihood);
    }

    for(std::vector<CategoricalDistribution>::const_iterator it = sub_distribs.begin(); it != sub_distribs.end(); ++it)
    {
        const CategoricalDistribution& d = *it;
        for(std::map<std::string, double>::const_iterator it2 = d.values().begin(); it2 != d.values().end(); ++it2)
            output.likelihood.setScore(it2->first, 1);
    }

    output.likelihood.setUnknownScore(0.01); // TODO
    output.likelihood.normalize();

    for(std::vector<CategoricalDistribution>::const_iterator it = sub_distribs.begin(); it != sub_distribs.end(); ++it)
    {
        const CategoricalDistribution& d = *it;
        output.likelihood.update(d);
    }
}

// ----------------------------------------------------------------------------------------------------

void Aggregator::addTrainingInstance(const Entity& e, const std::string& property, const std::string& value)
{
    for(std::vector<boost::shared_ptr<Module> >::const_iterator it = modules_.begin(); it != modules_.end(); ++it)
    {
        Module& m = **it;
        if (!m.serves_property(property))
            continue;

        m.addTrainingInstance(e, property, value);
    }
}

// ----------------------------------------------------------------------------------------------------

void Aggregator::train()
{
    for(std::vector<boost::shared_ptr<Module> >::const_iterator it = modules_.begin(); it != modules_.end(); ++it)
    {
        Module& m = **it;
        m.train();
    }
}

// ----------------------------------------------------------------------------------------------------

void Aggregator::loadRecognitionData()
{
    for(std::vector<boost::shared_ptr<Module> >::const_iterator it = modules_.begin(); it != modules_.end(); ++it)
    {
        Module& m = **it;

        tue::filesystem::Path p(classification_model_path_);
        p = p.join(m.name());

        if (p.exists())
            m.loadRecognitionData(p.string());
    }
}

// ----------------------------------------------------------------------------------------------------

void Aggregator::saveRecognitionData() const
{
    for(std::vector<boost::shared_ptr<Module> >::const_iterator it = modules_.begin(); it != modules_.end(); ++it)
    {
        Module& m = **it;

        tue::filesystem::Path p(classification_model_path_);
        p = p.join(m.name());
        m.saveRecognitionData(p.string());
    }
}

// ----------------------------------------------------------------------------------------------------

void Aggregator::addModule(const boost::shared_ptr<Module>& m)
{
    modules_.push_back(m);
}

// ----------------------------------------------------------------------------------------------------

} // end namespace ed

} // end namespace perception

