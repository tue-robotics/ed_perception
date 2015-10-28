#include <ed/perception/module.h>
//#include <ed/perception/aggregator.h>

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>

// Measurement data structures
#include <ed/measurement.h>
#include <rgbd/Image.h>

// File crawling
#include <tue/filesystem/crawler.h>

// Model loading
#include <ros/package.h>

// Measurement loading
#include <ed/io/filesystem/read.h>
#include <fstream>
#include <rgbd/serialization.h>
#include <ed/serialization/serialization.h>

// Show measurement
#include <opencv2/highgui/highgui.hpp>

// Include the perception plugin
#include "../src/perception_plugin.h"

#include "confusionmatrix.h"

// ----------------------------------------------------------------------------------------------------

int resize_factor = 20;

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cout << "Usage:\n\n   test-perception MEASUREMENT_DIRECTORY ED_CONFIG_FILE\n\n";
        return 1;
    }

    std::string measurement_dir = argv[1];
    std::string config_filename = argv[2];

    // - - - - -

    ed::perception::PerceptionPlugin plugin;

    // Needed to configure the plugin
    ed::PropertyKeyDB ed_property_key_db;

    tue::Configuration config;
    config.loadFromYAMLFile(config_filename);

    if (config.hasError())
    {
        std::cout << std::endl << "Error during configuration:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    if (config.readArray("plugins", tue::REQUIRED))
    {
        while(config.nextArrayItem())
        {
            std::string plugin_name, plugin_lib;
            if (!config.value("name", plugin_name) || !config.value("lib", plugin_lib) || plugin_name != "perception")
                continue;

            if (config.readGroup("parameters", tue::REQUIRED))
            {
                ed::InitData init(ed_property_key_db, config);
                plugin.initialize(init);

                config.endGroup();
            }
        }

        config.endArray();
    }

    if (config.hasError())
    {
        std::cout << std::endl << "Error during configuration:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    // - - - - -

    ConfusionMatrix cm(plugin.model_list());

    tue::filesystem::Crawler crawler(measurement_dir);

    std::set<std::string> files_had;

    int n_measurements = 0;
    tue::filesystem::Path filename;
    while(crawler.nextPath(filename))
    {
        // Get measurement id
        std::string filename_without_ext = filename.withoutExtension().string();

        // Get ground truth from folder name of containing file
        std::string truth = filename.parentPath().filename();

        // If file already done, continue to next file
        if (files_had.find(filename_without_ext) != files_had.end())
            continue;

        // Insert measurement id in files that were already handled
        files_had.insert(filename_without_ext);

        // We need an entity in a world model to run perception on
        ed::EntityConstPtr e;
        ed::WorldModel wm;

        // Add measurement to entity in world model using update request
        if (tue::filesystem::Path(filename_without_ext + ".json").exists())
        {
            ed::UpdateRequest update_req;
            if (!ed::readEntity(filename_without_ext + ".json", update_req))
                continue;

            wm.update(update_req);

            if (wm.numEntities() == 0)
                continue;

            e = *wm.begin();
        }
        else if (tue::filesystem::Path(filename_without_ext + ".mask").exists())
        {
            ed::MeasurementPtr msr(new ed::Measurement);
            if (!ed::read(filename_without_ext, *msr))
                continue;

            ed::EntityPtr e_temp(new ed::Entity("test-entity", "", 5));
            e_temp->addMeasurement(msr);

            e = e_temp;
        }

        if (!e)
            continue;

        // Create and configure perception worker input and output
        ed::perception::WorkerInput input;
        input.entity = e;

        ed::perception::WorkerOutput output;
        tue::Configuration result;
        output.data = result;

        // Set prior score of "unknown" entry in distribution
        input.type_distribution.setUnknownScore(plugin.unknown_probability_prior());

        // Add all possible model types to the type distribution
        for(std::vector<std::string>::const_iterator it = plugin.model_list().begin(); it != plugin.model_list().end(); ++it)
            input.type_distribution.setScore(*it, (1.0 - plugin.unknown_probability_prior()) / plugin.model_list().size());

        // Loop through perception modules
        const std::vector<boost::shared_ptr<ed::perception::Module> >& modules = plugin.perception_modules();
        for(std::vector<boost::shared_ptr<ed::perception::Module> >::const_iterator it = modules.begin(); it != modules.end(); ++it)
        {
            const boost::shared_ptr<ed::perception::Module>& module = *it;

            // Clear type distribution update
            output.type_update = ed::perception::CategoricalDistribution();

            // Process current module
            module->process(input, output);

            // Normalize output distribution
            output.type_update.normalize();

            // Update total type distribution
            input.type_distribution.update(output.type_update);

            std::string result_name;
            double d;
            output.type_update.getMaximum(result_name,d);
            if ( result_name != truth )
            {
                std::cout << module->name() << ":\nground truth: " << truth << "\nresult: " << result_name << "\n" << output.type_update << std::endl << std::endl;
            }

        }

        std::string result_name;
        double d;

        input.type_distribution.getMaximum(result_name,d);
        if ( result_name != truth )
        {
            std::cout << "Total: \n\t" << input.type_distribution << std::endl;
            std::cout << std::endl;
        }

        // Add perception result for current measurement to confusion matrix using the final type distribution and the ground truth
        cm.addResult(input.type_distribution,truth);

        ++n_measurements;

    }

    cm.show();

    if (n_measurements == 0)
        std::cout << "No measurements found." << std::endl;

    return 0;
}
