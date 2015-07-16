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

// ----------------------------------------------------------------------------------------------------

void showMeasurement(const ed::Measurement& msr)
{
    const cv::Mat& rgb_image = msr.image()->getRGBImage();
    const cv::Mat& depth_image = msr.image()->getDepthImage();

    cv::Mat masked_rgb_image(rgb_image.rows, rgb_image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat masked_depth_image(depth_image.rows, depth_image.cols, depth_image.type(), 0.0);

    for(ed::ImageMask::const_iterator it = msr.imageMask().begin(rgb_image.cols); it != msr.imageMask().end(); ++it)
        masked_rgb_image.at<cv::Vec3b>(*it) = rgb_image.at<cv::Vec3b>(*it);

    for(ed::ImageMask::const_iterator it = msr.imageMask().begin(depth_image.cols); it != msr.imageMask().end(); ++it)
        masked_depth_image.at<float>(*it) = depth_image.at<float>(*it);

    cv::imshow("Measurement: depth", masked_depth_image / 8);
    cv::imshow("Measurement: rgb", masked_rgb_image);
}

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

    tue::filesystem::Crawler crawler(measurement_dir);

    std::set<std::string> files_had;

    int n_measurements = 0;
    tue::filesystem::Path filename;
    while(crawler.nextPath(filename))
    {
        std::string filename_without_ext = filename.withoutExtension().string();
        if (files_had.find(filename_without_ext) != files_had.end())
            continue;

        files_had.insert(filename_without_ext);

        ed::EntityConstPtr e;
        ed::WorldModel wm;

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

        if (e->lastMeasurement())
            showMeasurement(*e->lastMeasurement());

        std::cout << "\n\n------------------------------------------------------------" << std::endl;
        std::cout << "    " << filename.withoutExtension() << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl << std::endl;

        ed::perception::WorkerInput input;
        input.entity = e;

        ed::perception::WorkerOutput output;
        tue::Configuration result;
        output.data = result;

        input.type_distribution.setUnknownScore(plugin.unknown_probability_prior());
        // Add all possible model types to the type distribution
        for(std::vector<std::string>::const_iterator it = plugin.model_list().begin(); it != plugin.model_list().end(); ++it)
            input.type_distribution.setScore(*it, (1.0 - plugin.unknown_probability_prior()) / plugin.model_list().size());

//        input.type_distribution.setUnknownScore(0.01); // TODO: magic number (probability that an object you encounter is unknown (not in the model list))

//        input.type_distribution.normalize();


        const std::vector<boost::shared_ptr<ed::perception::Module> >& modules = plugin.perception_modules();

        for(std::vector<boost::shared_ptr<ed::perception::Module> >::const_iterator it = modules.begin(); it != modules.end(); ++it)
        {
            const boost::shared_ptr<ed::perception::Module>& module = *it;

            // Clear type distribution update
            output.type_update = ed::perception::CategoricalDistribution();

            module->process(input, output);

            output.type_update.normalize();

            std::cout << module->name() << ":\n\t" << output.type_update << "\n" << std::endl;

            // Update total type distribution
            input.type_distribution.update(output.type_update);
        }

        std::cout << "Total: \n\t" << input.type_distribution << std::endl;
        std::cout << std::endl;

        // print perception result
        std::cout << result << std::endl;

        std::string max_type;
        double max_score;
        if (input.type_distribution.getMaximum(max_type, max_score) && max_score > input.type_distribution.getUnknownScore())
            std::cout << "Expected type: " << max_type << " (probability = " << max_score << ")" << std::endl;
        else
            std::cout << "Unknown entity (probability = " << input.type_distribution.getUnknownScore() << ")" << std::endl;

        ++n_measurements;

        cv::waitKey();
    }

    if (n_measurements == 0)
        std::cout << "No measurements found." << std::endl;

    return 0;
}
