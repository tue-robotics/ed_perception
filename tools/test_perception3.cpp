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

int resize_factor = 20;

// ----------------------------------------------------------------------------------------------------

class ConfusionMatrix
{
public:
//    friend std::ostream& operator<<(std::ostream& os, const ConfusionMatrix& cm);

    ConfusionMatrix(std::vector<std::string> options)
    {
        options_ = options;
        mat_ = std::vector<int>(options.size()*options.size(),0.0);
        maximum_ = 0;
    }

//    std::ostream& operator<<(std::ostream& os, const ConfusionMatrix& cm)
//    {

//    }

    void print()
    {
        for (std::vector<std::string>::const_iterator it = options_.begin(); it != options_.end(); it++ )
        {
            printf("%-15s", it->c_str());
        }

        printf("\n");

        int i = 0, j = 0; // column and row, respectively

        for (std::vector<int>::const_iterator it = mat_.begin(); it != mat_.end(); it++ )
        {
            printf("%-15i", *it);

            if ( i == options_.size() - 1 )
            {
                printf("\n");
                i = 0;
                j++;
            }
            else
                i++;
        }
    }

    cv::Mat toCvMat(int factor)
    {
        unsigned int column = 0, row = 0;
        cv::Mat mat = cv::Mat(options_.size(), options_.size(), CV_64FC3, cv::Scalar(0.0,0.0,0.0));
        for ( std::vector<int>::iterator it = mat_.begin(); it != mat_.end(); it++ )
        {
            if ( *it > 0 )
            {
                mat.at<cv::Vec3d>(row,column)[2] = 1.0; // times *iterator?
                if ( row == column )
                {
                    mat.at<cv::Vec3d>(row,column)[2] = 0.0;
                    mat.at<cv::Vec3d>(row,column)[1] = 1.0; // times *iterator?
                }

            }

            if ( column == options_.size()-1 )
            {
                row++;
                column = 0;
            }
            else
                column++;
        }

        cv::Mat dst;

        cv::resize(mat,dst,cv::Size(0,0),factor,factor,cv::INTER_NEAREST);

        for ( int i = 0; i < options_.size(); i++ )
        {
            cv::line(dst,cv::Point(0,factor*i),cv::Point(factor*options_.size(),factor*i),cv::Scalar(1,1,1));
            cv::line(dst,cv::Point(factor*i,0),cv::Point(factor*i,factor*options_.size()),cv::Scalar(1,1,1));
        }

        return dst;
    }

    void addResult(const ed::perception::CategoricalDistribution& dstr, const std::string& cat)
    {
        std::string label;
        double score;
        int labeli = -1, cati = -1;

        dstr.getMaximum(label,score);

        std::cout << "Ground truth: " << cat << std::endl;
        std::cout << "Perc. result: " << label << std::endl;

        for ( int i = 0; i < options_.size(); i++ )
        {
            if (options_[i] == label)
                labeli = i;
            if (options_[i] == cat)
                cati = i;
        }

        if (labeli == -1)
        {
            std::cout << "Item with maximum score is not one of the options" << std::endl;
            return;
        }
        if (cati == -1)
        {
            std::cout << "Ground truth item not one of the options" << std::endl;
            return;
        }

        std::cout << "Adding result at gt and res indices " << cati << " and " << labeli << std::endl;

        mat_[cati*options_.size()+labeli]++;
        if ( mat_[cati*options_.size()+labeli] > maximum_ )
            maximum_ = mat_[cati*options_.size()+labeli];
    }

    int size()
    {
        return options_.size();
    }

    int getMaximum()
    {
        return maximum_;
    }

    std::vector<std::string> getOptions()
    {
        return options_;
    }

    int at(int result, int truth )
    {
        return mat_[truth*options_.size()+result];
    }

private:
    std::vector<int> mat_;
    std::vector<std::string> options_;
    int maximum_;
};

// ----------------------------------------------------------------------------------------------------

void onMouse(int event, int x, int y, int flags, void* param)
{
    ConfusionMatrix* cm = (ConfusionMatrix*) param;
    cv::Mat mat = cm->toCvMat(resize_factor);

    char text[100];
    char counttext[20];
    cv::Mat img2;

    img2 = mat.clone();

    int resulti = x/resize_factor;
    int gtruthi = y/resize_factor;

    std::string result = cm->getOptions()[resulti];
    std::string gtruth = cm->getOptions()[gtruthi];

    int count = cm->at(resulti,gtruthi);

    sprintf(text, "Perception result: %s, ground truth: %s.", result.c_str(), gtruth.c_str());
    sprintf(counttext, "Count=%i", count);
    cv::putText(img2, text, cv::Point(5,15), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0));
    cv::putText(img2, counttext, cv::Point(5,30), cv::FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0));
    cv::imshow("Confusion matrix", img2);
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
        }

        std::cout << "Total: \n\t" << input.type_distribution << std::endl;
        std::cout << std::endl;

        // Add perception result for current measurement to confusion matrix using the final type distribution and the ground truth
        cm.addResult(input.type_distribution,truth);

        ++n_measurements;

        cv::namedWindow("Confusion matrix");
        cv::setMouseCallback("Confusion matrix", onMouse, &cm);
        cv::imshow("Confusion matrix", cm.toCvMat(resize_factor));
        cv::waitKey();
    }

    if (n_measurements == 0)
        std::cout << "No measurements found." << std::endl;

    return 0;
}
