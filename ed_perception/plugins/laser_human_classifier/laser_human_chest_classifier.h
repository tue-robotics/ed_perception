/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: March 2015
*/

#ifndef LASER_HUMAN_CHEST_CLASSIFIER_H_
#define LASER_HUMAN_CHEST_CLASSIFIER_H_

#include <ed/perception/module.h>

// People detector
#include <PeopleDetector.h>


class LaserHumanChestClassifier : public ed::perception::Module
{
/*
 * ###########################################
 *  				PRIVATE
 * ###########################################
 */
private:

    // module configuration
    mutable bool init_success_;
    bool debug_mode_;            /*!< Enable debug mode */
    std::string	module_name_;    /*!< Name of the module, for output */
    std::string	module_path_;    /*!< Name of the module, for output */
    std::string debug_folder_;   /*!< Path of the debug folder */

    // Administration


    //People Detector object
    mutable PeopleDetector people_detector_;

    // Parameters
    double people_detector_threshold_;
    int people_detector_n_hypotheses_;
    double max_point_distance_;
    int list_size_;
    int min_n_points_;  // min_n_points_person
    double min_segment_size_;
    std::string hypotheses_filename_;


/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/
public:

    LaserHumanChestClassifier();

    virtual ~LaserHumanChestClassifier();

    void loadConfig(const std::string& config_path);

    void process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const;

    void configure(tue::Configuration config);


    // New interface

    void classify(const ed::Entity& e, const std::string& property, const ed::perception::CategoricalDistribution& prior,
                  ed::perception::ClassificationOutput& output) const {}

    void addTrainingInstance(const ed::Entity& e, const std::string& property, const std::string& value) {}

    void train() {}

    void loadRecognitionData(const std::string& path) {}

    void saveRecognitionData(const std::string& path) const {}

};

#endif
