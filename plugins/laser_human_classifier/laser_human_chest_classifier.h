/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: March 2015
*/

#ifndef LASER_HUMAN_CHEST_CLASSIFIER_H_
#define LASER_HUMAN_CHEST_CLASSIFIER_H_

#include <ed/perception_modules/perception_module.h>

// People detector
#include <PeopleDetector.h>


class LaserHumanChestClassifier : public ed::PerceptionModule{
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
    std::string hypotheses_filename_;
    FILE *f_hypotheses;

    //People Detector object
    PeopleDetector pd;

    // Parameters
    int num_hypotheses_;


/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/
public:

    LaserHumanChestClassifier();

    virtual ~LaserHumanChestClassifier();

    void loadConfig(const std::string& config_path);

    void process(ed::EntityConstPtr e, tue::Configuration& result) const;

    void configure(tue::Configuration config);

};

#endif
