/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: January 2015
*/

#ifndef ED_PERCEPTION_OPEN_BR_ED_H_
#define ED_PERCEPTION_OPEN_BR_ED_H_

#include <ed/perception/module.h>

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"

class OpenBrEd : public ed::perception::Module
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


    // open biometrics
//    QSharedPointer<br::Transform> br_age_estimation;
//    QSharedPointer<br::Transform> br_gender_estimation;
//    QSharedPointer<br::Transform> br_face_rec;
//    QSharedPointer<br::Distance> br_face_rec_dist;

    void showDebugWindow(cv::Mat face_img,
                         std::string name,
                         double name_confidence,
                         std::string gender,
                         double gender_confidence,
                         int age,
                         double age_confidence) const;

/*
* ###########################################
*  				    PUBLIC
* ###########################################
*/
public:

    OpenBrEd();

    virtual ~OpenBrEd();

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
