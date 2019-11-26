/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: July 2015
*/

#ifndef HUMAN_CLASSIFIER_H_
#define HUMAN_CLASSIFIER_H_

// OpenCV includes
#include <opencv/cv.h>
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

// STL includes
#include <stdlib.h>
#include <cstring>
#include <stdio.h>
#include <math.h>
#include <numeric>
#include <vector>
#include <fstream>
#include <iostream>
#include <boost/lexical_cast.hpp>

// Human_Classifier includes
#include "human_classifier_structs.h"

//ED includes
#include "ed/measurement.h"


class HumanClassifier {

	/*
	 * ###########################################
	 *  				PRIVATE
	 * ###########################################
	 */
	private:         

        // ##### DEBUG VARIABLES #####
        bool debug_mode_;            /*!< Enable debug mode */
        std::string	module_name_;    /*!< Name of the module, for output */
        std::string debug_folder_;   /*!< Path of the debug folder */

        // ##### SETUP VARIABLES #####
        std::string	model_name_;
        bool	face_detect_enabled_; /*!< Enable face detection*/
        int 	border_size_;        /*!< Size in pixels for the border added to the DT map */
        int		match_iterations_;   /*!< Number of iteration for the Perfect Match algorithm */
        int		dt_line_width_;       /*!< Contour line width */
        double	max_template_err_;    /*!< Template Matching max error to classify a measure as Human */

        // ##### TEMPLATE MATCHING VARIABLES #####
        std::vector<std::vector<cv::Point> > kTemplatesOriginal;    /*!< Original templates, without scaling */
        cv::Mat morph_element_;      /*!< Morphologic operations structural element */
        int num_slices_matching_;     /*!< Number of slices to calculate best match initial position */

        std::string cascade_path_;


	/*
	 * ###########################################
	 *  				PUBLIC
	 * ###########################################
	 */
	public:

        // Constructors
        HumanClassifier() {}
        HumanClassifier(const std::string& module_name);

		// Default destructor
		virtual ~HumanClassifier();

        // Method that classifies a new candidate as human or not human
        bool Classify(const cv::Mat& depth_img,
                      const cv::Mat& color_img,
                      const cv::Mat& mask,
                      float& avg_depth,
                      float& template_match_error,
                      float &template_match_deviation,
                      std::string &template_stance) const;

        // Tries to fit a template on the given measurement
        bool TemplateClassification(const cv::Mat& depth_image,
                                    const cv::Mat& mask,
                                    float& avg_depth,
                                    cv::Point3i& match_pos,
                                    cv::Point3i& match_init_pos,
                                    float& match_error,
                                    float& match_variance,
                                    float& match_deviation,
                                    TemplateType &template_type,
                                    Roi &measurement) const;

        // Perfect Match algorithm used to fit the template
        bool PerfectMatch(const cv::Mat& mask,
                          cv::Mat& map_dt,
                          std::vector<cv::Point> template_pts,
                          cv::Point3i &best_pos,
                          cv::Point3i& start_pos,
                          float &best_error,
                          float& variance,
                          float& deviation) const;

        // function used to calculate the fitting error on the given position
        float ErrorFunction(const cv::Mat& gradX,
                            const cv::Mat& gradY,
                            const cv::Mat& imageDT,
                            std::vector<cv::Point>& template_pts,
                            cv::Vec3f& anchor,
                            cv::Vec3f& grad,
                            std::vector<float>& errValues) const;

        // tries to detect a face on the area where the template fit
        bool FaceDetection(const cv::Mat &color_img, Roi &measurement) const;

        // enlarge the area used to search for a face
        void OptimizeHeadRoi(cv::Rect& templateArea,
                             TemplateType templateN,
                             const cv::Mat &color_img,
                             cv::Rect& headArea) const;

        // Resizes a template according to a certain depth
        float ResizeTemplate(std::vector<cv::Point> src_template,
                             float depth,
                             float alpha,
                             TemplateType type,
                             std::vector<cv::Point>& dst_template) const;

        // clips a integer number between a min and a max
        int ClipInt(int val, int min, int max) const;

        // Global variable initializations, templates and face cascade training files loading
        bool Initializations(bool debug_mode,
                             const std::string debug_folder,
                             const std::string template_front_path,
                             const std::string template_left_path,
                             const std::string template_right_path,
                             int match_iterations,
                             int dt_line_width,
                             double max_template_err,
                             int border_size,
                             int num_slices_matching);

        // Loads a template with the given name and converts it into 2D points
        bool LoadTemplate(const std::string& template_path, std::vector<std::vector<cv::Point> >& template_list);

        // Clean all files inside the debug folder
		void CleanDebugFolder(const std::string& folder);

        // Generates a random string ID
        std::string GenerateID() const;

	};

#endif
