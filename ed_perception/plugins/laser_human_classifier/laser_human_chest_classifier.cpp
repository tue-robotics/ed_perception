/*
* Author: Luis Fererira
* E-mail: luisfferreira@outlook.com
* Date: March 2015
*/

#include "laser_human_chest_classifier.h"

#include "ed/measurement.h"
#include <ed/entity.h>
#include <ed/error_context.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <boost/filesystem.hpp>
#include <iostream>

#include <sensor_msgs/LaserScan.h>


// ----------------------------------------------------------------------------------------------------

LaserHumanChestClassifier::LaserHumanChestClassifier() :
    ed::perception::Module("laser_human_chest_classifier"),
    init_success_(false)
{

}


// ----------------------------------------------------------------------------------------------------


LaserHumanChestClassifier::~LaserHumanChestClassifier()
{
}


// ----------------------------------------------------------------------------------------------------

void LaserHumanChestClassifier::configure(tue::Configuration config) {

    bool file_loaded = true;
    FILE *f_hypotheses;

    if (!config.value("hypotheses_filename", hypotheses_filename_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'hypotheses_filename' not found. Using default: " << hypotheses_filename_ << std::endl;
    else
        hypotheses_filename_ = module_path_ + hypotheses_filename_;

    if (!config.value("num_hypotheses", people_detector_n_hypotheses_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'num_hypotheses' not found. Using default: " << people_detector_n_hypotheses_ << std::endl;

    if (!config.value("people_detector_threshold", people_detector_threshold_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'people_detector_threshold' not found. Using default: " << people_detector_threshold_ << std::endl;

    if (!config.value("max_point_distance", max_point_distance_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'max_point_distance' not found. Using default: " << max_point_distance_ << std::endl;

    if (!config.value("list_size", list_size_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'list_size' not found. Using default: " << list_size_ << std::endl;

    if (!config.value("min_segment_size", min_segment_size_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'min_segment_size' not found. Using default: " << min_segment_size_ << std::endl;

    if (!config.value("min_n_points", min_n_points_, tue::OPTIONAL))
        std::cout << "[" << module_name_ << "] " << "Parameter 'min_n_points' not found. Using default: " << min_n_points_ << std::endl;


    // try to load the hypothesis file
    try {
        f_hypotheses = fopen(hypotheses_filename_.c_str(), "r");
        if (f_hypotheses == NULL){
            throw -1;
        }
    } catch (int e) {
        std::cout << "[" << module_name_ << "] " << "Could not load hypothesis file! (" << hypotheses_filename_ << ")" << std::endl;
        file_loaded = false;
    }

    // initialize PeopleDetector
    if (file_loaded){
        std::cout << "[" << module_name_ << "] " << "Initializing PeopleDetector..." << std::endl;
        people_detector_.load(f_hypotheses, people_detector_n_hypotheses_);
        init_success_ = true;
        std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
    }

    init_success_ = false;   // JUST FOR NOW!
}


// ----------------------------------------------------------------------------------------------------


void LaserHumanChestClassifier::loadConfig(const std::string& config_path) {

    // module configuration
    module_name_ = "laser_human_detector";
    module_path_ = config_path;

    // default values for the parameters
    people_detector_threshold_ = 0.05;
    max_point_distance_ = 10.0;
    hypotheses_filename_ = config_path + "/training_files/hypo_with_arms_full.dat";
    people_detector_n_hypotheses_ = 100;
    list_size_ = 500;
    min_n_points_ = 5;  // min_n_points_person
    min_segment_size_ = 0.2;
}


// ----------------------------------------------------------------------------------------------------


void LaserHumanChestClassifier::process(const ed::perception::WorkerInput& input, ed::perception::WorkerOutput& output) const{
    ed::ErrorContext errc("Processing entity in LaserHumanDetector");

    if (!init_success_)
        return;

    const ed::EntityConstPtr& e = input.entity;
    tue::Configuration& config = output.data;

    // ---------- Prepare measurement ----------

    sensor_msgs::LaserScan::ConstPtr msg;

//    double t_start = ros::Time::now().toSec();

    // This is a list of segments.
    // For more information have a look at ../common/dynamictable.h/hxx
    dyntab_segments *list_segments=NULL;
    int num_readings = msg->ranges.size();
    double *angles = new double[num_readings]; // array of angles
    double *ranges = new double[num_readings]; // array of measurements

    // Get ranges and angles from msg
//    double t0 = ros::Time::now().toSec();
    for (int i = 0; i < num_readings; i++) {
        // Only consider 'nearby' points in the cloud (angles are not used during segmentation)
        if (msg->ranges[i] > max_point_distance_) ranges[i] = -10.0;
        else ranges[i] = msg->ranges[i];
        angles[i] = msg->angle_min + i * msg->angle_increment;

    }

    list_segments = new dyntab_segments (list_size_);
//    ROS_DEBUG("Looping over all segments takes %f [ms]", 1000*(ros::Time::now().toSec()-t0));


    // Segment the scan data
//    double t1 = ros::Time::now().toSec();
    people_detector_.segmentScan(people_detector_threshold_, num_readings, angles, ranges, list_segments);
//    ROS_DEBUG("Segmentation takes %f [ms]", 1000*(ros::Time::now().toSec()-t1));
//    ROS_DEBUG("Found %d segments", list_segments->num());

    // Message to WIRE
//    vector<wire::Evidence> ev_vector;
//    pbl::PMF class_pmf;
//    class_pmf.setProbability("person", 1.0);

    // Iterate over all segments
    int index_laser_point = 0;
//    double t2 = ros::Time::now().toSec();

    // ----------------------- Process -----------------------

    std::vector<cv::Point> people_locations;

    for (int i=0; i < list_segments->num(); i++)
    {

        // Get segments
        Segment *s = list_segments->getElement(i);

        // CHECK 1: number of laser points
        if ( s->num() < min_n_points_ )
        {
            s->type = -1;
            index_laser_point += s->num();
        }
        else
        {
            // Calculate size of the segment (exclude possible shadow points)
            int i_min = index_laser_point;
            int i_max = index_laser_point + s->num();
            double x_min = ranges[i_min]*cos(angles[i_min]);
            double x_max = ranges[i_max]*cos(angles[i_max]);
            double y_min = ranges[i_min]*sin(angles[i_min]);
            double y_max = ranges[i_max]*sin(angles[i_max]);
            double size_segment = sqrt((x_max-x_min)*(x_max-x_min) + (y_max-y_min)*(y_max-y_min));

            // Determine position of the segment
            double x_pos = 0, y_pos = 0;
            for (int out = 0; out < s->num(); out++) {
                if (out == static_cast<int>(s->num()/2)) {
                    x_pos = ranges[index_laser_point]*cos(angles[index_laser_point]);
                    y_pos = ranges[index_laser_point]*sin(angles[index_laser_point]);
                }
                ++index_laser_point;
            }

            // CHECK 2: size of the segment
            if (size_segment > min_segment_size_){

                // Classify current segment
                people_detector_.classify(s);

                // IF classified as being a person
                if (s->type == 1)
                {
//                    ROS_DEBUG("Segment is classified as being a person!");

                    // Person position equals average position of the segment points
//                    double HARDCODED_Z_OFFSET = -0.57; // position of a person is above the torso laser (upside down, hence negative number)
//                    wire::Evidence ev(msg->header.stamp.toSec());
//                    ev.addProperty("class_label", class_pmf);
//                    pbl::Matrix cov(3, 3);
//                    cov.zeros();
//                    cov(0,0) = 0.03; // TODO: Tune covariance
//                    cov(1,1) = 0.03;
//                    cov(2,2) = 0.03;
//                    ev.addProperty("position", pbl::Gaussian(pbl::Vector3(x_pos, y_pos, HARDCODED_Z_OFFSET), cov), msg->header.frame_id);
//                    ev_vector.push_back(ev);
//                    ROS_DEBUG("Position person is (%f,%f), #points is %d, size is %f [m]", x_pos, y_pos, s->num(), size_segment);
                    people_locations.push_back(cv::Point(x_pos, y_pos));
                    std::cout << "[" << module_name_ << "] " << "Person found at (" << x_pos << "," << y_pos << "), #point"
                                 << s->num() << " and size " << size_segment << std::endl;

                }
                // Update index in the segment is not a person
                else
                {
//                    ROS_DEBUG("Size is OK but not classified as being a person");
                    index_laser_point += s->num();
                }
            }
            // Update index if the size check fails
            else{
//                if (size_segment <= min_segment_size_) ROS_DEBUG("Not a person: too short (%f [m])", size_segment);
//                else if (size_segment >= 1.5) ROS_DEBUG("Not a person: too large (%f [m])", size_segment);
//                else ROS_DEBUG("Invalid range: r_min = %f and r_max = %f", ranges[i_min], ranges[i_max]);
                index_laser_point += s->num();
            }
        }
    }

//    ROS_DEBUG("Looping over all segments takes %f [ms]", 1000*(ros::Time::now().toSec()-t2));
//    ROS_DEBUG("Number of laser points checked %d/%d", index_laser_point, num_readings);

    // Publish results
//    client_->assertEvidence(ev_vector);

//    ROS_INFO("Full ppl_detection took %f [ms] for %zu ppl", 1000*(ros::Time::now().toSec()-t_start), ev_vector.size());

    // ----------------------- Assert results -----------------------

    // create group if it doesnt exist
    if (!config.readGroup("perception_result", tue::OPTIONAL))
    {
        config.writeGroup("perception_result");
    }

    config.writeGroup("laser_human_chest_classifier");

    if (people_locations.size() > 0){
        config.writeArray("detections");
        for (uint j = 0; j < people_locations.size(); j++){
            config.addArrayItem();
            config.setValue("x", people_locations[j].x);
            config.setValue("y", people_locations[j].y);
            config.endArrayItem();
        }
        config.endArray();
    }

    config.endGroup();  // close face_recognizer group
    config.endGroup();  // close perception_result group


    // Delete the list of segments
    list_segments->setAutoDelete(true);
    delete list_segments;
    list_segments = NULL;

    // Free memory
    delete [] ranges;
    delete [] angles;
}


// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PERCEPTION_MODULE(LaserHumanChestClassifier)
