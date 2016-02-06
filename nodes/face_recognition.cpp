#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

#include <geolib/datatypes.h>
#include <geolib/ros/msg_conversions.h>

#include <rgbd/serialization.h>
#include <fstream>

#include "face_recognizer.h"

#include "ed_perception/LearnPerson.h"
#include "ed_perception/RecognizePerson.h"

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>

FaceRecognizer fr;
rgbd::Client rgbd_client;

// ----------------------------------------------------------------------------------------------------

void visualizeResult(const rgbd::Image& image, const std::vector<FaceRecognitionResult>& detections = std::vector<FaceRecognitionResult>())
{
    cv::Mat canvas = image.getRGBImage().clone();

    for(std::vector<FaceRecognitionResult>::const_iterator it = detections.begin(); it != detections.end(); ++it)
    {
        cv::rectangle(canvas, it->rgb_roi, cv::Scalar(255, 0, 0), 2);
    }

    cv::imshow("face_recognition", canvas);
    cv::waitKey();
}

// ----------------------------------------------------------------------------------------------------

rgbd::ImagePtr loadImage(const std::string& filename)
{
    std::ifstream f_rgbd;
    f_rgbd.open(filename.c_str(), std::ifstream::binary);

    if (!f_rgbd.is_open())
    {
        std::cerr << "Could not open '" << filename << "'." << std::endl;
        return rgbd::ImagePtr();
    }

    rgbd::ImagePtr image(new rgbd::Image);

    tue::serialization::InputArchive a_in(f_rgbd);
    rgbd::deserialize(a_in, *image);

    return image;
}

// ----------------------------------------------------------------------------------------------------

rgbd::ImagePtr imageFromTopic()
{
    return rgbd_client.nextImage();
}

// ----------------------------------------------------------------------------------------------------

bool srvLearnPerson(ed_perception::LearnPerson::Request& req, ed_perception::LearnPerson::Response& res)
{
    std::cout << "Learn person " << req.person_name << std::endl;

    rgbd::ImagePtr image = rgbd_client.nextImage();
    if (!image)
    {
        res.error_msg = "Could not capture image";
        ROS_ERROR("%s", res.error_msg.c_str());
        return true;
    }

    fr.train(image->getRGBImage(), req.person_name);

//    visualizeResult(*image);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvRecognizePerson(ed_perception::RecognizePerson::Request& req, ed_perception::RecognizePerson::Response& res)
{
    std::cout << "Recognize person" << std::endl;

    rgbd::ImagePtr image = rgbd_client.nextImage();
    if (!image)
    {
        res.error_msg = "Could not capture image";
        ROS_ERROR("%s", res.error_msg.c_str());
        return true;
    }

    std::vector<FaceRecognitionResult> detections;
    fr.find(image->getRGBImage(), detections);

    res.person_detections.resize(detections.size());
    for(unsigned int i = 0; i < detections.size(); ++i)
    {
        const FaceRecognitionResult& det = detections[i];
        ed_perception::PersonDetection& det_msg = res.person_detections[i];

        det_msg.name = det.name;
        det_msg.name_score = det.name_score;

        if (det.gender == MALE)
            det_msg.gender = ed_perception::PersonDetection::MALE;
        else if (det.gender == FEMALE)
            det_msg.gender = ed_perception::PersonDetection::FEMALE;

        det_msg.age = det.age;

        geo::Pose3D pose = geo::Pose3D::identity();
        pose.t.z = 3;
        det_msg.pose.header.frame_id = image->getFrameId();
        geo::convert(pose, det_msg.pose.pose);
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{

    std::string image_filename;
    std::string config_filename;

    if (argc < 3)
    {
        std::cout << "Please provide --file or --config" << std::endl;
        return 1;
    }

    for(unsigned int i = 1 ; i + 1 < argc; i += 2)
    {
        std::string arg = argv[i];

        std::cout << arg << std::endl;

        if (arg[0] == '_')
            break;

        if (arg == "--file")
            image_filename = argv[i + 1];
        else if (arg == "--config")
            config_filename = argv[i + 1];
        else
        {
            std::cerr << "Unknown option: " << arg << std::endl;
            return 1;
        }
    }

    tue::Configuration config;

    if (!config_filename.empty())
    {
        tue::config::loadFromYAMLFile(config_filename, config);
        fr.configure(config);
    }

    if (!image_filename.empty())
    {
        rgbd::ImagePtr image = loadImage(image_filename);
        if (!image)
        {
            std::cerr << "Image could not be loaded" << std::endl;
            return 1;
        }

        fr.configure(config);

        if (config.hasError())
        {
            std::cerr << "Error in configuration: " << config.error().c_str() << std::endl;
            return 1;
        }

        std::vector<FaceRecognitionResult> detections;
        fr.find(image->getRGBImage(), detections);

        visualizeResult(*image, detections);
    }
    else
    {
        // No image filename given, so run as rosnode

        ros::init(argc, argv, "face_recognition");
        ros::NodeHandle nh;

        if (argc == 1)
        {
            ROS_ERROR("[FACE RECOGNITION] Please provide config file");
            return 1;
        }

        if (!config.hasError())
        {
            std::string topic;
            if (config.value("camera_topic", topic))
                rgbd_client.intialize(topic);

            fr.configure(config);
        }

        if (config.hasError())
        {
            ROS_ERROR("[FACE RECOGNITION] %s", config.error().c_str());
            return 1;
        }

        ros::ServiceServer srv_learn_person = nh.advertiseService("learn_person", srvLearnPerson);
        ros::ServiceServer srv_recognize_person = nh.advertiseService("recognize_person", srvRecognizePerson);

        ros::spin();
    }

    if (argc == 1 || std::string(argv[1]) != "--file")
    {


    }
    else
    {
        // Filename as argument, so run as test

        if (argc < 3)
        {
            std::cout << "Please provide rgbd filename" << std::endl;
            return 1;
        }

        std::string image_filename = argv[2];


    }

    return 0;
}
