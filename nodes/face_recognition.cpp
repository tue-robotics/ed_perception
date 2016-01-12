#include "rgbd/Client.h"
#include "rgbd/View.h"
#include <opencv2/highgui/highgui.hpp>

#include <rgbd/serialization.h>
#include <fstream>

#include "face_recognizer.h"

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

int main(int argc, char **argv)
{
    //ros::init(argc, argv, "face_recognition");
    //ros::NodeHandle nh;

    if (argc == 1)
    {
        std::cerr << "Please provide image file" << std::endl;
        return 1;
    }

    std::string image_filename = argv[1];

    rgbd::ImagePtr image = loadImage(image_filename);
    if (!image)
    {
        std::cerr << "Image could not be loaded" << std::endl;
        return 1;
    }

    cv::Mat canvas = image->getRGBImage().clone();

    FaceRecognizer fr;

    if (fr.train(image->getRGBImage(), "tim"))
    {
        cv::Rect roi;
        if (fr.find(image->getRGBImage(), "tim", &roi))
        {
            cv::rectangle(canvas, roi, cv::Scalar(255, 0, 0), 2);
        }
    }

    cv::imshow("face_recognition", canvas);
    cv::waitKey();

    return 0;
}
