#include "image_crawler.h"
#include "ed/perception/aggregator.h"

#include <rgbd/Image.h>

#include <ed/entity.h>
#include <ed/measurement.h>

#include <tue/config/reader.h>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: store_segments SOURCE-IMAGE-FILE-OR-DIRECTORY TARGET-DIRECTORY" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        usage();
        return 1;
    }

//    std::string location = argv[2];
//    std::cout << location << std::endl;

    ImageCrawler crawler;
    crawler.setPath(argv[1]);

    AnnotatedImage image;

    while(crawler.next(image))
    {
        std::vector<cv::Rect> ROIs;
        std::vector<ed::EntityConstPtr> correspondences;
        findAnnotatedROIs(image, correspondences, ROIs);

        boost::filesystem::path rgbd_filename = crawler.filename();

        for(unsigned int i = 0; i < correspondences.size(); ++i)
        {
            const cv::Rect bbox = ROIs[i];
            const Annotation& a = image.annotations[i];
            if (a.is_supporting || !correspondences[i] )
                continue;

            cv::Mat ROI = image.image->getRGBImage()(bbox);

            // Check if path exists and store segment in directory
            boost::filesystem::path p(getenv("HOME"));

            p += boost::filesystem::path("/training_images/");
            p += boost::filesystem::path(a.label);
            p += "/";

            if ( !boost::filesystem::exists(p) )
                boost::filesystem::create_directories(p);

            p += rgbd_filename.filename();

            p.replace_extension(boost::filesystem::path(".jpg"));

            std::cout << "writing to " << p.c_str() << std::endl;
            cv::imwrite( p.c_str(), ROI );
        }
    }

    return 0;
}
