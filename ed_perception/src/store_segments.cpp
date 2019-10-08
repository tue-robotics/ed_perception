#include "image_crawler.h"

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
    if (argc != 3)
    {
        usage();
        return 1;
    }

    ImageCrawler crawler;
    crawler.setPath(argv[1]);

    boost::filesystem::path target_path = argv[2];

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

            // Check if path exists and create directories if necessary
            boost::filesystem::path p = target_path / boost::filesystem::path(a.label);

            if ( !boost::filesystem::exists(p) )
                boost::filesystem::create_directories(p);

            boost::filesystem::path temp_filename = rgbd_filename.filename().replace_extension(".jpg");

            // Check if filename already exists in this directory.
            if ( boost::filesystem::exists( (p / temp_filename ) ) )
            {
                // If it does, add a number to the filename
                int i = 0;

                do
                {
                    i++;
                    temp_filename = rgbd_filename.filename().replace_extension("");    // Remove extension
                    std::cout << temp_filename.c_str() << std::endl;
                    std::stringstream integer;                              // Convert int to string type
                    integer << i;
                    temp_filename += integer.str();                         // Add number
                    temp_filename += ".jpg";                                // put extension back
                } while ( boost::filesystem::exists( p / temp_filename ) );
            }

            p = p / temp_filename;

            std::cout << "writing to " << p.c_str() << std::endl;
            cv::imwrite( p.c_str(), ROI );
        }
    }

    return 0;
}
