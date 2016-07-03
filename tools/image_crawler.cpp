#include "image_crawler.h"

#include <tue/filesystem/crawler.h>
#include <ed/update_request.h>
#include <ed/kinect/updater.h>

//#include <ed/entity.h>
//#include <rgbd/View.h>
//#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

ImageCrawler::ImageCrawler()
{
}

// ----------------------------------------------------------------------------------------------------

ImageCrawler::~ImageCrawler()
{
}

// ----------------------------------------------------------------------------------------------------

bool ImageCrawler::setPath(const std::string& path_str)
{
    tue::filesystem::Path path(path_str);
    if (!path.exists())
    {
        std::cerr << "Path '" << path << "' does not exist." << std::endl;
        return false;
    }

    filenames_.clear();
    i_current_ = -1;

    tue::filesystem::Crawler crawler;

    if (!path.isDirectory())
    {
        filenames_.push_back(path_str);
        return true;
    }

    crawler.setRootPath(path);

    tue::filesystem::Path filename;
    while (crawler.nextPath(filename))
    {
        if (filename.extension() == ".json")
            filenames_.push_back(filename.string());
    }

    std::sort(filenames_.begin(), filenames_.end());
}

// ----------------------------------------------------------------------------------------------------

bool ImageCrawler::previous(AnnotatedImage& image, bool do_segment)
{
    if (filenames_.empty())
        return false;

    if (i_current_ <= 0)
        return false;

    bool res;
    do
    {
        if ( i_current_ <= 0)
            return false;

        --i_current_;

        res = reload(image, do_segment);
        std::cout << "i_current = " << i_current_ << std::endl;
        std::cout << "res = " << res << std::endl;
        std::cout << "image.excluded = " << image.excluded << std::endl;
    } while ( res && image.excluded );

    return res;
}

// ----------------------------------------------------------------------------------------------------

bool ImageCrawler::next(AnnotatedImage& image, bool do_segment)
{
    if (filenames_.empty())
        return false;

    bool res;
    do
    {
        if ( i_current_ + 1 == filenames_.size() )
            return false;

        ++i_current_;

        res = reload(image, do_segment);
        std::cout << "i_current = " << i_current_ << std::endl;
        std::cout << "res = " << res << std::endl;
        std::cout << "image.excluded = " << image.excluded << std::endl;
    } while ( res && image.excluded );

    return res;
}

// ----------------------------------------------------------------------------------------------------

bool ImageCrawler::reload(AnnotatedImage& image, bool do_segment)
{
    if (i_current_ < 0)
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Read image + meta-data

    fromFile(filenames_[i_current_], image);

    if (!do_segment)
        return true;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Segment

    ed::UpdateRequest update_req;
    UpdateResult res(update_req);

    Updater updater;
    UpdateRequest kinect_update_req;
    kinect_update_req.area_description = image.area_description;
    kinect_update_req.max_yaw_change = 0.5 * M_PI;
    updater.update(image.world_model, image.image, image.sensor_pose, kinect_update_req, res);

    image.world_model.update(update_req);

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//    // Visualize fitting

//    ed::EntityConstPtr support = image.world_model.getEntity("support");
//    if (support && support->shape())
//    {
//        cv::Mat depth_vis(480, 640, CV_32FC1, 0.0);
//        rgbd::View view(*image.image, depth_vis.cols);
//        view.getRasterizer().rasterize(*support->shape(), image.sensor_pose, support->pose(), depth_vis);

//        cv::imshow("depth", depth_vis / 10);
//    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Add entities

    for(ed::WorldModel::const_iterator it = image.world_model.begin(); it != image.world_model.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        image.entities.push_back(e);
    }

//    entity_updates_ = res.entity_updates;

    return true;
}

// ----------------------------------------------------------------------------------------------------
