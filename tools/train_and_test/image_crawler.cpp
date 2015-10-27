#include "image_crawler.h"

#include <rgbd/Image.h>
#include <rgbd/serialization.h>
#include <rgbd/View.h>
#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/serialization/input_archive.h>
#include <tue/filesystem/crawler.h>
#include <ed/serialization/serialization.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/models/model_loader.h>
#include <ed/entity.h>
#include <ed/kinect/updater.h>
#include <fstream>

// ----------------------------------------------------------------------------------------------------

namespace
{

bool readImage(const std::string& filename, rgbd::ImagePtr& image, geo::Pose3D& sensor_pose,
               ed::WorldModel& world_model, std::string& area_description, std::vector<Annotation>& annotations)
{
    tue::config::DataPointer meta_data;

    try
    {
        meta_data = tue::config::fromFile(filename);
    }
    catch (tue::config::ParseException& e)
    {
        std::cerr << "Could not open '" << filename << "'.\n\n" << e.what() << std::endl;
        return false;
    }

    tue::config::Reader r(meta_data);

    // Read image
    std::string rgbd_filename;
    if (r.value("rgbd_filename", rgbd_filename))
    {
        tue::filesystem::Path abs_rgbd_filename = tue::filesystem::Path(filename).parentPath().join(rgbd_filename);

        std::ifstream f_rgbd;
        f_rgbd.open(abs_rgbd_filename.string().c_str(), std::ifstream::binary);

        if (!f_rgbd.is_open())
        {
            std::cerr << "Could not open '" << filename << "'." << std::endl;
            return false;
        }

        image.reset(new rgbd::Image);

        tue::serialization::InputArchive a_in(f_rgbd);
        rgbd::deserialize(a_in, *image);
    }

    // Read sensor pose
    if (!ed::deserialize(r, "sensor_pose", sensor_pose))
    {
        std::cerr << "No field 'sensor_pose' specified." << std::endl;
        return false;
    }

    // Reset world
    world_model = ed::WorldModel();

    annotations.clear();

    // Read annotations
    if (r.readArray("annotations"))
    {
        while(r.nextArrayItem())
        {
            std::string type;
            double px, py;

            if (!r.value("label", type) || !r.value("px", px) || !r.value("py", py))
                continue;

            // - - - - - - -

            ed::UpdateRequest req;
            ed::models::ModelLoader model_loader;

            std::stringstream error;
            ed::UUID id = "support";

            bool on_top_of_found = false;
            if (model_loader.create(id, type, req, error))
            {
                // Check if this model has an 'on_top_of' area defined
                if (!req.datas.empty())
                {
                    tue::config::Reader r(req.datas.begin()->second);

                    if (r.readArray("areas"))
                    {
                        while(r.nextArrayItem())
                        {
                            std::string a_name;
                            if (r.value("name", a_name) && a_name == "on_top_of")
                            {
                                on_top_of_found = true;
                                break;
                            }
                        }
                    }
                }
            }

            if (on_top_of_found)
            {

                int x = px * image->getDepthImage().cols;
                int y = py * image->getDepthImage().rows;
                rgbd::View view(*image, image->getDepthImage().cols);

                geo::Vec3 pos = sensor_pose * (view.getRasterizer().project2Dto3D(x, y) * 3);
                pos.z = 0;

                req.setPose(id, geo::Pose3D(geo::Mat3::identity(), pos));

                // Update world
                world_model.update(req);

                area_description = "on_top_of " + id.str();
            }
            else
            {
                annotations.push_back(Annotation(type, px, py));
            }


            // - - - - - - -

        }

        r.endArray();
    }

//    if (r.hasError())
//    {
//        std::cout << "Error while reading file '" << filename << "':\n\n" << r.error() << std::endl;
//        return false;
//    }

    return true;
}

}

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
}

// ----------------------------------------------------------------------------------------------------

bool ImageCrawler::previous()
{
    if (filenames_.empty())
        return false;

    if (i_current_ <= 0)
        return false;

    --i_current_;
    return reload();
}

// ----------------------------------------------------------------------------------------------------

bool ImageCrawler::next()
{
    if (filenames_.empty())
        return false;

    if (i_current_ + 1 == filenames_.size())
        return false;

    ++i_current_;
    return reload();
}

// ----------------------------------------------------------------------------------------------------

bool ImageCrawler::reload()
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Clear current data

    image_.reset();
    entities_.clear();
    entity_updates_.clear();

    if (i_current_ < 0)
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Read image + meta-data

    geo::Pose3D sensor_pose;
    ed::WorldModel world_model;
    std::string area_description;

    if (!readImage(filenames_[i_current_], image_, sensor_pose, world_model, area_description, annotations_))
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Segment

    ed::UpdateRequest update_req;
    UpdateResult res(update_req);

    Updater updater;
    updater.update(world_model, image_, sensor_pose, area_description, res);

    world_model.update(update_req);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Add entities

    for(ed::WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        entities_.push_back(e);
    }

    entity_updates_ = res.entity_updates;
}

// ----------------------------------------------------------------------------------------------------
