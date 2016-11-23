#include "annotated_image.h"

#include <rgbd/Image.h>
#include <rgbd/serialization.h>
#include <rgbd/View.h>
#include <tue/config/read.h>
#include <tue/config/reader.h>
#include <tue/config/write.h>
#include <tue/config/writer.h>
#include <tue/serialization/input_archive.h>
#include <tue/filesystem/crawler.h>
#include <ed/serialization/serialization.h>
#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/models/model_loader.h>
#include <ed/entity.h>
#include <ed/measurement.h>
#include <ed/kinect/updater.h>
#include <fstream>

#include <tf/transform_datatypes.h>
#include <geolib/ros/tf_conversions.h>

// ----------------------------------------------------------------------------------------------------

// Decomposes 'pose' into a (X, Y, YAW) and (Z, ROLL, PITCH) component
void decomposePose(const geo::Pose3D& pose, geo::Pose3D& pose_xya, geo::Pose3D& pose_zrp)
{
    tf::Matrix3x3 m;
    geo::convert(pose.R, m);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose_xya.R.setRPY(0, 0, yaw);
    pose_xya.t = geo::Vec3(pose.t.x, pose.t.y, 0);

    pose_zrp = pose_xya.inverse() * pose;
}

// ----------------------------------------------------------------------------------------------------

bool fromFile(const std::string& filename, AnnotatedImage& image)
{
    try
    {
        image.meta_data = tue::config::fromFile(filename);
    }
    catch (tue::config::ParseException& e)
    {
        std::cerr << "Could not open '" << filename << "'.\n\n" << e.what() << std::endl;
        return false;
    }

    // Clear image
    image.image.reset();
    image.entities.clear();

    tue::config::Reader r(image.meta_data);

    // Read image
    std::string rgbd_filename;
    if (r.value("rgbd_filename", rgbd_filename))
    {
        tue::filesystem::Path abs_rgbd_filename = tue::filesystem::Path(filename).parentPath().join(rgbd_filename);

        std::ifstream f_rgbd;
        f_rgbd.open(abs_rgbd_filename.string().c_str(), std::ifstream::binary);

        if (!f_rgbd.is_open())
        {
            std::cerr << "Could not open '" << abs_rgbd_filename << "'." << std::endl;
            return false;
        }

        image.image.reset(new rgbd::Image);

        tue::serialization::InputArchive a_in(f_rgbd);
        rgbd::deserialize(a_in, *image.image);
    }

    // Read sensor pose
    if (!ed::deserialize(r, "sensor_pose", image.sensor_pose))
    {
        std::cerr << "No field 'sensor_pose' specified." << std::endl;
        return false;
    }

    // Reset world
    image.world_model = ed::WorldModel();

    image.annotations.clear();

    if (!r.value("area", image.area_name, tue::config::OPTIONAL))
        image.area_name = "on_top_of";

    image.excluded = false;
    if (!r.value("excluded", image.excluded, tue::config::OPTIONAL))
        image.excluded = false;

    // Read annotations
    if (r.readArray("annotations"))
    {
        while(r.nextArrayItem())
        {
            std::string type;
            double px, py;

            if (!r.value("label", type) || !r.value("px", px) || !r.value("py", py))
                continue;

            image.annotations.push_back(Annotation(type, px, py));
            Annotation& ann = image.annotations.back();

            // - - - - - - -

            ed::UpdateRequest req;
            ed::models::ModelLoader model_loader;

            std::stringstream error;
            ed::UUID id = "support";

            bool area_found = false;
            if (model_loader.create(id, type, req, error))
            {
                // Check if this model has an the given area
                if (!req.datas.empty())
                {
                    tue::config::Reader r(req.datas.begin()->second);

                    if (r.readArray("areas"))
                    {
                        while(r.nextArrayItem())
                        {
                            std::string a_name;
                            if (r.value("name", a_name) && a_name == image.area_name)
                            {
                                area_found = true;
                                break;
                            }
                        }
                    }
                }
            }

            if (area_found)
            {
                int x = px * image.image->getDepthImage().cols;
                int y = py * image.image->getDepthImage().rows;
                rgbd::View view(*image.image, image.image->getDepthImage().cols);

                // Decompose the sensor_pose into (x, y, yaw) and (z, roll, pitch)
                geo::Pose3D sensor_pose_xya, sensor_pose_zrp;
                decomposePose(image.sensor_pose, sensor_pose_xya, sensor_pose_zrp);

                // Estimate based on the pixel of the entity annotation where it is w.r.t.
                // the sensor
                geo::Pose3D pose_SENSOR_XYA;
                pose_SENSOR_XYA.t = geo::Vec3(view.getRasterizer().project2Dto3DX(x), 1, 0);
                pose_SENSOR_XYA.R.setRPY(0, 0, -0.5 * M_PI); // This assumes estimated entity position is with its x-axis towards camera

                // Calculate the entity pose in map frame
                geo::Pose3D pose_MAP = sensor_pose_xya * pose_SENSOR_XYA;
                pose_MAP.t.z = 0;

                req.setPose(id, pose_MAP);

                // Update world
                image.world_model.update(req);

                image.area_description = image.area_name + " " + id.str();

                ann.is_supporting = true;
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

// ----------------------------------------------------------------------------------------------------

bool toFile(const std::string& filename, const AnnotatedImage& image)
{
    // Temporarily make new data containing the annotations. If we would directly add to the existing meta-data
    // we would add the existing annotations *again*. Now, we will completely overwrite them.
    tue::config::DataPointer data;

    tue::config::Writer w(data);

    w.writeArray("annotations");
    for (std::vector<Annotation>::const_iterator it = image.annotations.begin(); it != image.annotations.end(); ++it)
    {
        w.addArrayItem();
        const Annotation& a = *it;
        w.setValue("label", a.label);
        w.setValue("px", a.px);
        w.setValue("py", a.py);
        w.endArrayItem();
    }
    w.endArray();

    w.setValue("area", image.area_name);
    std::cout << "Writing area: " << image.area_name << std::endl;

    w.setValue("excluded", image.excluded);
    std::cout << "Excluding image" << std::endl;

    // Add the new annotations to the existing meta data (complete overwriting the annotations array)

    tue::config::DataPointer meta_data;
    meta_data.add(image.meta_data);
    meta_data.add(data);

    tue::config::toFile(filename, meta_data, tue::config::JSON, 0);
}

// ----------------------------------------------------------------------------------------------------

void findAnnotatedROIs(const AnnotatedImage& img, std::vector<ed::EntityConstPtr>& correspondences, std::vector<cv::Rect>& entity_rects)
{
    findAnnotationCorrespondences(img, correspondences);

    const cv::Mat& rgb = img.image->getRGBImage();

    entity_rects.resize(correspondences.size());

    for(unsigned int i = 0; i < correspondences.size(); ++i)
    {
        if ( !correspondences[i] )
        {
            continue;
        }

        const ed::EntityConstPtr& e = correspondences[i];

        const ed::ImageMask& mask = e->bestMeasurement()->imageMask();

        cv::Point p_min(rgb.cols, rgb.rows);
        cv::Point p_max(0, 0);

        for(ed::ImageMask::const_iterator it = mask.begin(rgb.cols); it != mask.end(); ++it)
        {
            const cv::Point2i& p = *it;
            p_min.x = std::min(p_min.x, p.x);
            p_min.y = std::min(p_min.y, p.y);
            p_max.x = std::max(p_max.x, p.x);
            p_max.y = std::max(p_max.y, p.y);
        }

        entity_rects[i] = cv::Rect(std::min(p_min.x+5, rgb.cols),
                                   std::min(p_min.y+5, rgb.rows),
                                   std::max(p_max.x - p_min.x - 5, 0),
                                   std::max(p_max.y - p_min.y - 5, 0));
    }
}

// ----------------------------------------------------------------------------------------------------

// Finds which annotations belong to which entities in AnnotatedImage 'image'. Returns a vector which
// is as long as image.annotations, and has for each index the corresponding entity, or a nullptr if
// none could be found
void findAnnotationCorrespondences(const AnnotatedImage& img, std::vector<ed::EntityConstPtr>& correspondences)
{
    const cv::Mat& depth = img.image->getDepthImage();

    std::vector<cv::Rect> entity_rects(img.entities.size());

    for(unsigned int i = 0; i < img.entities.size(); ++i)
    {
        const ed::EntityConstPtr& e = img.entities[i];
        if (!e->has_pose() || e->shape() || !e->bestMeasurement())
        {
            entity_rects[i].x = -1; // flag that this entity is not to be associated
            continue;
        }

        const ed::ImageMask& mask = e->bestMeasurement()->imageMask();

        cv::Point p_min(depth.cols, depth.rows);
        cv::Point p_max(0, 0);

        for(ed::ImageMask::const_iterator it = mask.begin(depth.cols); it != mask.end(); ++it)
        {
            const cv::Point2i& p = *it;
            p_min.x = std::min(p_min.x, p.x);
            p_min.y = std::min(p_min.y, p.y);
            p_max.x = std::max(p_max.x, p.x);
            p_max.y = std::max(p_max.y, p.y);
        }

        entity_rects[i] = cv::Rect(p_min.x, p_min.y, p_max.x - p_min.x, p_max.y - p_min.y);
    }

    correspondences.resize(img.annotations.size());
    for(unsigned int i = 0; i < img.annotations.size(); ++i)
    {
        const Annotation& a = img.annotations[i];
        if (a.is_supporting)
            continue;

        cv::Point p_2d(a.px * depth.cols, a.py * depth.rows);

        for(unsigned int j = 0; j < entity_rects.size(); ++j)
        {
            const cv::Rect& rect = entity_rects[j];
            if (rect.x < 0)
                continue;

            if (rect.contains(p_2d))
            {
                correspondences[i] = img.entities[j];
                break;
            }
        }
    }
}


