#ifndef _ANNOTATED_IMAGE_H_
#define _ANNOTATED_IMAGE_H_

#include <ed/types.h>
#include <ed/world_model.h>
#include <rgbd/types.h>
#include <tue/config/data_pointer.h>

// ----------------------------------------------------------------------------------------------------

struct Annotation
{
    Annotation() {}
    Annotation(const std::string& label_, double px_, double py_)
        : label(label_), px(px_), py(py_), is_supporting(false) {}

    std::string label;
    double px;
    double py;
    bool is_supporting;
};

// ----------------------------------------------------------------------------------------------------

struct AnnotatedImage
{

    ed::WorldModel world_model;

    std::vector<ed::EntityConstPtr> entities;

    std::vector<Annotation> annotations;

    rgbd::ImagePtr image;

    geo::Pose3D sensor_pose;

    std::string area_description;

    tue::config::DataPointer meta_data;

};

// ----------------------------------------------------------------------------------------------------

bool fromFile(const std::string& filename, AnnotatedImage& image);

bool toFile(const std::string& filename, const AnnotatedImage& image);

void findAnnotationCorrespondences(const AnnotatedImage& img, std::vector<ed::EntityConstPtr>& correspondences);

// ----------------------------------------------------------------------------------------------------

#endif
