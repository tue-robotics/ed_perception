#ifndef _IMAGE_CRAWLER_H_
#define _IMAGE_CRAWLER_H_

#include <rgbd/types.h>
#include <ed/kinect/entity_update.h>

// ----------------------------------------------------------------------------------------------------

struct Annotation
{
    Annotation(const std::string& label_, double px_, double py_)
        : label(label_), px(px_), py(py_) {}

    std::string label;
    double px;
    double py;
};

// ----------------------------------------------------------------------------------------------------

class ImageCrawler
{

public:

    ImageCrawler();

    ~ImageCrawler();

    bool setPath(const std::string& path);

    bool previous();

    bool next();

    bool reload();

    const std::string& filename() const { return filenames_[i_current_]; }

    rgbd::ImageConstPtr image() const { return image_; }

    const std::vector<ed::EntityConstPtr>& entities() const { return entities_; }

    const std::vector<EntityUpdate>& entity_updates() const { return entity_updates_; }

    const std::vector<Annotation>& annotations() const { return annotations_; }

private:

    int i_current_;

    std::vector<std::string> filenames_;

    std::vector<ed::EntityConstPtr> entities_;

    std::vector<EntityUpdate> entity_updates_;

    std::vector<Annotation> annotations_;

    rgbd::ImagePtr image_;

};

#endif
