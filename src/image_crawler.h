#ifndef _IMAGE_CRAWLER_H_
#define _IMAGE_CRAWLER_H_

#include <rgbd/types.h>
#include <ed/kinect/entity_update.h>

#include "annotated_image.h"

// ----------------------------------------------------------------------------------------------------

class ImageCrawler
{

public:

    ImageCrawler();

    ~ImageCrawler();

    bool setPath(const std::string& path);

    bool previous(AnnotatedImage& image, bool do_segment = true);

    bool next(AnnotatedImage& image, bool do_segment = true);

    bool reload(AnnotatedImage& image, bool do_segment);

    const std::string& filename() const { return filenames_[i_current_]; }

    const std::vector<std::string>& filenames() const { return filenames_; }

    int index() const { return i_current_; }

private:

    int i_current_;

    std::vector<std::string> filenames_;

};

#endif
