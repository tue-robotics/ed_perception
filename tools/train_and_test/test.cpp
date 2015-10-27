#include "image_crawler.h"
#include "ed/perception/aggregator.h"

#include <rgbd/Image.h>

#include <ed/entity.h>
#include <ed/measurement.h>

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: test-perception CONFIG-FILE IMAGE-FILE-OR-DIRECTORY" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void correspond(const rgbd::Image& image, const std::vector<Annotation>& annotations,
                const std::vector<ed::EntityConstPtr>& entities, std::vector<ed::EntityConstPtr>& correspondences)
{
    const cv::Mat& depth = image.getDepthImage();

    std::vector<cv::Rect> entity_rects(entities.size());

    for(unsigned int i = 0; i < entities.size(); ++i)
    {
        const ed::EntityConstPtr& e = entities[i];
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

    correspondences.resize(annotations.size());
    for(unsigned int i = 0; i < annotations.size(); ++i)
    {
        const Annotation& a = annotations[i];
        cv::Point p_2d(a.px * depth.cols, a.py * depth.rows);

        for(unsigned int j = 0; j < entity_rects.size(); ++j)
        {
            const cv::Rect& rect = entity_rects[j];
            if (rect.x < 0)
                continue;

            if (rect.contains(p_2d))
            {
                correspondences[i] = entities[j];
                break;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        usage();
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Load aggregator configuration

    ed::perception::Aggregator aggregator;

    tue::Configuration config;
    if (config.loadFromYAMLFile(argv[1]))
        aggregator.configure(config, false);

    if (config.hasError())
    {
        std::cout << config.error() << std::endl;
        return 1;
    }

    aggregator.loadRecognitionData();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ImageCrawler crawler;
    crawler.setPath(argv[2]);

    while(crawler.next())
    {
        std::cout << crawler.filename() << std::endl;

        std::vector<ed::EntityConstPtr> correspondences;
        correspond(*crawler.image(), crawler.annotations(), crawler.entities(), correspondences);

        for(unsigned int i = 0; i < crawler.annotations().size(); ++i)
        {
            const ed::EntityConstPtr& e = correspondences[i];
            if (!e)
                continue;

            const Annotation& a = crawler.annotations()[i];

            tue::Configuration data;
            ed::perception::ClassificationOutput output(data);
            ed::perception::CategoricalDistribution prior;

            aggregator.classify(*e, "type", prior, output);

            std::cout << a.label << ": " << output.likelihood << std::endl;

//            std::cout << data << std::endl;
        }
    }

    return 0;
}
