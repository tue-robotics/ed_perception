#include "image_crawler.h"
#include "ed/perception/aggregator.h"

#include <rgbd/Image.h>

#include <ed/entity.h>
#include <ed/measurement.h>

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: train-perception CONFIG-FILE IMAGE-FILE-OR-DIRECTORY" << std::endl;
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
        aggregator.configure(config);

    if (config.hasError())
    {
        std::cout << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ImageCrawler crawler;
    crawler.setPath(argv[2]);

    while(crawler.next())
    {
        std::cout << crawler.filename() << std::endl;
        std::cout << crawler.entities().size() << std::endl;

        const cv::Mat& depth = crawler.image()->getDepthImage();

        std::vector<cv::Rect> entity_rects(crawler.entities().size());

        for(unsigned int i = 0; i < crawler.entities().size(); ++i)
        {
            const ed::EntityConstPtr& e = crawler.entities()[i];
            if (!e->has_pose() || e->shape() || !e->bestMeasurement())
                continue;

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

        for(std::vector<Annotation>::const_iterator it = crawler.annotations().begin(); it != crawler.annotations().end(); ++it)
        {
            const Annotation& a = *it;
            cv::Point p_2d(a.px * depth.cols, a.py * depth.rows);

            std::cout << "Annotation " << a.label << ": " << p_2d << std::endl;

            for(unsigned int i = 0; i < crawler.entities().size(); ++i)
            {
                const ed::EntityConstPtr& e = crawler.entities()[i];
                if (!e->has_pose() || e->shape() || !e->bestMeasurement())
                    continue;


                const cv::Rect& rect = entity_rects[i];

                std::cout << "    " << rect << std::endl;

                if (rect.contains(p_2d))
                {
                    if (!e->shape())
                    {
                        std::cout << e->id() << " " << a.label << std::endl;
                        aggregator.addTrainingInstance(*e, "type", a.label);
                        break;
                    }
                }
            }
        }

    }

    aggregator.train();
    aggregator.saveRecognitionData();

    return 0;
}
