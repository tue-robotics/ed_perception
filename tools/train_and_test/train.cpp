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
        aggregator.configure(config, true);

    if (config.hasError())
    {
        std::cout << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ImageCrawler crawler;
    crawler.setPath(argv[2]);

    AnnotatedImage image;

    while(crawler.next(image))
    {
        std::cout << crawler.filename() << std::endl;

        std::vector<ed::EntityConstPtr> correspondences;
        findAnnotationCorrespondences(image, correspondences);

        for(unsigned int i = 0; i < image.annotations.size(); ++i)
        {
            const ed::EntityConstPtr& e = correspondences[i];
            const Annotation& a = image.annotations[i];
            if (!e || a.is_supporting)
                continue;

            aggregator.addTrainingInstance(*e, "type", a.label);
        }
    }

    aggregator.train();
    aggregator.saveRecognitionData();

    return 0;
}
