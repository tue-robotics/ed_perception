#include "image_crawler.h"
#include "ed/perception/aggregator.h"

#include "../confusionmatrix.h"

// ----------------------------------------------------------------------------------------------------

void usage()
{
    std::cout << "Usage: test-perception CONFIG-FILE IMAGE-FILE-OR-DIRECTORY" << std::endl;
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


    ConfusionMatrix confusionMatrix;

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

            tue::Configuration data;
            ed::perception::ClassificationOutput output(data);
            ed::perception::CategoricalDistribution prior;

            aggregator.classify(*e, "type", prior, output);

            std::cout << a.label << ": " << output.likelihood << std::endl;

            confusionMatrix.addResult(output.likelihood,a.label);

//            std::cout << data << std::endl;
        }
    }

    confusionMatrix.show();

    return 0;
}
