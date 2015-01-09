#include "face_recognition.h"

#include "ed/measurement.h"
#include <ed/entity.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>


// ----------------------------------------------------------------------------------------------------

FaceRecognition::FaceRecognition() :
    PerceptionModule("face_recognition"),
    init_success_(false)
{

}


// ----------------------------------------------------------------------------------------------------

FaceRecognition::~FaceRecognition(){

}

// ----------------------------------------------------------------------------------------------------

void FaceRecognition::loadConfig(const std::string& config_path) {

    module_name_ = "face_recognition";
    debug_mode_ = false;

    using_Eigen_ = true;
    using_Fisher_ = true;
    using_LBP_ = true;

    // Get the path to your CSV.
    std::string fn_csv = config_path + "/faces_learned.csv";

    // Read in the data. This can fail if no valid input filename is given
    try {
        std::cout << "[" << module_name_ << "] " << "Reading CSV file with faces locations and names" << std::endl;
        read_csv(fn_csv, images_, labels_, labelsInfo_);
    } catch (cv::Exception& e) {
        std::cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << std::endl;
        exit(1);
    }

    // Quit if there are not enough images to train
    if(images_.size() <= 1) {
        std::string error_message = "Dataset only has 2 images, need more.";
        CV_Error(CV_StsError, error_message);
    }

    // resize to the maximum number of FaceRecognizers you're going to use
    models.resize(3);

    // train Eingen Faces
    if (using_Eigen_){
        models[EIGEN] = cv::createEigenFaceRecognizer();

        std::cout << "[" << module_name_ << "] " << "Training Eigen Faces..." << std::endl;
        models[EIGEN]->train(images_, labels_);

//        std::string saveModelPath = "/tmp/face-rec-model_eigen.txt";
//        models[EIGEN]->save(saveModelPath);

        std::cout << "[" << module_name_ << "] " << "Eigen Faces trained!" << std::endl;
    }

    // train Fisher Faces
    if (using_Fisher_){
        models[FISHER] = cv::createFisherFaceRecognizer();

        std::cout << "[" << module_name_ << "] " << "Training Fisher Faces..." << std::endl;
        models[FISHER]->train(images_, labels_);

//        std::string saveModelPath = "/tmp/face-rec-model_fisher.txt";
//        models[FISHER]->save(saveModelPath);

        std::cout << "[" << module_name_ << "] " << "Fisher Faces trained!" << std::endl;
    }

    // train LBP
    if (using_LBP_){
        models[LBPH] = cv::createLBPHFaceRecognizer();

        std::cout << "[" << module_name_ << "] " << "Training LBPH Faces..." << std::endl;
        models[LBPH]->train(images_, labels_);

//        std::string saveModelPath = "/tmp/face-rec-model_LBPH.txt";
//        models[LBPH]->save(saveModelPath);

        std::cout << "[" << module_name_ << "] " << "LBPH Faces trained!" << std::endl;
    }

    init_success_ = true;
    std::cout << "[" << module_name_ << "] " << "Ready!" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void FaceRecognition::process(ed::EntityConstPtr e, tue::Configuration& config) const{

    if (!init_success_)
        return;

    if (!isFaceFound(config.limitScope())){
        std::cout << "[" << module_name_ << "] " << "Couldn't find a face, skipping recognition"<< std::endl;
        return;
    }

    // ---------- Prepare measurement ----------

    // Get the best measurement from the entity
    ed::MeasurementConstPtr msr = e->lastMeasurement();
    if (!msr)
        return;

    uint min_x, max_x, min_y, max_y;

    // create a view
    rgbd::View view(*msr->image(), msr->image()->getRGBImage().cols);

    // get color image
    const cv::Mat& color_image = msr->image()->getRGBImage();

    // crop it to match the view
    cv::Mat cropped_image(color_image(cv::Rect(0,0,view.getWidth(), view.getHeight())));

    // initialize bounding box points
    max_x = 0;
    max_y = 0;
    min_x = view.getWidth();
    min_y = view.getHeight();

    cv::Mat mask = cv::Mat::zeros(view.getHeight(), view.getWidth(), CV_8UC1);
    // Iterate over all points in the mask
    for(ed::ImageMask::const_iterator it = msr->imageMask().begin(view.getWidth()); it != msr->imageMask().end(); ++it)
    {
        // mask's (x, y) coordinate in the depth image
        const cv::Point2i& p_2d = *it;

        // paint a mask
        mask.at<unsigned char>(*it) = 255;

        // update the boundary coordinates
        if (min_x > p_2d.x) min_x = p_2d.x;
        if (max_x < p_2d.x) max_x = p_2d.x;
        if (min_y > p_2d.y) min_y = p_2d.y;
        if (max_y < p_2d.y) max_y = p_2d.y;
    }

    // ----------------------- Process -----------------------

    cv::Rect faceRect;
    cv::Point eye_left;
    cv::Point eye_right;

    // get location of face and eyes
    if (!getFaceInfo(config.limitScope(), faceRect)){
        std::cout << "[" << module_name_ << "] " << "Problem retrieving face information" << std::endl;
    }

    cv::Mat face(cropped_image(faceRect));


    // ----------------------- TESTING ---------------------------------
    std::string num_test = boost::lexical_cast<std::string>(rand() % 8);
    std::string test_path = "/home/luisf/test_subjects/person" + num_test + ".pgm";
    cv::Mat test_img = cv::imread(test_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    std::string test_name = "person" + num_test;
    // ------------------------------------------------------------------

    // The following line predicts the label of a given
    std::vector<int> predictedLabel; predictedLabel.resize(3);
    std::vector<double> confidence; confidence.resize(3);

    predictedLabel[EIGEN] = -1; confidence[EIGEN] = 0.0;
    models[EIGEN]->predict(test_img, predictedLabel[EIGEN], confidence[EIGEN]);

    predictedLabel[FISHER] = -1; confidence[FISHER] = 0.0;
    models[FISHER]->predict(test_img, predictedLabel[FISHER], confidence[FISHER]);

    predictedLabel[LBPH] = -1; confidence[LBPH] = 0.0;
    models[LBPH]->predict(test_img, predictedLabel[LBPH], confidence[LBPH]);

    std::cout << "[" << module_name_ << "] " << "Eigen faces: " << labelsInfo_.at(predictedLabel[EIGEN]) << "(" << test_name << ")"
              << " with probability " << confidence[EIGEN] <<std::endl;

    std::cout << "[" << module_name_ << "] " << "Fisher faces: " << labelsInfo_.at(predictedLabel[FISHER]) << "(" << test_name << ")"
              << " with probability " << confidence[FISHER] <<std::endl;

    std::cout << "[" << module_name_ << "] " << "LBPH faces: " << labelsInfo_.at(predictedLabel[LBPH]) << "(" << test_name << ")"
              << " with probability " << confidence[LBPH] <<std::endl;



    // ----------------------- Assert results -----------------------

    // create group if it doesnt exist
    if (!config.readGroup("perception_result", tue::OPTIONAL))
    {
        config.writeGroup("perception_result");
    }

    config.writeGroup("face_recognizer");

    config.writeGroup("eigen");
    config.setValue("name", labelsInfo_.at(predictedLabel[EIGEN]));
    config.setValue("score", confidence[EIGEN]);
    config.endGroup();  // close eigen group

    config.writeGroup("fisher");
    config.setValue("name", labelsInfo_.at(predictedLabel[FISHER]));
    config.setValue("score", confidence[FISHER]);
    config.endGroup();  // close fisher group

    config.writeGroup("lbph");
    config.setValue("name", labelsInfo_.at(predictedLabel[LBPH]));
    config.setValue("score", confidence[LBPH]);
    config.endGroup();  // close lbph group

    config.endGroup();  // close face_recognizer group
    config.endGroup();  // close perception_result group
}


// ----------------------------------------------------------------------------------------------------

bool FaceRecognition::isFaceFound(tue::Configuration config) const{

    double score;
    std::string group_label = "face";

    if (!config.readGroup("perception_result", tue::OPTIONAL))
        return false;

    if (!config.readGroup("face_detector", tue::OPTIONAL))
        return false;


    if (config.value("score", score, tue::OPTIONAL) && config.value("label", group_label, tue::OPTIONAL)){
        return score == 1;
    }
}


// ----------------------------------------------------------------------------------------------------

bool FaceRecognition::getFaceInfo(tue::Configuration config, cv::Rect& faceRect) const{

    int topX = 0, topY= 0, width= 0, height= 0;

    if (!config.readGroup("perception_result", tue::OPTIONAL)) return false;

    if (!config.readGroup("face_detector", tue::OPTIONAL)) return false;

    if (!config.readGroup("faces_front", tue::OPTIONAL)) return false;

//    std::cout << "im here" << std::endl;

    if (config.value("height", height)){
        std::cout << "found it" << std::endl;
//        return false;
    }

    faceRect = cv::Rect(topX, topY, width, height);

    std::cout << "rect " << topX << ", " << topY << ", " << height << ", " << width << std::endl;

    return true;
}

// ----------------------------------------------------------------------------------------------------

void FaceRecognition::read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels, std::map<int, std::string>& labelsInfo, char separator) {
    std::ifstream csv(filename.c_str());

    if (!csv) CV_Error(CV_StsBadArg, "No valid input file was given, please check the given filename.");

    std::string line;
    std::string path;
    std::string classlabel;
    std::string info;
    int unique_names=0;

    while (getline(csv, line)) {
        std::stringstream line_stream(line);
        path.clear();
        classlabel.clear();
        info.clear();

        getline(line_stream, path, separator);
        getline(line_stream, classlabel, separator);
        getline(line_stream, info, separator);

        if(!path.empty() && !classlabel.empty()) {
//            std::cout << "Processing " << path << std::endl;
            int label = atoi(classlabel.c_str());

            if(!info.empty()){
                if (labelsInfo.find(label) == labelsInfo.end()) unique_names ++;
                labelsInfo.insert(std::make_pair(label, info));
            }

            // 'path' can be file, dir or wildcard path
            std::string root(path.c_str());
            std::vector<std::string> files;
            cv::glob(root, files, true);

            for(std::vector<std::string>::const_iterator f = files.begin(); f != files.end(); ++f) {
//                std::cout << "\t" << *f << std::endl;
                cv::Mat img = cv::imread(*f, CV_LOAD_IMAGE_GRAYSCALE);
                static int w=-1, h=-1;
                static bool showSmallSizeWarning = true;

                if(w>0 && h>0 && (w!=img.cols || h!=img.rows)) std::cout << "\t* Warning: images should be of the same size!" << std::endl;

                if(showSmallSizeWarning && (img.cols<50 || img.rows<50)) {
                    std::cout << "* Warning: for better results images should be not smaller than 50x50!" << std::endl;
                    showSmallSizeWarning = false;
                }

                images.push_back(img);
                labels.push_back(label);
            }
        }
    }

    std::cout << "[" << module_name_ << "] " << images_.size() << " faces were loaded for " << unique_names << " different people." << std::endl;
}


ED_REGISTER_PERCEPTION_MODULE(FaceRecognition)
