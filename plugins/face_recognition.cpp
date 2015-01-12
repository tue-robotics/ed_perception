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
    std::string fn_csv = config_path + "/faces_learned_office.csv";

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

        std::cout << "[" << module_name_ << "] " << "Fisher Faces trained!" << std::endl;
    }

    // train LBP
    if (using_LBP_){
        models[LBPH] = cv::createLBPHFaceRecognizer();

        std::cout << "[" << module_name_ << "] " << "Training LBPH Faces..." << std::endl;
        models[LBPH]->train(images_, labels_);

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
//        std::cout << "[" << module_name_ << "] " << "Couldn't find a face, skipping recognition"<< std::endl;
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

    // get location of face and eyes
    if (!getFaceInfo(config.limitScope(), faceRect)){
        std::cout << "[" << module_name_ << "] " << "Problem retrieving face information, aborting recognition!" << std::endl;
        return;
    }

    cv::Mat face(cropped_image(faceRect));
    cv::Mat face_aligned;

    int TARGET_SIZE = 150;
    float VERTICAL_OFFSET = 0.35;
    float HORIZONTAL_OFFSET = 0.25;

    if (!AlignFace(cropped_image, faceRect, TARGET_SIZE, HORIZONTAL_OFFSET, VERTICAL_OFFSET, face_aligned))
        std::cout << "Could not align face!" << std::endl;

    // ----------------------- TESTING --------------------------------------------------------------------------
//    std::string path_temp = "/home/luisf/faces_office/"; path_temp.append(ed::Entity::generateID().c_str()); path_temp.append(".pgm");
//    cv::imwrite(path_temp, face_aligned);

    cv::imwrite("/tmp/face.png", face);
    cv::imwrite("/tmp/face_aligned.png", face_aligned);
    cv::imwrite("/tmp/full.png", cropped_image);

//    std::string num_test = boost::lexical_cast<std::string>(rand() % 8);
//    std::string test_path = "/home/luisf/test_subjects/person" + num_test + ".pgm";
//    face_aligned = cv::imread(test_path.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
//    resize(face_aligned, face_aligned, cv::Size(), 112, 92, cv::INTER_NEAREST);

    // -----------------------------------------------------------------------------------------------------------

    // The following line predicts the label of a given
    std::vector<int> predictedLabel; predictedLabel.resize(3);
    std::vector<double> confidence; confidence.resize(3);

    predictedLabel[EIGEN] = -1; confidence[EIGEN] = 0.0;
    models[EIGEN]->predict(face_aligned, predictedLabel[EIGEN], confidence[EIGEN]);

    predictedLabel[FISHER] = -1; confidence[FISHER] = 0.0;
    models[FISHER]->predict(face_aligned, predictedLabel[FISHER], confidence[FISHER]);

    predictedLabel[LBPH] = -1; confidence[LBPH] = 0.0;
    models[LBPH]->predict(face_aligned, predictedLabel[LBPH], confidence[LBPH]);

//    std::cout << "[" << module_name_ << "] " << "Eigen faces: " << labelsInfo_.at(predictedLabel[EIGEN])
//              << " with probability " << confidence[EIGEN] <<std::endl;

//    std::cout << "[" << module_name_ << "] " << "Fisher faces: " << labelsInfo_.at(predictedLabel[FISHER])
//              << " with probability " << confidence[FISHER] <<std::endl;

//    std::cout << "[" << module_name_ << "] " << "LBPH faces: " << labelsInfo_.at(predictedLabel[LBPH])
//              << " with probability " << confidence[LBPH] <<std::endl;



    // ----------------------- Assert results -----------------------

    // create group if it doesnt exist
    if (!config.readGroup("perception_result", tue::OPTIONAL))
    {
        config.writeGroup("perception_result");
    }

    config.writeGroup("face_recognizer");

    if ((predictedLabel[EIGEN] == predictedLabel[FISHER] && predictedLabel[EIGEN] == predictedLabel[LBPH])||
            predictedLabel[EIGEN] == predictedLabel[FISHER]){

        config.setValue("label", labelsInfo_.at(predictedLabel[EIGEN]));
        config.setValue("score", 1);
    }else{
        config.setValue("label", "");
        config.setValue("score", 0);
    }

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

    if (!config.readArray("faces_front", tue::OPTIONAL)) return false;

    while(config.nextArrayItem()){
        if (!config.value("x", topX) || !config.value("y", topY) || !config.value("height", height) || !config.value("width", width)){
           std::cout << "[" << module_name_ << "] " << "Incorrect construction of face information" << std::endl;
            return false;
        }
    }

    faceRect = cv::Rect(topX, topY, width, height);

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

// ----------------------------------------------------------------------------------------------------

bool FaceRecognition::AlignFace(cv::Mat origImg, cv::Rect faceLoc, int targetSize, float horizOffset, float vertOffset, cv::Mat& faceImg) const{
    std::vector<cv::Rect> leftEyeLoc;
    std::vector<cv::Rect> rightEyeLoc;
    cv::Point leftEye;
    cv::Point rightEye;
    cv::Point refOrigin;
    cv::Point topCorner;
    cv::Rect cropArea;
    int bestEyeLeft = 0;
    int bestEyeRight = 0;
    int horizOffsetEye;
    int vertOffsetEye;
    int dist;
    float scale;
    bool faceAligned = false;
    cv::Mat faceDetectGray;

    // create copy of the image for the haar detector and convert it to gray scale
    cvtColor(origImg(faceLoc), faceDetectGray, CV_BGR2GRAY);

    // detect eyes
    //leftEyeDetector_.detectMultiScale(faceDetectGray, leftEyeLoc, 1.1, 1, 0 | CV_HAAR_SCALE_IMAGE);
    //rightEyeDetector_.detectMultiScale(faceDetectGray, rightEyeLoc, 1.1, 1, 0 | CV_HAAR_SCALE_IMAGE);

    // find the first detected eye that is on the correct side of the face
    for (uint i = 0; i < leftEyeLoc.size(); i++){
        if (leftEyeLoc[i].x < faceDetectGray.cols / 2){
            bestEyeLeft = i;
            break;
        }
    }

    for (uint i = 0; i < rightEyeLoc.size(); i++){
        if (rightEyeLoc[i].x > faceDetectGray.cols / 2){
            bestEyeRight = i;
            break;
        }
    }

    // get center point for the eyes, if found
    if (leftEyeLoc.size() > 0 && rightEyeLoc.size() > 0){
        // get the center of the eye location
        leftEye = cv::Point(leftEyeLoc[bestEyeLeft].x + leftEyeLoc[bestEyeLeft].width / 2,
                leftEyeLoc[bestEyeLeft].y + leftEyeLoc[bestEyeLeft].height / 2);

        rightEye = cv::Point(rightEyeLoc[bestEyeRight].x + rightEyeLoc[bestEyeRight].width / 2,
                rightEyeLoc[bestEyeRight].y + rightEyeLoc[bestEyeRight].height / 2);
    }else{
        leftEye = cv::Point(0,0);
        rightEye = cv::Point(0,0);
    }

    // if the eyes are on the correct side of the face
    if (leftEye.x < faceDetectGray.rows / 2 && rightEye.x > faceDetectGray.rows / 2) {
        refOrigin = cv::Point(faceLoc.x, faceLoc.y);

        // calculate distance between the eyes in original scale
        dist = Distance(leftEye, rightEye);

        if (dist > origImg.cols){
            std::cout << "[" << module_name_ << "] " << images_.size() << " Bad eye positions. Aborting alignement!" << std::endl;
            //leftEye.x, leftEye.y, rightEye.x, rightEye.y;
            return false;
        }

        // copy original color image to align it
        origImg.copyTo(faceImg);

        // draw circles on the eyes
        circle(faceImg, leftEye + refOrigin, 2, cv::Scalar(0, 255, 0), 1);
        circle(faceImg, rightEye + refOrigin, 2, cv::Scalar(0, 0, 255), 1);

        // determine the horizontal and vertical offeset for the left eye
        horizOffsetEye = floor(horizOffset * targetSize);
        vertOffsetEye = floor(vertOffset * targetSize);

        // determine the scale value so that the current distance between the eyes will cv::Match
        //	the one defined by the offsets
        scale = (targetSize - 2 * horizOffsetEye) / (float) dist;

        //rotate and resize the full image
        rotateFace(faceImg, faceImg, leftEye + refOrigin, rightEye + refOrigin, leftEye + refOrigin);
        resize(faceImg, faceImg, cv::Size(), scale, scale, cv::INTER_NEAREST);

        // top corner is the position of the left eye - offsets
        topCorner = cv::Point((leftEye.x + refOrigin.x) * scale, (leftEye.y + refOrigin.y) * scale)
                 - cv::Point (horizOffsetEye, vertOffsetEye);

        cropArea = cv::Rect(ClipInt(topCorner.x, 0 , faceImg.cols - targetSize),
                        ClipInt(topCorner.y, 0 , faceImg.rows - targetSize),
                        targetSize, targetSize);

        if (topCorner.x > faceImg.cols - targetSize || topCorner.y > faceImg.rows - targetSize)
            std::cout << "[" << module_name_ << "] " << " Top Corner had to be clipped, face ROI was too small" << std::endl;

        // crop the aligned image
        faceImg(cropArea).copyTo(faceImg);

        // eyes were corcv::Rectly detected
        faceAligned = true;
    }else{
        // if the eye detection failed, still focus more the original face ROI
        int topOffset;
        int size;

        // make the selected area square
        if (faceDetectGray.cols < faceDetectGray.rows){
            topOffset = faceDetectGray.cols * 0.1;
            size = faceDetectGray.cols * 0.75;
        }else{
            topOffset = faceDetectGray.rows * 0.1;
            size = faceDetectGray.rows * 0.75;
        }

        // crop the face, without going over the image
        faceDetectGray(cv::Rect(topOffset, topOffset*2,
                     ClipInt(size, 1, faceDetectGray.cols - topOffset),
                     ClipInt(size, 1, faceDetectGray.rows - topOffset*2))).copyTo(faceDetectGray);

        resize(faceDetectGray, faceImg, cv::Size(targetSize, targetSize), 0, 0, cv::INTER_NEAREST);

        faceAligned = true;
    }

    return faceAligned;
}

// ----------------------------------------------------------------------------------------------------

int FaceRecognition::ClipInt(int val, int min, int max) const{
    return val <= min ? min : val >= max ? max : val;
}

// ----------------------------------------------------------------------------------------------------

int FaceRecognition::Distance(cv::Point p1, cv::Point p2) const{
    float dx;
    float dy;
    dx = p2.x - p1.x;
    dy = p2.y - p1.y;
    return floor(sqrt(dx * dx + dy * dy));
}

// ----------------------------------------------------------------------------------------------------

void FaceRecognition::rotateFace(cv::Mat faceImg, cv::Mat& rotatedImg, cv::Point leftEye, cv::Point rightEye, cv::Point center) const{
    double mod1, mod2, innerp, angle;
    cv::Point p3 (rightEye.x, leftEye.y);
    cv::Mat rotMat;

    // module of the line between right eye and left eye
    mod1 = sqrt(pow(rightEye.y - leftEye.y, 2) + pow(rightEye.x - leftEye.x, 2));
    // module of the line between the left eye and a cv::Point in the horizontal line passing bye the left eye
    mod2 = sqrt(pow(p3.y - leftEye.y, 2) + pow(rightEye.x - leftEye.x, 2));
    // calculate inner product
    innerp = (rightEye.x - leftEye.x) * (p3.x - leftEye.x) + (rightEye.y - leftEye.y) * (p3.y - leftEye.y);
    // calculate angle and convert it to degrees
    angle = acos(innerp / (mod1 * mod2)) * 180 / M_PI;

    // determine if its a positive or negative rotation
    if (leftEye.y > rightEye.y)
        angle *= -1;

    // get rotation cv::Matrix and apply it to the image
    rotMat = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::warpAffine(faceImg, rotatedImg, rotMat, faceImg.size());
}

ED_REGISTER_PERCEPTION_MODULE(FaceRecognition)
