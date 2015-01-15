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

    // module configuration
    module_name_ = "face_recognition";
    debug_mode_ = false;

    using_Eigen_ = false;
    using_Fisher_ = true;
    using_LBPH_ = true;

    face_target_size_ = 150;
    face_vert_offset_ = 0.35;
    face_horiz_offset_ = 0.25;

    eigen_treshold_ = 3000;
    fisher_treshold_ = 2000;
    lbph_treshold_ = 50;

    // Get the path to the CSV file and images
    std::string csv_file_path = config_path + "/faces_learned_office.csv";
    std::string images_relative_path = config_path + "/faces_office/";

    // Read in the data. This can fail if no valid input filename is given
    try {
        std::cout << "[" << module_name_ << "] " << "Reading CSV file with faces locations and names" << std::endl;
        read_csv(csv_file_path, images_relative_path, images_, labels_, labelsInfo_);
    } catch (cv::Exception& e) {
        std::cout << "[" << module_name_ << "] " << "Error opening file \"" << csv_file_path << "\". Reason: " << e.msg << std::endl;
        return;
    }

    // quit if there are not enough images to train
    if(images_.size() <= 1) {
        std::cout << "[" << module_name_ << "] " << "Face dataset doesn't have enought images! Need more than 2." << std::endl;
        return;
    }

    // resize to the maximum number of FaceRecognizers you're going to use
    models.resize(3);

    // train Eingen Faces
    if (using_Eigen_){
        models[EIGEN] = cv::createEigenFaceRecognizer();
        std::cout << "[" << module_name_ << "] " << "Training Eigen Faces..." << std::endl;
        models[EIGEN]->train(images_, labels_);
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
    if (using_LBPH_){
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

    cv::Rect face_rect;
    cv::Mat face_aligned;

    // get location of face and eyes
    if (!getFaceInfo(config.limitScope(), face_rect)){
        std::cout << "[" << module_name_ << "] " << "Problem retrieving face information, aborting recognition!" << std::endl;
        return;
    }

    if (!alignFace(cropped_image, face_rect, face_target_size_, face_horiz_offset_, face_vert_offset_, face_aligned))
        std::cout << "[" << module_name_ << "] " << "Could not align face!" << std::endl;

    // ----------------------- MANUAL LEARNING --------------------------------------------------------------------------
//    cv::imwrite((std::string)"/home/luisf/faces_office/" + ed::Entity::generateID().c_str() + ".pgm", face_aligned);

//    cv::imwrite("/tmp/min_box.png", cropped_image(bouding_box));
//    cv::imwrite("/tmp/face.png", cropped_image(faceRect));
//    cv::imwrite("/tmp/face_aligned.png", face_aligned);
//    cv::imwrite("/tmp/full.png", cropped_image);

    // -----------------------------------------------------------------------------------------------------------

    // The following line predicts the label of a given
    std::vector<int> predict_label; predict_label.resize(3);
    std::vector<double> confidence; confidence.resize(3);

    // initialize values
    predict_label[EIGEN] = -1; confidence[EIGEN] = 0.0;
    predict_label[FISHER] = -1; confidence[FISHER] = 0.0;
    predict_label[LBPH] = -1; confidence[LBPH] = 0.0;

    // perform recognition
    if (using_Eigen_){
        models[EIGEN]->predict(face_aligned, predict_label[EIGEN], confidence[EIGEN]);
    }

    if (using_Fisher_){
        models[FISHER]->predict(face_aligned, predict_label[FISHER], confidence[FISHER]);
    }

    if (using_LBPH_){
        models[LBPH]->predict(face_aligned, predict_label[LBPH], confidence[LBPH]);
    }


    // ----------------------- Assert results -----------------------

    // fill labels with the predicted name, in case there was a prediction
    std::string eigen_label = predict_label[EIGEN] > -1 ? labelsInfo_.at(predict_label[EIGEN]) : "";
    std::string fisher_label = predict_label[FISHER] > -1 ? labelsInfo_.at(predict_label[FISHER]) : "";
    std::string lbph_label = predict_label[LBPH] > -1 ? labelsInfo_.at(predict_label[LBPH]) : "";

    // match these results into one
    double confidence_match;
    std::string label_match;

    matchResults(eigen_label, fisher_label, lbph_label,
                 confidence[EIGEN], confidence[FISHER], confidence[LBPH],
                 label_match, confidence_match);

    // create group if it doesnt exist
    if (!config.readGroup("perception_result", tue::OPTIONAL))
    {
        config.writeGroup("perception_result");
    }

    config.writeGroup("face_recognizer");

    if (!label_match.empty()){
        config.setValue("label", label_match);
        config.setValue("score", confidence_match);
    }else{
        config.setValue("label", "");
        config.setValue("score", 0);
    }

    if (using_Eigen_ && predict_label[EIGEN] > -1){
        config.writeGroup("eigen");
        config.setValue("name", eigen_label);
        config.setValue("score", confidence[EIGEN]);
        config.endGroup();  // close eigen group
    }

    if (using_Fisher_ && predict_label[FISHER] > -1){
        config.writeGroup("fisher");
        config.setValue("name", fisher_label);
        config.setValue("score", confidence[FISHER]);
        config.endGroup();  // close fisher group
    }

    if (using_LBPH_ && predict_label[LBPH] > -1){
        config.writeGroup("lbph");
        config.setValue("name", lbph_label);
        config.setValue("score", confidence[LBPH]);
        config.endGroup();  // close lbph group
    }

    config.endGroup();  // close face_recognizer group
    config.endGroup();  // close perception_result group
}


// ----------------------------------------------------------------------------------------------------


void FaceRecognition::matchResults(std::string eigenLabel, std::string fisherLabel, std::string lbphLabel,
                                   float eigenConf, float fisherConf, float lbphConf,
                                   std::string& label_match, double& confidence_match) const{

    std::map <std::string, double> ordered_results;
    std::map <std::string, double>::iterator find_it;

    // get a the confidence in terms of percentage, regarding the threshold of each method
    if (using_Eigen_ && !eigenLabel.empty()){
        find_it = ordered_results.find(eigenLabel);
        // average the error percentage in case the match already existed
        if (find_it != ordered_results.end())
            find_it->second = (find_it->second + (eigenConf/eigen_treshold_))/2;
        else
            ordered_results[eigenLabel] = eigenConf/eigen_treshold_;
    }

    if (using_Fisher_ && !fisherLabel.empty()){
        find_it = ordered_results.find(fisherLabel);
        if (find_it != ordered_results.end())
            find_it->second = (find_it->second + (fisherConf/fisher_treshold_))/2;
        else
            ordered_results[fisherLabel] = fisherConf/fisher_treshold_;
    }

    if (using_LBPH_&& !lbphLabel.empty()){
        find_it = ordered_results.find(lbphLabel);
        if (find_it != ordered_results.end())
            find_it->second = (find_it->second + (lbphConf/lbph_treshold_))/2;
        else
            ordered_results[lbphLabel] = lbphConf/lbph_treshold_;
    }

//    std::cout << "values: " <<std::endl;
//    for(std::map<std::string, double>::const_iterator it = ordered_results.begin(); it != ordered_results.end(); ++it)
//    {
//        std::cout << it->first << ", " << it->second << "\n";
//    }

    // return the match with the lowest percentage of error
    label_match = ordered_results.begin()->first;
    confidence_match = ordered_results.begin()->second;
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

    // get the region of the first face found in the config
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


void FaceRecognition::read_csv(const std::string& filename,
                               const std::string& images_relative_path,
                               std::vector<cv::Mat>& images,
                               std::vector<int>& labels,
                               std::map<int, std::string>& labelsInfo,
                               char separator) {

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
            std::string root(images_relative_path.c_str()); root.append(path.c_str());
            std::vector<std::string> files;
            cv::glob(root, files, true);

            for(std::vector<std::string>::const_iterator f = files.begin(); f != files.end(); ++f) {
//                std::cout << "\t reading from: " << *f << std::endl;
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


bool FaceRecognition::alignFace(cv::Mat origImg, cv::Rect faceLoc, int targetSize, float horizOffset, float vertOffset, cv::Mat& faceImg) const{
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
    cv::equalizeHist(faceDetectGray, faceDetectGray);

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
        dist = euclidDistance(leftEye, rightEye);

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

        cropArea = cv::Rect(clipInt(topCorner.x, 0 , faceImg.cols - targetSize),
                        clipInt(topCorner.y, 0 , faceImg.rows - targetSize),
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
            // offset of the new face rectangle compared to the original one
            topOffset = faceDetectGray.cols * 0.05;
            // zoom in on the face
            size = faceDetectGray.cols * 0.90;
        }else{
            // offset of the new face rectangle compared to the original one
            topOffset = faceDetectGray.rows * 0.05;
            // zoom in on the face
            size = faceDetectGray.rows * 0.90;
        }

        // crop the face, without going over the image
        faceDetectGray(cv::Rect(topOffset, topOffset*2,
                     clipInt(size, 1, faceDetectGray.cols - topOffset),
                     clipInt(size, 1, faceDetectGray.rows - topOffset*2))).copyTo(faceDetectGray);

        resize(faceDetectGray, faceImg, cv::Size(targetSize, targetSize), 0, 0, cv::INTER_NEAREST);

        faceAligned = true;
    }

    return faceAligned;
}

// ----------------------------------------------------------------------------------------------------

int FaceRecognition::clipInt(int val, int min, int max) const{
    return val <= min ? min : val >= max ? max : val;
}

// ----------------------------------------------------------------------------------------------------

int FaceRecognition::euclidDistance(cv::Point p1, cv::Point p2) const{
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
