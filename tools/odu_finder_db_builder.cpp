#include <fstream>
#include <iostream>
#include <list>

#include <vocabulary_tree/simple_kmeans.h>
#include <vocabulary_tree/vocabulary_tree.h>
#include <vocabulary_tree/database.h>
#include <vocabulary_tree/tree_builder.h>
#include <vocabulary_tree/simple_kmeans.h>

#include <siftfast/siftfast.h>

#include <dirent.h>
#include <cmath>
#include <sys/stat.h>
#include <ANN/ANN.h>
#include <math.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <time.h>

#include <tue/config/configuration.h>

#include <boost/filesystem.hpp>


// TYPE DEFS
typedef Eigen::Matrix<float, 1, 128> Feature;
typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;

// DOCUMENTINFO CLASS
class DocumentInfo
{
    private:
        bool delete_document;
    public:
        vt::Document* document;
        std::string name;
        DocumentInfo();
        DocumentInfo(vt::Document* document, std::string& name);
        ~DocumentInfo();
        void write (std::ostream& out);
        void read(std::istream& in);
};

// ----------------------------------------------------------------------------------------------------

DocumentInfo::DocumentInfo() :
    delete_document(false) {
}

// ----------------------------------------------------------------------------------------------------

DocumentInfo::DocumentInfo(vt::Document* document, std::string& name) :
    delete_document(false), document(document), name(name) {
}

// ----------------------------------------------------------------------------------------------------

DocumentInfo::~DocumentInfo() {
    if (delete_document)
        delete[] document;
}

// ----------------------------------------------------------------------------------------------------

void DocumentInfo::write(std::ostream& out) {
    size_t length = name.length();
    out.write((char*) &length, sizeof(size_t));
    out.write(name.c_str(), name.length());
    size_t doc_length = document->size();
    out.write((char*) &doc_length, sizeof(size_t));
    out.write((char*) &(document->at(0)), doc_length * sizeof(vt::Word));
}

// ----------------------------------------------------------------------------------------------------

void DocumentInfo::read(std::istream& in) {
    size_t length;
    in.read((char*) &length, sizeof(size_t));
    char* name = new char[length + 1];
    in.read(name, length);
    name[length] = 0;
    this->name.assign(name);
    size_t doc_length;
    in.read((char*) &doc_length, sizeof(size_t));
    document = new vt::Document(doc_length);
    in.read((char*) &document->at(0), doc_length * sizeof(vt::Word));
    this->delete_document = true;
    delete[] name;
}



// ######################################################################
// ######################################################################

class OduDBBuilder{
    public:

       OduDBBuilder(std::string debug_folder);
       void learnImage(std::string model_name, cv::Mat& image);
       void buildDatabase(std::string database_directory_);

    private:

        // CONFIGURATION VARIABLES
        std::string module_name_;
        std::string debug_dir_;
        std::vector<std::string> included_dirs_;
        int votes_count_;
        int tree_k_;
        int tree_levels_;
        int min_cluster_size_;
        int object_id_;

        // DATABASE
        std::vector<vt::Document> docs_;
        vt::TreeBuilder<Feature> tree_builder_;
        vt::VocabularyTree<Feature> tree_;
        vt::Database* db_;
        std::vector<std::string> image_names_;
        std::map<int, DocumentInfo*> documents_map_;
        std::string database_directory_;
        std::vector<FeatureVector> images_;

        // FUNCTIONS
        void saveDatabaseWithoutTree(std::string& directory);
        void saveDatabase(std::string& directory);
        bool processFile(IplImage* image, std::vector<FeatureVector>& images_learn, std::string model_name, bool save_debug_img);
        Keypoint extractKeypoints(IplImage *image, bool frames_only);
};


// ----------------------------------------------------------------------------------------------------

OduDBBuilder::OduDBBuilder(std::string debug_folder){
    module_name_ = "odu_finder_db_builder";

    tree_builder_ = vt::TreeBuilder<Feature> (Feature::Zero());
    tree_k_ = 5;
    tree_levels_ = 5;

    debug_dir_ = debug_folder;

    // create
    try {
        boost::filesystem::path dir(debug_dir_);
        boost::filesystem::remove_all(dir);
        boost::filesystem::create_directories(dir);
    } catch(const boost::filesystem::filesystem_error& e){
       if(e.code() == boost::system::errc::permission_denied)
           std::cout << "boost::filesystem permission denied" << std::endl;
       else
           std::cout << "boost::filesystem failed with error: " << e.code().message() << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

void OduDBBuilder::learnImage(std::string model_name, cv::Mat& image){

    // convert to grayscale
    if(image.channels() > 1){
        cv::cvtColor(image, image, CV_BGR2GRAY);
    }

    // create compatible copy
    IplImage copy = image;
    IplImage* image_ipl = &copy;

    if(processFile(image_ipl, images_, model_name, true)){
        image_names_.push_back(model_name);
    } else{
        std::cout << "[" << module_name_ << "] " << "No features found, ignoring image for model " << model_name << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

void OduDBBuilder::buildDatabase(std::string database_directory) {

    try {
        boost::filesystem::path dir(database_directory);
        boost::filesystem::remove_all(dir);
        boost::filesystem::create_directories(dir);
    } catch(const boost::filesystem::filesystem_error& e){
       if(e.code() == boost::system::errc::permission_denied)
           std::cout << "boost::filesystem permission denied" << std::endl;
       else
           std::cout << "boost::filesystem failed with error: " << e.code().message() << std::endl;
    }

    FeatureVector all_features;
    for (unsigned int i = 0; i < images_.size(); ++i)
        for (unsigned int j = 0; j < images_[i].size(); ++j)
            all_features.push_back(images_[i][j]);

    std::cout << "[" << module_name_ << "] " << "Building a tree with " << all_features.size() << " nodes..." << std::endl;

    tree_builder_.build(all_features, tree_k_, tree_levels_);
    tree_ = tree_builder_.tree();

    std::cout << "[" << module_name_ << "] " << "Creating the documents..." << std::endl;

    docs_.resize(images_.size());

    for (unsigned int i = 0; i < images_.size(); ++i) {

        for (unsigned int j = 0; j < images_[i].size(); ++j) {
            docs_[i].push_back(tree_.quantize(images_[i][j]));
        }
    }

    std::cout << "[" << module_name_ << "] " << "Creating database with " << images_.size() << " images" <<std::endl;
    db_ = new vt::Database(tree_.words());

    std::cout << "[" << module_name_ << "] " << "Populating the database with the documents..." << std::endl;
    for (unsigned int i = 0; i < images_.size(); ++i) {
        documents_map_[db_->insert(docs_[i])] = new DocumentInfo(&(docs_[i]), image_names_[i]);
    }

    std::cout << "[" << module_name_ << "] " << "Training database..." << std::endl;
    db_->computeTfIdfWeights(1);

    std::cout << "[" << module_name_ << "] " << "Database created!" << std::endl;

    std::cout << "[" << module_name_ << "] " << "Saving database to " << database_directory << std::endl;
    saveDatabase(database_directory);

    std::cout << "[" << module_name_ << "] " << "Database ready!" << std::endl;
}



// ----------------------------------------------------------------------------------------------------

void OduDBBuilder::saveDatabaseWithoutTree(std::string& directory) {
//        std::cout << "[" << moduleName_ << "] " << "Saving documents..." << std::endl;

    std::string documents_file(directory);
    documents_file.append("/images.documents");
    std::ofstream out(documents_file.c_str(), std::ios::out | std::ios::binary);
    size_t map_size = documents_map_.size();
    out.write((char*) &map_size, sizeof(size_t));
    std::map<int, DocumentInfo*>::iterator iter;

    for (iter = documents_map_.begin(); iter != documents_map_.end(); ++iter) {
        // TODO: sometimes the last iter->second->document has size 0, this is just a quick fix
        out.write((char*) &iter->first, sizeof(int));
        iter->second->write(out);
    }

//        std::cout << "[" << moduleName_ << "] " << "Saving weights..." << std::endl;

    std::string weights_file(directory);
    weights_file.append("/images.weights");
    db_->saveWeights(weights_file.c_str());
    out.close();
}

// ----------------------------------------------------------------------------------------------------

void OduDBBuilder::saveDatabase(std::string& directory) {
//        std::cout << "[" << moduleName_ << "] " << "Saving the tree..." << std::endl;
    std::string tree_file(directory);
    tree_file.append("/images.tree");
    tree_.save(tree_file.c_str());
    saveDatabaseWithoutTree(directory);
}

// ----------------------------------------------------------------------------------------------------

bool OduDBBuilder::processFile(IplImage* image, std::vector<FeatureVector>& images_learn, std::string model_name, bool save_debug_img) {

    Keypoint keypoints = extractKeypoints(image, false);

    FeatureVector features;
    Keypoint p = keypoints;
    int count = 0;

    while (p != NULL){
        Feature f(p->descrip);
        features.push_back(f);
        p = p->next;
        ++count;
    }

//        std::cout << "[" << moduleName_ << "] " << count << " features found" << std::endl;

    // if features were found, save the image
    if (count > 0){
        images_learn.push_back(features);

        if (save_debug_img){
            std::string save_path = debug_dir_ + model_name + ".png";

            // copy image and converto to RGB
            IplImage *destination = cvCreateImage( cvSize( image->width, image->height), IPL_DEPTH_8U, 3);
            cvCvtColor(image, destination, CV_GRAY2BGR);

            // draw circles over keypoints
            p = keypoints;
            while (p != NULL) {
                cvCircle(destination, cvPoint((int) (p->col), (int) (p->row)), 3, cvScalar(0, 0, 255));
                p = p->next;
            }
            cvSaveImage(save_path.c_str(), destination);
            cvReleaseImage(&destination);
        }

        FreeKeypoints(keypoints);
    }else
        // if features were not found, return failure in learning
        return false;

    return count > 0;
}

// ----------------------------------------------------------------------------------------------------


Keypoint OduDBBuilder::extractKeypoints(IplImage *image, bool frames_only) {

    Image sift_image = CreateImage(image->height, image->width);

    for (int i = 0; i < image->height; ++i) {
        uint8_t* pSrc = (uint8_t*) image->imageData + image->widthStep * i;
        float* pDst = sift_image->pixels + i * sift_image->stride;
        for (int j = 0; j < image->width; ++j)
            pDst[j] = (float) pSrc[j] * (1.0f / 255.0f);
    }

    Keypoint keypoints;
    if (frames_only)
        keypoints = GetKeypointFrames(sift_image);
    else
        keypoints = GetKeypoints(sift_image);
    DestroyAllImages();
    return keypoints;
}
