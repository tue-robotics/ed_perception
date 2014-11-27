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



// ----------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------

class OduFinderLearner{
    public:

    // ----------------------------------------------------------------------------------------------------

    OduFinderLearner(){
        moduleName_ = "odu_Finder_learner";

        tree_builder = vt::TreeBuilder<Feature> (Feature::Zero());
        tree_k = 5;
        tree_levels = 5;

    }

    // ----------------------------------------------------------------------------------------------------

    void learnImage(std::string model_name, cv::Mat& image){

        // convert to grayscale
        cv::cvtColor(image, image, CV_BGR2GRAY);

        // create compatible copy
        IplImage copy = image;
        IplImage* image_ipl = &copy;

        if(process_file(image_ipl, images, false)){
            image_names.push_back(model_name);
        } else{
            std::cout << "[" << moduleName_ << "] " << "No features found, ignoring image for model " << model_name << std::endl;
        }
    }

    // ----------------------------------------------------------------------------------------------------

    void build_database(std::string database_directory) {

        FeatureVector all_features;
        for (unsigned int i = 0; i < images.size(); ++i)
            for (unsigned int j = 0; j < images[i].size(); ++j)
                all_features.push_back(images[i][j]);

        std::cout << "[" << moduleName_ << "] " << "Building a tree with " << all_features.size() << " nodes..." << std::endl;

        tree_builder.build(all_features, tree_k, tree_levels);
        tree = tree_builder.tree();

        std::cout << "[" << moduleName_ << "] " << "Creating the documents..." << std::endl;

        docs.resize(images.size());

        for (unsigned int i = 0; i < images.size(); ++i) {

            for (unsigned int j = 0; j < images[i].size(); ++j) {
                docs[i].push_back(tree.quantize(images[i][j]));
            }
        }

        std::cout << "[" << moduleName_ << "] " << "Creating database with " << images.size() << " images" <<std::endl;
        db = new vt::Database(tree.words());

        std::cout << "[" << moduleName_ << "] " << "Populating the database with the documents..." << std::endl;
        for (unsigned int i = 0; i < images.size(); ++i) {
            documents_map[db->insert(docs[i])] = new DocumentInfo(&(docs[i]), image_names[i]);
        }

        std::cout << "[" << moduleName_ << "] " << "Training database..." << std::endl;
        db->computeTfIdfWeights(1);

        std::cout << "[" << moduleName_ << "] " << "Database created!" << std::endl;

        std::cout << "[" << moduleName_ << "] " << "Saving database to " << database_directory << std::endl;
        save_database(database_directory);

        std::cout << "[" << moduleName_ << "] " << "Database ready!" << std::endl;
    }

    private:

        // CONFIGURATION VARIABLES
        std::string moduleName_;
        std::vector<std::string> included_dirs;
        int votes_count;
        int tree_k;
        int tree_levels;
        int min_cluster_size;
        int object_id;

        // DATABASE
        std::vector<vt::Document> docs;
        vt::TreeBuilder<Feature> tree_builder;
        vt::VocabularyTree<Feature> tree;
        vt::Database* db;
        std::vector<std::string> image_names;
        std::map<int, DocumentInfo*> documents_map;
        std::string database_directory;
        std::vector<FeatureVector> images;

// ----------------------------------------------------------------------------------------------------

        void save_database_without_tree(std::string& directory) {
//        std::cout << "[" << moduleName_ << "] " << "Saving documents..." << std::endl;

        std::string documents_file(directory);
        documents_file.append("/images.documents");
        std::ofstream out(documents_file.c_str(), std::ios::out | std::ios::binary);
        size_t map_size = documents_map.size();
        out.write((char*) &map_size, sizeof(size_t));
        std::map<int, DocumentInfo*>::iterator iter;

        for (iter = documents_map.begin(); iter != documents_map.end(); ++iter) {
            // TODO: sometimes the last iter->second->document has size 0, this is just a quick fix
            out.write((char*) &iter->first, sizeof(int));
            iter->second->write(out);
        }

//        std::cout << "[" << moduleName_ << "] " << "Saving weights..." << std::endl;

        std::string weights_file(directory);
        weights_file.append("/images.weights");
        db->saveWeights(weights_file.c_str());
        out.close();
    }

    // ----------------------------------------------------------------------------------------------------

    void save_database(std::string& directory) {
//        std::cout << "[" << moduleName_ << "] " << "Saving the tree..." << std::endl;
        std::string tree_file(directory);
        tree_file.append("/images.tree");
        tree.save(tree_file.c_str());
        save_database_without_tree(directory);
    }

    // ----------------------------------------------------------------------------------------------------

    bool process_file(IplImage* image, std::vector<FeatureVector>& images_learn, bool onlySaveImages) {

        Keypoint keypoints = extract_keypoints(image, false);

//        std::cout << "[" << moduleName_ << "] " << "Keypoints extracted" << std::endl;

        FeatureVector features;
        Keypoint p = keypoints;
        int count = 0;

        while (p != NULL) {
            Feature f(p->descrip);
            features.push_back(f);
            p = p->next;
            ++count;
        }

//        std::cout << "[" << moduleName_ << "] " << count << " features found" << std::endl;

        if (!onlySaveImages && count > 0)
            images_learn.push_back(features);
        else {
//            IplImage *colour_image = cvLoadImage((char*) filename.c_str());
//            p = keypoints;
//            while (p != NULL) {
//                cvCircle(colour_image, cvPoint((int) (p->col), (int) (p->row)), 3, cvScalar(255, 255, 0));
//                p = p->next;
//            }
//            cvSaveImage((char*) filename.c_str(), colour_image);
//            cvReleaseImage(&colour_image);
        }

//        cvReleaseImage(&image);       // TODO cannot release the image after copying it from cv::Mat, do i really need this?
        FreeKeypoints(keypoints);

        // if features were not found, return failure in learning
        return count > 0;
    }

    // ----------------------------------------------------------------------------------------------------

    Keypoint extract_keypoints(IplImage *image, bool frames_only) {

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
};
