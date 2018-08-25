#include <DLib/include/DUtils/DUtils.h>
#include <DLib/include/DUtilsCV/DUtilsCV.h>
#include <DLib/include/DVision/DVision.h>
#include "loopDetector.h"

// ----------------------------------------------------------------------------

LoopDetector::LoopDetector(const std::string &strVocabularyFile, const std::string &strSettingsFile) {

    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);
    if(!fSettings.isOpened()) {
        std::cerr << "Could not open " << strSettingsFile << " in OrbExtractor constructor.";
    }


    // Set loop detector parameters
    //typename TDetector::Parameters params(m_height, m_width);

    // Parameters given by default are:
    // use nss = true
    // alpha = 0.3
    // k = 3
    // geom checking = GEOM_DI
    // di levels = 0
    mCount = 0;

    // We are going to change these values individually:
    int height = fSettings["Camera.height"];
    int width = fSettings["Camera.width"];
    float frequency = fSettings["Camera.fps"];
    bool use_nss = true; // use normalized similarity score instead of raw score
    float alpha = 0.3; // nss threshold
    int k = 3; // a loop must be consistent with 3 previous matches
    DLoopDetector::GeometricalCheck geom_check = DLoopDetector::GEOM_DI; // use direct index for geometrical checking
    int di_levels = 3; // use three direct index levels

    mpParams = new OrbLoopDetector::Parameters(height, width, frequency, use_nss, alpha, k, geom_check, di_levels);

    // To verify loops you can select one of the next geometrical checkings:
    // GEOM_EXHAUSTIVE: correspondence points are computed by comparing all
    //    the features between the two images.
    // GEOM_FLANN: as above, but the comparisons are done with a Flann structure,
    //    which makes them faster. However, creating the flann structure may
    //    be slow.
    // GEOM_DI: the direct index is used to select correspondence points between
    //    those features whose vocabulary node at a certain level is the same.
    //    The level at which the comparison is done is set by the parameter
    //    di_levels:
    //      di_levels = 0 -> features must belong to the same leaf (word).
    //         This is the fastest configuration and the most restrictive one.
    //      di_levels = l (l < L) -> node at level l starting from the leaves.
    //         The higher l, the slower the geometrical checking, but higher
    //         recall as well.
    //         Here, L stands for the depth levels of the vocabulary tree.
    //      di_levels = L -> the same as the exhaustive technique.
    // GEOM_NONE: no geometrical checking is done.
    //
    // In general, with a 10^6 vocabulary, GEOM_DI with 2 <= di_levels <= 4
    // yields the best results in recall/time.
    // Check the T-RO paper for more information.
    //
    // Load the vocabulary to use
    std::cout << "Loading vocabulary..." << std::endl;
    mpVoc = new OrbVocabulary(strVocabularyFile);

    mpDetector = new OrbLoopDetector(*mpVoc, *mpParams);
    // we can allocate memory for the expected number of images
    //mpDetector->allocate(filenames.size());

    mpExtractor = new OrbExtractor(strSettingsFile);

}

// ---------------------------------------------------------------------------

LoopDetector::~LoopDetector(){
    std::cout << "LoopDetector destructor called." << std::endl;
    delete mpDetector;
    delete mpVoc;
    delete mpExtractor;
    delete mpParams;
}

// ---------------------------------------------------------------------------


void LoopDetector::process(const cv::Mat &im, DLoopDetector::DetectionResult &result) {
    // prepare profiler to measure times
    //DUtils::Profiler profiler;

    // get features
    //profiler.profile("features");
    mpExtractor->operator()(im, mvKeys, mvDescriptors);
    //profiler.stop();

    //profiler.profile("detection");
    mpDetector->detectLoop(mvKeys, mvDescriptors, result);
    //profiler.stop();

/*
       int match = -1;
       if(result.detection())
       {
           match = result.match;
           std::cout << "- Loop found with image " << match << "!"
                << std::endl;
           ++mCount;
       }

       else
       {
           cout << "- No loop: ";
           switch(result.status)
           {
               case DLoopDetector::CLOSE_MATCHES_ONLY:
                   std::cout << "All the images in the database are very recent" << std::endl;
                   break;

               case DLoopDetector::NO_DB_RESULTS:
                   std::cout << "There are no matches against the database (few features in"
                           " the image?)" << std::endl;
                   break;

               case DLoopDetector::LOW_NSS_FACTOR:
                   std::cout << "Little overlap between this image and the previous one"
                        << std::endl;
                   break;

               case DLoopDetector::LOW_SCORES:
                   std::cout << "No match reaches the score threshold (alpha: " <<
                        mpParams->alpha << ")" << std::endl;
                   break;

               case DLoopDetector::NO_GROUPS:
                   std::cout << "Not enough close matches to create groups. "
                        << "Best candidate: " << result.match << std::endl;
                   break;

               case DLoopDetector::NO_TEMPORAL_CONSISTENCY:
                   std::cout << "No temporal consistency (k: " << mpParams->k << "). "
                        << "Best candidate: " << result.match << std::endl;
                   break;

               case DLoopDetector::NO_GEOMETRICAL_CONSISTENCY:
                   std::cout << "No geometrical consistency. Best candidate: "
                        << result.match << std::endl;
                   break;

               default:
                   break;
           }
       }


    std::cout << std::endl;

    if(mCount == 0) {
        std::cout << "No loops found in this image sequence" << std::endl;
    } else {
        std::cout << mCount << " loops found in this image sequence!" << std::endl;
    }


    std::cout << std::endl << "Execution time:" << std::endl
              << " - Feature computation: " << profiler.getMeanTime("features") * 1e3
              << " ms/image" << std::endl
              << " - Loop detection: " << profiler.getMeanTime("detection") * 1e3
              << " ms/image" << std::endl;
*/
}

// ---------------------------------------------------------------------------

void LoopDetector::saveDatabase(const std::string &strDatabaseFile) {
    std::cout << "Saving database to " << strDatabaseFile << "..." << std::endl;
    mpDetector->getDatabase().save(strDatabaseFile);

}

// ---------------------------------------------------------------------------


void LoopDetector::loadDatabase(const std::string &strDatabaseFile) {
    std::cout << "Loading database from " << strDatabaseFile << ". This may take a while..." << std::endl;
    OrbDatabase db;
    db.load(strDatabaseFile);
    mpDetector->setDatabase(db);
}
