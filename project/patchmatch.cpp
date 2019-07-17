#include "patchmatch.h"

#define MATCH_INVALID -1
#define DISPARITY_INVALID MINF
#define NEIGHBORHOOD_INVALID INT_MAX

#define NUM_ITERATIONS 5
#define ALPHA 0.2

/**
 * @brief PatchMatch::PatchMatch
 * @param stereoImage: input whole image, will read all necessary information
 * @param width
 * @param height
 * @param patchSize
 */
PatchMatch::PatchMatch(StereoImage* stereoImage, int width, int height, int patchSize) :
    m_stereoImage(stereoImage),
    m_leftImage(stereoImage->getLeftImageRectified()),
    m_rightImage(stereoImage->getRightImageRectified()),
    m_width(width),
    m_height(height),
    m_patchSize(patchSize),
    m_disparity(stereoImage->getDisparity())
{
    m_matches = new int[m_width * m_height];
    m_neighborhood = new int[m_width * m_height];
    //first set all matches to invalid
#pragma omp parallel for
    for(int i = 0; i < m_height * m_width; i++){
        m_matches[i] = MATCH_INVALID;
        m_neighborhood[i] = NEIGHBORHOOD_INVALID;
    }
}

/**
 * @brief PatchMatch::computeDisparity finds correspondences between the left and the right image and sets the disparity
 * in the stereo image class accordingly.
 */
void PatchMatch::computeDisparity()
{
    for (int i = 0; i < NUM_ITERATIONS; i++){

        progressBar((float) i / NUM_ITERATIONS, "Finding correspondences...");

        for (int col = m_patchSize/2; col < m_width - m_patchSize / 2; col++){
            for (int row = m_patchSize / 2; row < m_height - m_patchSize / 2; row++){
                int idx = row * m_width + col;

                if(m_leftImage[idx].has_value()){
                    randomSearch(idx);
                    propagate(idx+1);
                }
            }
        }
    }

    int min_disparity = 0;

    // Compute disparity based on original image indices
#pragma omp parallel for
    for(int idx = 0; idx < m_width * m_height; idx++) {

        // For invalid matches set disparity to -1!
        if(m_matches[idx] == MATCH_INVALID) {
            m_disparity[idx] = DISPARITY_INVALID;
            continue;
        }

        // TODO: In case disparity is not just difference of x values, rewrite
        int idxLeft = idx;
        int idxRight = m_matches[idx];

        m_disparity[idx] = idxRight - idxLeft;

        if(m_disparity[idx] < min_disparity){
            min_disparity = m_disparity[idx];
        }
    }

    for(int idx = 0; idx < m_width * m_height; idx++){
        m_disparity[idx] -= min_disparity;
    }
}

void PatchMatch::propagate(int idx)
{
    if(m_neighborhood[idx-1] == NEIGHBORHOOD_INVALID) return;

    if((idx - 1) % m_width > 0 && m_matches[idx - 1] % m_width > 0){
        int leftNeighborhood = evalNeighborhood(idx, m_matches[idx - 1] + 1);
        if (leftNeighborhood < m_neighborhood[idx]){
            m_neighborhood[idx] = leftNeighborhood;
            m_matches[idx] = m_matches[idx - 1] + 1;
//            std::cout << "Propagated Index " << m_matches[idx - 1] << " -> " << m_matches[idx - 1] + 1 << " from Pixel " << idx-1 << " to " << idx << std::endl;
        }
    }
}

void PatchMatch::randomSearch(int idx)
{
//    double search_radius = m_width - m_patchSize;
    int rowEnd = (idx/m_width+1)*m_width - m_patchSize/2;
    int lower = idx;
    int higher = mymax(rowEnd, rowEnd);
    int range = higher - lower;

    for (int iter = 0; iter < 15; ++iter) {
        // pick a random pixel in current row
        double r = 1.0 * rand() / RAND_MAX;
        int tested_match = lower + range * r;
//        std::cout << "Pixel: " << idx << " Random nr: " << r << ", tested match: " << tested_match << ", tested match col: " << range * r << std::endl;

        // compute pixel distance and update if patch matches better
        int neighborhood = evalNeighborhood(idx, tested_match);
        if (neighborhood < m_neighborhood[idx]){
            m_neighborhood[idx] = neighborhood;
            m_matches[idx] = tested_match;
//            std::cout << "Found new match for " << idx << ": " << tested_match << std::endl;
        }
    }
}

/**
 * @brief PatchMatch::evalNeighborhood
 * This method calculates the summed pixel distances between two patches. Patches are identified by their centers.
 * This method returns NEIGHBORHOOD_INVALID if any pixel of the patch has no value.
 * TODO: maybe change that?-> Still calculate the pixel distance for invalid patches
 *
 * @param center_left
 * @param center_right
 * @return
 */
int PatchMatch::evalNeighborhood(int center_left, int center_right)
{

    int totalDistance = 0;
    bool invalid = false;
#pragma omp parallel for collapse(2)
    for (int i = -m_patchSize / 2; i <= m_patchSize / 2; i++){
        for (int j = -m_patchSize / 2; j <= m_patchSize / 2; j++){

            int idx_left = center_left + i * m_width + j;
            int idx_right = center_right + i * m_width + j;

            if(!m_leftImage[idx_left].has_value() || !m_rightImage[idx_right].has_value() || invalid == true){
                invalid = true;
            }
            else {
                Vector4i pixelDistance = m_leftImage[idx_left].value().cast<int>() - m_rightImage[idx_right].value().cast<int>();
                totalDistance += pixelDistance.dot(pixelDistance);
            }
        }
    }

    return invalid ? NEIGHBORHOOD_INVALID : totalDistance;
}
