#include "patchmatch.h"
#include "prerequisites.h"

#include <cmath>
#include <Eigen/Eigen>
#include <cstdlib>
#include <optional>
#include <omp.h>


#define MATCH_INVALID -1
#define DISPARITY_INVALID MINF
#define NEIGHBORHOOD_INVALID INT_MAX

#define NUM_ITERATIONS 2
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

    //initialize correspondences with their own index (exclude borders not reachable by patch match)
#pragma omp parallel for
    for(int y = patchSize / 2; y < m_height - patchSize / 2; y++){
        for(int x = patchSize / 2; x < m_width - patchSize /2; x++){

            int idx = y * m_width + x;

            if(m_leftImage[idx].has_value()){
                m_matches[idx] = idx;
                m_neighborhood[idx] = evalNeighborhood(idx, idx);
            }
        }
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

#pragma omp parallel for
        for (int y = m_patchSize/2; y < m_height - m_patchSize / 2; y++){
            for (int x = m_patchSize / 2; x < m_width - m_patchSize / 2; x++){
                int idx = y * m_width + x;

                if(m_matches[idx] == MATCH_INVALID){
                    continue;
                }
                propagate(idx);
                randomSearch(y, idx);
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
    if(idx - 1 % m_width > 0){
        int leftNeighborhood = evalNeighborhood(idx, m_matches[idx - 1]);
        if (leftNeighborhood < m_neighborhood[idx]){

            m_neighborhood[idx] = leftNeighborhood;
            m_matches[idx] = m_matches[idx - 1];
        }
    }

    if(idx > m_width){
        int idx_propagated = m_matches[idx - m_width] + m_width;
        int aboveNeighborhood = evalNeighborhood(idx, idx_propagated);
        if(aboveNeighborhood < m_neighborhood[idx]){
            m_neighborhood[idx] = aboveNeighborhood;
            m_matches[idx] = idx_propagated;
        }
    }
}

void PatchMatch::randomSearch(int row, int idx)
{
    double search_radius = ALPHA * m_width;

    while (search_radius > 1){
        // pick a random pixel in current row
        double r = 2.0 * rand() / RAND_MAX - 1;
        int tested_match = m_matches[idx] + ceil(search_radius * r);
        int tested_match_col = tested_match - row * m_width;

        // test if random pixel is in image
        if (tested_match_col < m_patchSize/2 || tested_match_col >= m_width - m_patchSize/2){
            continue;
        }

        // compute pixel distance and update if patch matches better
        int neighborhood = evalNeighborhood(idx, tested_match);
        if (m_neighborhood[idx] > neighborhood){
            m_neighborhood[idx] = neighborhood;
            m_matches[idx] = tested_match;
        }

        search_radius *= ALPHA;
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

    for (int i = -m_patchSize / 2; i <= m_patchSize / 2; i++){
        for (int j = -m_patchSize / 2; j <= m_patchSize / 2; j++){

            int idx_left = center_left + i * m_width + j;
            int idx_right = center_right + i * m_width + j;

            if(!m_leftImage[idx_left].has_value() || !m_rightImage[idx_right].has_value()){
                return NEIGHBORHOOD_INVALID;
            }

            Vector4i pixelDistance = m_leftImage[idx_left].value().cast<int>() - m_rightImage[idx_right].value().cast<int>();
            totalDistance += pixelDistance.dot(pixelDistance);
        }
    }

    return totalDistance;
}
