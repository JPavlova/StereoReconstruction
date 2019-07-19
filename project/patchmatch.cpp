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

#define NUM_ITERATIONS 20
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
                m_matches[idx] = idx;//y * m_width;
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
        for (int col = m_patchSize/2; col < m_width - m_patchSize / 2; col++){
            for (int row = m_patchSize / 2; row < m_height - m_patchSize / 2; row++){
                int idx = row * m_width + col;

                if(m_matches[idx] == MATCH_INVALID){
                    continue;
                }

                propagate(idx,i);
                randomSearch(idx);
            }
        }
    }

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

        m_disparity[idx] = idxLeft - idxRight;
    }
}

void PatchMatch::propagate(int idx, int iteration)
{
    bool even = iteration % 2 == 0;

    if((idx - 1) % m_width > m_patchSize/2 && !even|| (idx + 1) % m_width < m_width - m_patchSize/2 && even){

        //consider left neighbor for odd iterations
        int idx_propagated = even ? m_matches[idx + 1] : m_matches[idx -1];

        if(idx_propagated != MATCH_INVALID && idx_propagated > idx){

            int neighborhood_propagated = evalNeighborhood(idx, idx_propagated);

            if (neighborhood_propagated < m_neighborhood[idx]){
                m_neighborhood[idx] = neighborhood_propagated;
                m_matches[idx] = idx_propagated;
            }
        }
    }

    if(idx > (m_patchSize/2 + 1) * m_width && !even || idx < (m_height - m_patchSize/2 - 1) * m_width && even){

        //consider bottom neighbor for odd iterations
        int idx_propagated = even ? m_matches[idx + m_width] - m_width : m_matches[idx - m_width] + m_width;

        if(idx_propagated != MATCH_INVALID && idx_propagated > idx){

            int neighborhood_propagated = evalNeighborhood(idx, idx_propagated);

            if(neighborhood_propagated < m_neighborhood[idx]){
                m_neighborhood[idx] = neighborhood_propagated;
                m_matches[idx] = idx_propagated;
            }
        }
    }
}

void PatchMatch::randomSearch(int idx)
{
    int prevMatch = m_matches[idx];
    double search_radius = std::min(ALPHA * m_width, double(prevMatch % m_width - m_patchSize/2));

    while (search_radius > 1){
        // pick a random pixel in current row (only to the left)
        double r = double(rand()) / RAND_MAX;
        int tested_match = prevMatch - ceil(search_radius * r);

        // compute pixel distance and update if patch matches better
        int neighborhood = evalNeighborhood(idx, tested_match);
        if (neighborhood < m_neighborhood[idx]){
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
 *
 * @param center_left
 * @param center_right
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
            else {
                Vector4i pixelDistance = m_leftImage[idx_left].value().cast<int>() - m_rightImage[idx_right].value().cast<int>();
                totalDistance += pixelDistance.dot(pixelDistance);
            }
        }
    }

    return totalDistance;
}
