#include "patchmatch.h"
#include <cmath>
#include <Eigen/Eigen>
#include <cstdlib>
#include "prerequisites.h"
#include <optional>

#define MATCH_INVALID -1
#define DISPARITY_INVALID -1.f
#define NEIGHBORHOOD_INVALID INT_MAX

#define NUM_ITERATIONS 1
#define ALPHA 0.5

/**
 * @brief PatchMatch::PatchMatch
 * @param stereoImage: input whole image, will read all necessary information
 * @param width
 * @param height
 * @param patchSize
 */
PatchMatch::PatchMatch(StereoImage* stereoImage, int width, int height, int patchSize) : m_stereoImage(stereoImage),
                                                                                                    m_leftImage(stereoImage->getLeftImageRectified()),
                                                                                                    m_rightImage(stereoImage->getRightImageRectified()),
                                                                                                    m_leftLookup(stereoImage->getLeftImageLookup()),
                                                                                                    m_rightLookup(stereoImage->getRightImageLookup()),
                                                                                                    m_width(width),
                                                                                                    m_height(height),
                                                                                                    m_patchSize(patchSize),
                                                                                                    m_disparity(stereoImage->getDisparity())
{
    m_matches = new int[m_width * m_height];
    m_neighborhood = new int[m_width * m_height];

    for(int i = 0; i < m_width * m_height; i++){
        if(!m_leftImage[i].has_value()){
            m_matches[i] = MATCH_INVALID;
            m_neighborhood[i] = NEIGHBORHOOD_INVALID;
        } else {
            m_matches[i] = i;
            m_neighborhood[i] = evalNeighborhood(i, i);
        }
    }
}

void PatchMatch::computeDisparity()
{
    for (int i = 0; i < NUM_ITERATIONS; i++){
        for (int y = m_patchSize/2; y < (m_height - m_patchSize / 2); y++){ // divided by 10 to save time... CHANGE BACK
            for (int x = m_patchSize / 2; x < m_width - m_patchSize / 2; x++){
                int idx = y * m_width + x;

                if(m_matches[idx] == MATCH_INVALID){ // das hier macht gar nichts
                    continue;
                }

                if (x > 0){
                    propagate(idx);
                }
                randomSearch(y, idx);

                if ((idx % 1000) == 0)
                    progressBar(((float) y * (float) m_width + (float) x) / ((float) m_height * (float) m_width), "PM Iteration " + std::to_string(i + 1));
            }
        }
    }

    // Compute disparity based on original image indices
    for(int idx = 0; idx < m_width * m_height; idx++) {

        // For invalid matches set disparity to -1!
        if(m_matches[idx] == MATCH_INVALID) {
            m_disparity[idx] = DISPARITY_INVALID;
            continue;
        }

        // TODO: In case disparity is not just difference of x values, rewrite

        int idxLeft = idx;
        int idxRight = m_matches[idx];

        int colLeft = idxLeft % m_width;
        int colRight = idxRight % m_width;

        m_disparity[idx] = (float) colRight - (float) colLeft;

    }
}

void PatchMatch::propagate(int idx)
{
    if (m_neighborhood[idx - 1] < m_neighborhood[idx]){
        m_neighborhood[idx] = evalNeighborhood(idx, m_matches[idx - 1]);
        m_matches[idx] = m_matches[idx - 1];
    }
}

void PatchMatch::randomSearch(int row, int idx)
{
    double search_radius = ALPHA * m_width;

    while (search_radius > 1){

        double r = 2.0 * rand() / RAND_MAX - 1;
        int tested_disparity = m_matches[idx] + ceil(search_radius * r);
        search_radius *= ALPHA;
        int tested_disparity_col = tested_disparity - row * m_width;

        if (tested_disparity_col < 0 || tested_disparity_col >= m_width){
            continue;
        }

        int neighborhood = evalNeighborhood(idx, tested_disparity);

        if (m_neighborhood[idx] > neighborhood){
            m_neighborhood[idx] = neighborhood;
            m_matches[idx] = tested_disparity;
        }
    }
}

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
