#include "patchmatch.h"
#include <cmath>
#include <Eigen/Eigen>
#include <cstdlib>

#define NUM_ITERATIONS 5
#define ALPHA 0.5

PatchMatch::PatchMatch(Pixel *leftImage, Pixel *rightImage, int width, int height, int patchSize) : m_leftImage(leftImage),
                                                                                                    m_rightImage(rightImage),
                                                                                                    m_width(width),
                                                                                                    m_height(height),
                                                                                                    m_patchSize(patchSize)
{
    //The disparity image is a map of all possible patch centers. Thus some of the outer cells have to be excluded (patch size is assumed to be odd).
    m_width_disparity = width - m_patchSize + 1;
    m_height_disparity = height - m_patchSize + 1;
    m_disparity = new int[m_width_disparity * m_height_disparity];
    memset(m_disparity, 0, m_width_disparity * m_height_disparity * sizeof(int));
}

int *PatchMatch::computeDisparity()
{
    init();
    for (int i = 0; i < NUM_ITERATIONS; i++)
    {
        for (int y = 0; y < m_height_disparity; y++){
            for (int x = 0; x < m_width_disparity; x++){
                int idx_disparity = y * m_width_disparity + x;
                int idx_original = (y + m_patchSize / 2) * m_width + x + m_patchSize / 2;

                if (x > 0){
                    propagate(idx_disparity, idx_original);
                }

                randomSearch(x, y, idx_disparity, idx_original);

            }
        }
    }

    return m_disparity;
}

void PatchMatch::propagate(int idx_disparity, int idx_original)
{
    int leftAssociation = evalNeighborhood(idx_original, idx_original - 1 + m_disparity[idx_disparity - 1]);
    int centerAssociation = evalNeighborhood(idx_original, idx_original + m_disparity[idx_disparity]);

    if (leftAssociation < centerAssociation){
        m_disparity[idx_disparity] = m_disparity[idx_disparity - 1] - 1;
    }
}

void PatchMatch::randomSearch(int x, int y, int idx_disparity, int idx_original)
{
    int best_neighborhood = evalNeighborhood(idx_original, idx_original + m_disparity[idx_disparity]);
    int best_disparity = m_disparity[idx_disparity];
    double search_radius = ALPHA * m_width_disparity;

    while (search_radius > 1){
        double r = 2.0 * rand() / RAND_MAX - 1;
        int tested_disparity = m_disparity[idx_disparity] + ceil(search_radius * r);
        search_radius *= ALPHA;
        int tested_disparity_col = tested_disparity + idx_disparity - y * m_width_disparity;

        if (tested_disparity_col < 0 || tested_disparity_col >= m_width_disparity){
            continue;
        }

        int neighborhood = evalNeighborhood(idx_original, idx_original + tested_disparity);

        if (best_neighborhood > neighborhood){
            best_neighborhood = neighborhood;
            best_disparity = tested_disparity;
        }
    }

    m_disparity[idx_disparity] = best_disparity;
}

int PatchMatch::getDisparityWidth()
{
    return m_width_disparity;
}

int PatchMatch::getDisparityHeight()
{
    return m_height_disparity;
}

int PatchMatch::evalNeighborhood(int center_left, int center_right)
{
    int totalDistance = 0;

    for (int i = -m_patchSize / 2; i <= m_patchSize / 2; i++){
        for (int j = -m_patchSize / 2; j <= m_patchSize / 2; j++){
            Vector4i pixelDistance = m_leftImage[center_left + i * m_width + j].cast<int>() - m_rightImage[center_right + i * m_width + j].cast<int>();
            totalDistance += pixelDistance.dot(pixelDistance);
        }
    }

    return totalDistance;
}
