#include "patchmatch.h"
#include "prerequisites.h"

#include <cmath>
#include <Eigen/Eigen>
#include <cstdlib>
#include <optional>
//#include <omp.h>

#define DISPARITY_INVALID -1
#define NEIGHBORHOOD_INVALID INT_MAX

#define NUM_ITERATIONS 5
#define ALPHA 0.5

PatchMatch::PatchMatch(std::optional<Pixel> *leftImage, std::optional<Pixel> *rightImage, int width, int height, int patchSize) : m_leftImage(leftImage),
                                                                                                    m_rightImage(rightImage),
                                                                                                    m_width(width),
                                                                                                    m_height(height),
                                                                                                    m_patchSize(patchSize)
{
    m_disparity = new int[m_width * m_height];
    m_neighborhood = new int[m_width * m_height];

    for(int i = 0; i < m_height * m_width; i++){
        m_disparity[i] = DISPARITY_INVALID;
        m_neighborhood[i] = NEIGHBORHOOD_INVALID;
    }

    for(int y = patchSize / 2; y < m_height - patchSize / 2; y++){
        for(int x = patchSize / 2; x < m_width - patchSize /2; x++){

            int idx = y * m_width + x;

            if(m_leftImage[idx].has_value()){
                m_disparity[idx] = idx;
                m_neighborhood[idx] = evalNeighborhood(idx, idx);
            }
        }
    }
}

int *PatchMatch::computeDisparity()
{
    for (int i = 0; i < NUM_ITERATIONS; i++){
        std::cout << "Start" << std::endl;
//#pragma omp parallel for
        for (int y = m_patchSize/2; y < m_height - m_patchSize / 2; y++){
            for (int x = m_patchSize / 2; x < m_width - m_patchSize / 2; x++){

                int idx = y * m_width + x;

                if(m_disparity[idx] == DISPARITY_INVALID){
                    continue;
                }

                if (x > 0){
                    propagate(idx);
                }

                randomSearch(y, idx);

                progressBar(((float) y * (float) m_width + (float) x) / ((float) m_height * (float) m_width), "PM Iteration " + std::to_string(i + 1));
            }
        }
    }

    return m_disparity;
}

void PatchMatch::propagate(int idx)
{
    if (m_neighborhood[idx - 1] < m_neighborhood[idx]){
        m_neighborhood[idx] = evalNeighborhood(idx, m_disparity[idx - 1]);
        m_disparity[idx] = m_disparity[idx - 1];
    }
}

void PatchMatch::randomSearch(int row, int idx)
{
    double search_radius = ALPHA * m_width;

    while (search_radius > 1){

        double r = 2.0 * rand() / RAND_MAX - 1;
        int tested_disparity = m_disparity[idx] + ceil(search_radius * r);
        search_radius *= ALPHA;
        int tested_disparity_col = tested_disparity - row * m_width;

        if (tested_disparity_col < 0 || tested_disparity_col >= m_width){
            continue;
        }

        int neighborhood = evalNeighborhood(idx, tested_disparity);

        if (m_neighborhood[idx] > neighborhood){
            m_neighborhood[idx] = neighborhood;
            m_disparity[idx] = tested_disparity;
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
