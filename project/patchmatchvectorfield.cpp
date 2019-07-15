#include "patchmatchvectorfield.h"
#include "prerequisites.h"

#include <cmath>
#include <Eigen/Eigen>
#include <cstdlib>
#include <optional>
#include <omp.h>

PatchMatchVectorField::PatchMatchVectorField(StereoImage* stereoImage, int width, int height, int patchSize) :
    m_stereoImage(stereoImage),
    m_leftImage(stereoImage->getLeftImageRectified()),
    m_rightImage(stereoImage->getRightImageRectified()),
    m_disparities(stereoImage->getDisparity()),
    m_width(width),
    m_height(height),
    m_patchSize(patchSize)
{
    // fresh start
    m_nearestNeighborField = new Vector2i[m_width * m_height];
    m_patchDistances = new float[m_width * m_height];

    for (int y = m_patchSize/2; y < m_height - m_patchSize/2; y++) {
        for (int x = m_patchSize/2; x < m_width - m_patchSize/2; x++) {
            int idx = y * m_width + x;
            Vector2i current(x, y);
            Vector2i offset(0, 0);

            m_nearestNeighborField[idx] = offset;
            m_patchDistances[idx] = patchDistance(current, offset);
        }
    }
}

PatchMatchVectorField::~PatchMatchVectorField() {
    SAFE_DELETE_ARRAY(m_nearestNeighborField);
    SAFE_DELETE_ARRAY(m_patchDistances);
    SAFE_DELETE_ARRAY(m_disparities);
}

void PatchMatchVectorField::iterativeRun(int iteration) {
    // find optimal patch matches / offsets
    Vector2i position;
    for (int i = 0; i < iteration; i++) {
        for (int y = m_patchSize/2; y < (m_height - m_patchSize/2); y++) {
            for (int x = m_patchSize/2; x < m_width - m_patchSize/2; x++) {
                position = Vector2i(x, y);
                propagate(position);
                randomSearch(position);
                if (indexFromVector(position) % 1000 == 0)
                    progressBar(((float) y * (float) m_width + (float) x) / ((float) m_height * (float) m_width), "PM Iteration " + std::to_string(i + 1));
            }
        }
    }
}

void PatchMatchVectorField::computeDisparities() {
    int i = 0;
    for (int y = 0; y < m_height; y++) {
        for (int x = 0; x < m_width; x++) {
            i = indexFromVector(Vector2i(x, y));
            if (m_patchDistances[i] == INF) {
                m_disparities[i] = MINF;
            }
            else {
                m_disparities[i] = m_nearestNeighborField[i](0); // its enough to retrieve the offset in x direction
            }
        }
    }
}

/**
 * @brief PatchMatchVectorField::propagate compares the offset at position with the offset of the neighbors in order to eventually
 * optimize the offset at position for a better patch match.
 * @param position
 */
void PatchMatchVectorField::propagate(Vector2i position) {
    int i = indexFromVector(position);
    int iNeighborLeft = indexFromVector(position + Vector2i(-1, 0));
    int iNeighborRight = indexFromVector(position + Vector2i(0,-1));
    Vector2i neighborOffsetLeft = m_nearestNeighborField[iNeighborLeft];
    Vector2i neighborOffsetRight = m_nearestNeighborField[iNeighborRight];
    float patchDistanceLeft = patchDistance(position, neighborOffsetLeft);
    float patchDistanceRight = patchDistance(position, neighborOffsetRight);

    if (patchDistanceLeft < m_patchDistances[i]) {
        m_patchDistances[i] = patchDistanceLeft;
        m_nearestNeighborField[i] = neighborOffsetLeft;
    }
    if (patchDistanceRight < m_patchDistances[i]) {
        m_patchDistances[i] = patchDistanceRight;
        m_nearestNeighborField[i] = neighborOffsetRight;
    }
}

void PatchMatchVectorField::randomSearch(Vector2i position) {
    float searchRadius = ALPHA * m_width;
    int i = indexFromVector(position);
    Vector2i randomOffset;
    float patchDistance_;

    while (searchRadius > 1.0f) {
        float r = 2.0f * std::rand() / RAND_MAX - 1.0f;
        randomOffset = Vector2i((int) std::round(r * searchRadius), 0);
        patchDistance_ = patchDistance(position, randomOffset);

        if (patchDistance_ < m_patchDistances[i]) {
            m_patchDistances[i] = patchDistance_;
            m_nearestNeighborField[i] = randomOffset;
        }

        searchRadius *= ALPHA;
    }
}

float PatchMatchVectorField::patchDistance(Vector2i positionLeft, Vector2i offset) {
    Vector2i currentPositionLeft, currentPositionRight, patchOffset;
    Vector4i colorLeft, colorRight;
    float result = 0.0f;

    int validPixelCounter = 0;
    for (int y = -m_patchSize/2; y < m_patchSize/2; y++) {
        for (int x = -m_patchSize/2; x < m_patchSize/2; x++) {
            patchOffset = Vector2i(x, y);
            currentPositionLeft = positionLeft + patchOffset;
            currentPositionRight = positionLeft + offset + patchOffset;
            if (color(m_leftImage, currentPositionLeft, colorLeft) && color(m_rightImage, currentPositionRight, colorRight)) {
                result += (colorLeft - colorRight).norm();
                validPixelCounter++;
            }
            else {
                return INF;
            }
        }
    }

    if (validPixelCounter == 0) {
        return INF;
    }
    return result / validPixelCounter;
}

int PatchMatchVectorField::indexFromVector(Vector2i v) {
    return v(1) * m_width + v(0);
}

bool PatchMatchVectorField::color(std::optional<Pixel> *image, Vector2i position, Vector4i &color) {
    int idx = indexFromVector(position);
    if (image[idx].has_value()) {
        color = image[idx].value().cast<int>(); // does this implicit cast work?
        return true;
    }
    return false;
}
