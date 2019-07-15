#ifndef PATCHMATCHVECTORFIELD_H
#define PATCHMATCHVECTORFIELD_H

#include "stereoimage.h"
#include <optional>

#define ALPHA 0.5

class PatchMatchVectorField
{
public:
    explicit PatchMatchVectorField(StereoImage* stereoImage, int width, int height, int patchSize);
    ~PatchMatchVectorField();

    void iterativeRun(int iteration);
    void computeDisparities();

private:
    void propagate(Vector2i position);
    void randomSearch(Vector2i position);
    float patchDistance(Vector2i positionA, Vector2i offset);
    int indexFromVector(Vector2i v);
    bool color(std::optional<Pixel> *image, Vector2i position, Vector4i &color);

    StereoImage* m_stereoImage;
    std::optional<Pixel> *m_leftImage;
    std::optional<Pixel> *m_rightImage;
    int m_width, m_height, m_patchSize;

    Vector2i *m_nearestNeighborField;
    float *m_patchDistances;
    float *m_disparities;
};

#endif // PATCHMATCHVECTORFIELD_H
