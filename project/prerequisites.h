#ifndef PREREQUISITES_H
#define PREREQUISITES_H

#include <Eigen/Eigen>

// Type definitions

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif

typedef Eigen::Matrix<unsigned char, 4, 1> Pixel; // Works as Vector4uc
typedef float Feature;

enum DEPTH_MODE {
    GRAY,
    HSV
};

struct Vertex {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector4f position;
    Pixel color;
};

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(ptr) {if(ptr!=nullptr) {delete[] ptr; ptr = nullptr;}}
#endif

#endif // PREREQUISITES_H
