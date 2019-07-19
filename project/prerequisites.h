#ifndef PREREQUISITES_H
#define PREREQUISITES_H

#include <Eigen/Eigen>
#include <iostream>

// Type definitions

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif
#ifndef INF
#define INF std::numeric_limits<float>::infinity()
#endif
#ifndef INTMIN
#define INTMIN -std::numeric_limits<int>::infinity()
#endif
#ifndef INTMAX
#define INTMAX std::numeric_limits<int>::infinity()
#endif
#ifndef EPSILON
#define EPSILON 0.000001
#endif

typedef Eigen::Matrix<unsigned char, 4, 1> Pixel; // Works as Vector4uc
const static Pixel invalidPixel = Pixel(-1, -1, -1, -1);

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

static void progressBar(float progress, std::string title) {
    int progressBarWidth = 70;
    if (progress < 1.0) {
        std::cout << "\t" << title;
        std::cout << " [";
        int progressBarPos = int(progressBarWidth * progress);
        for (int i = 0; i < progressBarWidth; ++i) {
            if (i <= progressBarPos) std::cout << "=";
            else std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();
    } else{
        std::cout << "\t" << title << " finished";
        std::cout.flush();
        std::cout << std::endl;
    }
}

// for debugging
static void my_print_vector(Eigen::Vector3f v) {
    std::cout << v(0) << ", " << v.y() << ", " << v.z() << std::endl;
}

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(ptr) {if(ptr!=nullptr) {delete[] ptr; ptr = nullptr;}}
#endif

#endif // PREREQUISITES_H
