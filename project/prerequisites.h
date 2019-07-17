#ifndef PREREQUISITES_H
#define PREREQUISITES_H

#include <Eigen/Eigen>
#include <iostream>
#include <fstream>
#include <regex>

// Type definitions

#ifndef MINF
#define MINF -std::numeric_limits<float>::infinity()
#endif
#ifndef INF
#define INF std::numeric_limits<float>::infinity()
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

struct calibrationData {
    float focalLength, baseline;
    int width, height;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix4f rightExtrinsic;
};

static calibrationData readCalibration(std::string filename) {
    std::cout << "Reading calibration file..." << std::endl;
    std::string line;
    std::ifstream myfile(filename);
    calibrationData data;
    data.rightExtrinsic.setIdentity();

    std::regex numbers("-*[0-9]+[.]*[0-9]*");
    std::smatch number_match;

    if (myfile.is_open()) {

        while (std::getline(myfile,line)) {
            if(line.find("focalLength") != std::string::npos) {
                std::regex_search(line, number_match, numbers);
                data.focalLength = atof(number_match.str(0).c_str());
                std::cout << "\tFocal length: " << data.focalLength << ", ";
            }
            else if(line.find("baseline") != std::string::npos) {
                std::regex_search(line, number_match, numbers);
                data.baseline = atof(number_match.str(0).c_str());
                std::cout << "\tBaseline: " << data.baseline << ", ";
            }
            else if(line.find("width") != std::string::npos) {
                std::regex_search(line, number_match, numbers);
                data.width = atof(number_match.str(0).c_str());
                std::cout << "\tWidth: " << data.width << ", ";
            }
            else if(line.find("height") != std::string::npos) {
                std::regex_search(line, number_match, numbers);
                data.height = atof(number_match.str(0).c_str());
                std::cout << "\tHeight: " << data.height << ", ";
            }
            else if(line.find("extrinsic") != std::string::npos) {
                std::cout << "\n\t Extrinsics:\n";
                float a[16] = {0.f};
                for(int i = 0; i < 16; i++) {
                    std::regex_search(line, number_match, numbers);
                    a[i] = atof(number_match.str(0).c_str());
                    std::cout << a[i] << "," << ((i + 1) % 4 == 0 ? "\n\t" : "\t");
                    line = number_match.suffix();
                }
                data.rightExtrinsic << a[0], a[1], a[2], a[3],
                                       a[4], a[5], a[6], a[7],
                                       a[8], a[9], a[10], a[11],
                                       a[12], a[13], a[14], a[15];
            }
        }
        myfile.close();
        std::cout << std::endl;
    }

    return data;
}

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

static int square(int a) {
    return a*a;
}

static int mymax(int a, int b) {
    return a > b ? a : b;
}

static int mymin(int a, int b) {
    return a < b ? a : b;
}

// for debugging
static void my_print_vector(Eigen::Vector3f v) {
    std::cout << v(0) << ", " << v.y() << ", " << v.z() << std::endl;
}

#ifndef SAFE_DELETE_ARRAY
#define SAFE_DELETE_ARRAY(ptr) {if(ptr!=nullptr) {delete[] ptr; ptr = nullptr;}}
#endif

#endif // PREREQUISITES_H
