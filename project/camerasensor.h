#ifndef CAMERASENSOR_H
#define CAMERASENSOR_H

#include <Eigen/Eigen>

/**
 * @brief The CameraSensor class
 *
 * Left camera, also referred to as camera 1 is in world coordinates (reference frame).
 * The right camera, also referred to as camera 2 is translated to the right.
 * Depth values: Should be relative to the reference frame (left camera, camera 1).
 */

typedef unsigned char BYTE;

class CameraSensor {
public:
    explicit CameraSensor() {
        Init();
    }

    int Init() {
        m_leftImageWidth = 64;
        m_leftImageHeight = 48;
        m_rightImageWidth = 64;
        m_rightImageHeight = 48;

        m_leftIntrinsics << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f, 0.0f, 0.0f, 1.0f; // later: use actual parameter
        m_leftExtrinsics.setIdentity();

        m_rightIntrinsics << 525.0f, 0.0f, 319.5f, 0.0f, 525.0f, 239.5f, 0.0f, 0.0f, 1.0f; // later: use actual parameter
        m_rightExtrinsics <<    1.0f, 0.0f, 0.0f, 7.0f, // translated along x axis by 7, later: use actual parameter
                                0.0f, 1.0f, 0.0f, 0.0f,
                                0.0f, 0.0f, 1.0f, 0.0f,
                                0.0f, 0.0f, 0.0f, 1.0f;
    }

    Eigen::Matrix3f getLeftIntrinsics() {
        return m_leftIntrinsics;
    }
    Eigen::Matrix3f getRightIntrinsics() {
        return m_leftIntrinsics;
    }
    unsigned int getLeftImageWidth() {
        return m_leftImageWidth;
    }
    unsigned int getLeftImageHeight() {
        return m_leftImageHeight;
    }
    unsigned int getRightImageWidth() {
        return m_leftImageWidth;
    }
    unsigned int getRightImageHeight() {
        return m_leftImageHeight;
    }

private:
    Eigen::Matrix3f m_leftIntrinsics;
    Eigen::Matrix4f m_leftExtrinsics;

    Eigen::Matrix3f m_rightIntrinsics;
    Eigen::Matrix4f m_rightExtrinsics;

    unsigned int m_leftImageWidth;
    unsigned int m_leftImageHeight;
    unsigned int m_rightImageWidth;
    unsigned int m_rightImageHeight;
};

#endif // CAMERASENSOR_H
