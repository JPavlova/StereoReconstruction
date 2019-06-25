#include <iostream>
#include <vector>

#include "stereoimage.h"
#include "prerequisites.h"
#include "exporter.h"
#include <FreeImage.h>

// MAIN FILE TO RUN PIPELINE

/**
 * 1. READ IN IMAGES
 *      - convert .png format into array
 *      - read in ground truth, store in array with same indices
 *      -> would propose a "StereoImage" class containing left, right, depth information in arrays
 *      - Input: Path to database
 *      - Output: StereoImage object
 *
 * 2. RECTIFY IMAGES
 *      - rectification algorithm by paper
 *      - Input: StereoImage object
 *      - Output: StereoImage rectified (inherited class?)
 *
 * 3. PATCHMATCH ALGORITHM
 *      - maybe start with simple Block matching algorithm and expand later?
 *      - randomized approach, thus needs few iterations?
 *      -> epipolar geometry at end or in an intermediate step between 3/4
 *      - Input: StereoImage rectified
 *      - Output: Array of Depth values -> Could also be added to StereoImage object
 *
 *      --> at this step comparison to ground truth possible
 *
 * 4. OUTPUT REPRESENTATION
 *      - represent depth values in HSV / greyvalue depthmap
 *      -> optional: create PointCloud mesh
 *      - Input: StereoImage rectified + Array of depth values (easier in one object)
 *      - Output: Image like format, either to save as .png again or stream?
 *
 * **/

int main(int argc, char *argv[])
{
    // Read in data

    // Create StereoImage

    // -- Rectify
    // -- Patchmatch


    // part 4:
    CameraSensor sensor;
    int width = sensor.getLeftImageWidth();
    int height = sensor.getLeftImageHeight();

    // just some temporary "images" to work with
    Pixel tmp_left[width * height];
    Pixel tmp_right[width * height];

    StereoImage testImage(tmp_left, tmp_right, &sensor);

    // point cloud/ backprojection
    Vertex vertices[width * height];
    testImage.backproject_frame(vertices);

    // export point cloud to .off
    writeMesh(vertices, width, height, "./pointcloud.off");
    writeDepthImage(testImage.getDepthImage(), width, height, DEPTH_MODE::GRAY, "./depth.png");

    return 0;
}
