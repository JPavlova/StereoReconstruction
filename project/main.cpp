// MAIN FILE TO RUN PIPELINE
// PROPOSED APPROACH (LUKAS)

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

class StereoImage {
	public:
		explicit StereoImage(unsigned char * image_left, unsigned char * image_right);
		void rectify();
		void patchmatch();
		void reconstruct();

	private:
		// raw data
		std::vector<pixel> image_left;
		std::vector<pixel> image_right;
		// final output, empty at the beginning
		std::vector<pixel> image_depth;
		
		// empty at beginning, filled with a rectified copy of the image
		std::vector<pixel> image_left_rect;
		std::vector<pixel> image_right_rect;
		
		// individual features
		std::vector<feature> features_left;
		std::vector<feature> features_right;
	
		// tuple list of features that match
		std::vector<index_tuple> feature_matches;
		
		// point cloud
		std::vector<point> point_cloud;

}
