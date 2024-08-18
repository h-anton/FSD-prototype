#ifndef CONE_HPP
#define CONE_HPP

#define YELLOW_CONE 0
#define BLUE_CONE 1

/**
 * @struct Cone
 * @brief Represents a detected cone in an image with associated properties.
 * 
 * This struct encapsulates the information related to a cone detected in an image.
 * It includes the type of the cone, its position and dimensions in pixel coordinates, 
 * and the cropped image frame that contains just the cone. Additionally, it stores 
 * keypoints for feature extraction or analysis.
 *
 * @author Anton Haes
 */
struct Cone {
public:    
	int type; // Color of the cone
    int world_coordinates_mm[2]; // x- and y-world coordinates of the cone
    cv::Mat frame; // Cropped image frame that contains the cone, used for keypoint detection
    
    // Top left pixel coordinates of bounding box
    int start_x;
    int start_y;
    // Bottom right pixel coordinates of bounding box
    int end_x;
    int end_y;
    // Width and height of bounding box
    int width;
    int height;
    
    std::vector<std::pair<float, float>> keypoints; // Vector with the keypoints of the cone
    
    // Constructor for cone object
    Cone(int cone_type, int pixel_x, int pixel_y, cv::Mat cone_frame)
        : type(cone_type), start_x(pixel_x), start_y(pixel_y), frame(cone_frame) {
        width = cone_frame.cols;
        height = cone_frame.rows;
        end_x = start_x + width;
        end_y = start_y + height;
    }
};

#endif // Cone_HPP

