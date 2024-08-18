#ifndef UI_HPP
#define UI_HPP

#include <opencv2/opencv.hpp>

#define COLOR_RED		cv::Scalar(0, 0, 255)
#define COLOR_GREEN		cv::Scalar(0, 255, 0)
#define COLOR_BLUE		cv::Scalar(255, 0, 0)
#define COLOR_YELLOW	cv::Scalar(0, 255, 255)
#define COLOR_GREY		cv::Scalar(127, 127, 127)

/**
 * @brief A class to encapsulate an OpenCV window for displaying frames.
 *
 * @author Anton Haes
 */
class Window {
public:

    /**
     * @brief Constructor that initializes the OpenCV window with the given name.
     * 
     * @author Anton Haes
     * 
     * @param name The name of the window.
     */
    Window(const std::string& name) : window_name(name) {
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    }
    
    /**
     * @brief Destructor that destroys the OpenCV window.
     * 
     * @author Anton Haes
     */
    ~Window() {
        cv::destroyWindow(window_name);
    }
    
    /**
     * @brief Loads and displays a frame in the OpenCV window.
     * 
     * @author Anton Haes
     *
     * @param frame The frame to display.
     */
    int loadFrame(cv::Mat frame) {
        cv::imshow(window_name, frame);
        return cv::waitKey(1);
    }
    
    /**
     * @brief Loads, resizes, and displays a frame in the OpenCV window.
     * 
     * @author Anton Haes
     *
     * @param frame The frame to display.
     * @param width The width to resize the frame to.
     * @param height The height to resize the frame to.
     */
    int loadFrame(cv::Mat* frame, int width, int height) {
        cv::resize(*frame, *frame, cv::Size(width, height));
        cv::imshow(window_name, *frame);
        return cv::waitKey(1);
    }
    
    /**
     * @brief Loads, resizes, and displays a 2 frames next to each other in the OpenCV window.
     *
     * @author Anton Haes
     * 
     * @param frame_left The first frame to display.
     * @param frame_right The second frame to display
     * @param width The width to resize both frames to.
     * @param height The height to resize both frames to.
     * 
     * @return Return the ASCII code of the key that was pressed, or -1 if no key was pressed.
     */
    int load2Frames(cv::Mat* frame_left, cv::Mat* frame_right, int width, int height) {
        // resize the left frame to match the output size
    	cv::resize(*frame_left, *frame_left, cv::Size(width, height));
        // resize the right frame to match the output size
    	cv::resize(*frame_right, *frame_right, cv::Size(width, height));
        // create a new frame, and fill it with the left and right frame
    	cv::Mat combined_frame(width*2, height, CV_8UC3);
    	cv::hconcat(*frame_left, *frame_right, combined_frame);
        // show the frame
    	cv::imshow(window_name, combined_frame);
        return cv::waitKey(1);
    }

private:
    std::string window_name; // The name of the OpenCV window.
};

#endif // UI_HPP

