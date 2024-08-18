#ifndef BASLER_CAMERA_HPP
#define BASLER_CAMERA_HPP

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>

/**
 * @class BaslerCamera
 * @brief A class to interface with a Basler camera or emulate a camera using a video file.
 * 
 * This class allows users to either interface with a physical Basler camera
 * or emulate a camera by reading frames from a video file. It provides
 * functionality to initialize the camera, retrieve frames, and handle resources
 * properly.
 *
 * @author Anton Haes
 */
class BaslerCamera {
public:
    // timeout when grabbing frames from the camera
    int camera_timeout = 5000;
	
	/**
     * @brief Constructs a BaslerCamera object for a real camera with the specified exposure time.
     * 
     * Initializes the Basler camera, sets it to continuous acquisition mode, and
     * configures the exposure time.
     *
     * @author Anton Haes
     * 
     * @param exposure_time The exposure time for the camera in microseconds.
     * @throws std::runtime_error if the camera initialization fails.
     */
    BaslerCamera(float exposure_time) : is_emulated(false) {
    	// Initialize Basler camera using Pylon API
        Pylon::PylonInitialize();
        camera = new Pylon::CInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
        // sets up free-running continuous acquisition.
        camera->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        // configure exposure time
        GenApi::INodeMap& nodemap = camera->GetNodeMap();
        Pylon::CFloatParameter(nodemap, "ExposureTime").SetValue(exposure_time);
        // Specify the output pixel format.
        format_converter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    }
	
	/**
     * @brief Constructs a BaslerCamera object to emulate a camera using a video file.
     * 
     * Opens the specified video file for reading frames.
     *
     * @author Anton Haes
     * 
     * @param path_to_video_file The path to the video file to be used for emulation.
     * @throws std::runtime_error if the video file cannot be opened.
     */
    BaslerCamera(const std::string path_to_video_file) : is_emulated(true) {
    	// Use OpenCV library to read and playback video file
        if (!video_capture.open(path_to_video_file)) {
            throw std::runtime_error("Error opening video file.");
        }
    }

	/**
     * @brief Destructor for the BaslerCamera class.
     * 
     * Closes the camera and releases resources if the camera is not in emulation mode.
     *
     * @author Anton Haes
     */
    ~BaslerCamera() {
        if (!is_emulated) {
            camera->Close();
            delete camera;
            Pylon::PylonTerminate();
        }
    }
	
	/**
     * @brief Retrieves the next frame from the camera or video file.
     * 
     * If the camera is being emulated, it reads the next frame from the video file.
     * If the camera is real, it grabs the latest frame from the camera.
     *
     * @author Anton Haes
     * 
     * @return cv::Mat The captured frame as an OpenCV Mat object. If the end of the video file is reached, it will return an empty frame.
     * @throws std::runtime_error if a frame cannot be retrieved or if the camera is not grabbing frames.
     */
    cv::Mat getFrame() {
        // Case where the camera is being emulated
        if (is_emulated) {
            cv::Mat frame;
            // Read next frame from video
            bool successful = video_capture.read(frame);
            if (!successful) {
                return cv::Mat(); // Return an empty Mat to indacte the end of the video file
            }
            return frame;
        }

        // Case where the camera is not being emulated
        if (!camera->IsGrabbing()) {
            throw std::runtime_error("Camera is currently not grabbing frames.");
        }
        // Wait for an image and then retrieve it
        camera->RetrieveResult(camera_timeout, ptr_grab_result, Pylon::TimeoutHandling_ThrowException);
        // Check if the image was grabbed successfully
        if (!ptr_grab_result->GrabSucceeded()) {
            throw std::runtime_error("Image not grabbed successfully from camera.");
        }
        // Convert image to pylonImage
        format_converter.Convert(pylon_image, ptr_grab_result);
        // Convert image to OpenCV Mat
        cv::Mat frame(ptr_grab_result->GetHeight(), ptr_grab_result->GetWidth(), CV_8UC3, (uint8_t*)pylon_image.GetBuffer());
        // Copy the frame to ensure memory safety
        frame = frame.clone();
        return frame;
    }

private:
    bool is_emulated; // variable to indicate if the camera is being emulated with a video file
    
    // Pylon objects for Basler camera
    Pylon::CInstantCamera* camera;
    Pylon::CGrabResultPtr ptr_grab_result;
    Pylon::CPylonImage pylon_image;
    Pylon::CImageFormatConverter format_converter;
    
    // OpenCV object to read video file
    cv::VideoCapture video_capture;
};

#endif // BASLER_CAMERA_HPP

