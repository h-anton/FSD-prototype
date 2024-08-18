#include <opencv2/opencv.hpp>

#include "BaslerCamera.hpp"
#include "UI.hpp"
#include "Calibration.hpp"

Chessboard lower_chessboard_tower = {
	.start_pixel = {685, 675},
	.pixel_width = 580,
	.pixel_height = 275,
	.world_coordinates_mm = {-30, 20, 5}
};
Chessboard upper_chessboard_tower = {
	.start_pixel = {720, 500},
	.pixel_width = 520,
	.pixel_height = 175,
	.world_coordinates_mm = {-30, 60, 15}
};

Chessboard lower_chessboard_car = {
	.start_pixel = {685, 500},
	.pixel_width = 580,
	.pixel_height = 160,
	.world_coordinates_mm = {-30, 20, 26}
};
Chessboard upper_chessboard_car = {
	.start_pixel = {775, 365},
	.pixel_width = 420,
	.pixel_height = 115,
	.world_coordinates_mm = {-30, 60, 36}
};

int main(int argc, char** argv) {

    Chessboard *chessboards_tower[] = {&lower_chessboard_tower, &upper_chessboard_tower, NULL};
    Chessboard *chessboards_car[] = {&lower_chessboard_car, &upper_chessboard_car, NULL};
    
    //BaslerCamera camera(20000.0f); // Initialize with exposure time of 20000 microseconds
    //BaslerCamera camera("/home/anton/Desktop/recording-fablab.mp4");
    BaslerCamera camera("/home/anton/Desktop/test-setup-recording-low-fps.mp4");
    
    Calibration calibration(&camera);
    calibration.findChessboardCorners(chessboards_tower);
    calibration.calibrate(chessboards_tower);
    calibration.save("../thesis-tower-calibration.bin");
    
    Window window("Camera playback");

    while (true) {
        cv::Mat frame = camera.getFrame();
        
        calibration.drawChessboardCorners(&frame, chessboards_tower);
        
        int keypress = window.loadFrame(frame, 768, 480);
        if (keypress >= 0) break; // Exit on any key press
    }

	return 0;
}
