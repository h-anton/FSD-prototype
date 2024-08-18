#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "BaslerCamera.hpp"

#include "Cone.hpp"
#include "Yolo.hpp"
#include "RektNet.hpp"

#include "Vision3D.hpp"
#include "PathFinding.hpp"

#include "UI.hpp"
#include "DrawMap.hpp"

#include "Car.hpp"

int main(int argc, char** argv) {

	system("sudo nvpmodel -m 4"); // Set the correct power profile
	system("sudo jetson_clocks"); // Set the CPU & GPU clocks to the maximum

    //BaslerCamera camera("/home/anton/Desktop/RIP_FS_Deggendorf.mp4");
    //BaslerCamera camera("/home/anton/Desktop/test-setup-recording-low-fps.mp4");
    BaslerCamera camera("/home/anton/Desktop/recording-fablab.mp4");
    Vision3D vision3D("../models/calibration_data_tower.bin");
    
    //BaslerCamera camera(50000.0);
    //Vision3D vision3D("../models/calibration_data_car.bin");
    
    Yolo yolo("../models/yolo_736x480.trt");
    RektNet rektnet("../models/rektnet_dynamic_fp16.trt");
    
    PathFinding path_finder;
    
    Car robot;
    bool drive = false;
    uint8_t angle = 90;
    uint8_t basespeed = 40;

    Window window("Camera playback");
    
    double average_frame_time = 0;
    long frame_count = 0;
    std::chrono::high_resolution_clock::time_point t0, t1;
    t0 = std::chrono::high_resolution_clock::now();
    
    while(1) {
    	cv::Mat camera_frame = camera.getFrame();
    	if (camera_frame.empty()) break; // End of video file

		std::vector<Cone> cones = yolo.getCones(camera_frame);
		rektnet.getKeypoints(&cones);
		vision3D.calculatePosition(&cones);
		path_finder.findPath(&cones);
		
		// Drive the car
		uint8_t angle = path_finder.calculateDirection();
		if (drive) robot.drive(angle, basespeed, basespeed);
    	if (!drive) robot.drive(angle, 0, 0);
		
		// Draw a map of what the car can see
		cv::Mat map_frame = DrawMap::getMapFrame(&cones, &path_finder.edges, &path_finder.path);
		
		// Draw the bounding boxes around the cones and their keypoints
		for(Cone cone: cones) {
			cv::Point top_left(cone.start_x, cone.start_y);
			cv::Point bottom_right(cone.end_x, cone.end_y);
			if (cone.type == YELLOW_CONE) cv::rectangle(camera_frame, top_left, bottom_right, COLOR_YELLOW, 3);
			if (cone.type == BLUE_CONE) cv::rectangle(camera_frame, top_left, bottom_right, COLOR_BLUE, 3);
			for(std::pair<float, float> keypoint: cone.keypoints) {
				int x = cone.start_x + (int)(keypoint.first*cone.width);
				int y = cone.start_y + (int)(keypoint.second*cone.height);
				cv::circle(camera_frame, cv::Point(x, y), 5, COLOR_RED, -1);
			}
		}
		
		// Measure running time of program
		t1 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double, std::milli> delta_t = t1 - t0;
		double delta_t_ms = delta_t.count();
		double fps = 1000.0 / delta_t_ms;
		average_frame_time += delta_t_ms;
		frame_count += 1;
		std::string frame_time_text = "Frame time: " + std::to_string((int)delta_t_ms) + " ms";
		std::string fps_text = "FPS: " + std::to_string((int)fps);
		cv::putText(camera_frame, frame_time_text, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 2, COLOR_RED, 2, cv::LINE_AA);
		cv::putText(camera_frame, fps_text, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 2, COLOR_RED, 2, cv::LINE_AA);

    	
    	// Display the output
    	int keypress = window.load2Frames(&camera_frame, &map_frame, 736, 480);
    	if (keypress == 27) break; // Press ESC to quit
    	if (keypress == 32) drive = !drive; // Press spacebar to toggle drive
    	
    	t0 = std::chrono::high_resolution_clock::now();
    }
    
    robot.drive(90, 0, 0); // Stop the robot and put the wheels straight
    
    average_frame_time /= (double)frame_count;
    std::cout << "Average frame time: " << (int)average_frame_time << " ms" << std::endl;
    std::cout << "Average FPS: " << (int)(1000.0/average_frame_time) << std::endl;
	return 0;
}
