#ifndef DRAW_MAP_HPP
#define DRAW_MAP_HPP

#include <opencv2/opencv.hpp>

#include "UI.hpp"
#include "Cone.hpp"
#include "Track.hpp"


// Private functions
namespace {
	
	/**
	 * @brief Draws cones on a given frame.
	 * 
	 * This function draws cones on the provided frame using their 3D world coordinates. Each cone
	 * is represented as a filled circle with a color determined by its type.
	 * 
	 * @author Anton Haes
	 *
	 * @param frame A pointer to the `cv::Mat` frame where the cones will be drawn.
	 * @param cones A pointer to a vector of `Cone` objects representing the detected cones. 
	 *              Each cone is drawn as a circle with a color corresponding to its type.
	 */
    void drawCones3D(cv::Mat* frame, std::vector<Cone>* cones) {
		int point_thickness = 10;
		
		for(Cone cone: *cones) {
			// Get the color of the cone
			cv::Scalar color;
			if (cone.type == YELLOW_CONE) color = COLOR_YELLOW;
			if (cone.type == BLUE_CONE) color = COLOR_BLUE;
			// Get the coordinates of the cone
			int point_x = cone.world_coordinates_mm[0] + frame->cols/2;
			int point_y = frame->rows - cone.world_coordinates_mm[1];
			cv::Point point = cv::Point(point_x, point_y);
			// Draw the cone
			cv::circle(*frame, point, point_thickness, color, -1);
		}
	}
	
	/**
	 * @brief Draws the track boundaries on a given frame.
	 * 
	 * This function draws edges on the provided frame, representing boundaries of the track. 
	 * Each edge is drawn as a line between two points.
	 *
	 * @author Anton Haes
	 * 
	 * @param frame A pointer to the `cv::Mat` frame where the edges will be drawn.
	 * @param edges A pointer to a vector of `Edge<double>` objects representing the edges to be drawn. 
	 */
	void drawEdges(cv::Mat* frame, std::vector<Edge<double>>* edges) {
		int line_thickness = 3;
		
		for (Edge<double> edge: *edges) {
			// We only draw the boundaries of the track
			if (edge.same_color) {
				// Get the start and end point of each edge
				int start_point_x = edge.point1.getX() + frame->cols/2;
				int start_point_y = frame->rows - edge.point1.getY();
				cv::Point start_point(start_point_x, start_point_y);
				int end_point_x = edge.point2.getX() + frame->cols/2;
				int end_point_y = frame->rows - edge.point2.getY();
				cv::Point end_point(end_point_x, end_point_y);
				// Get the color of the track boundary
				cv::Scalar color;
				if (edge.point1.color == YELLOW_CONE) color = COLOR_YELLOW;
				if (edge.point1.color == BLUE_CONE) color = COLOR_BLUE;
				// Draw the edge
				cv::line(*frame, start_point, end_point, color, line_thickness, cv::LINE_8);
			}
		}
	}
	
	/**
	 * @brief Draws the path the car should follow on a given frame.
	 * 
	 * This function draws the path the car should follow on the provided
	 * frame by connecting a series of points with lines. 
	 * The path is visualized in red.
	 *
	 * @author Anton Haes
	 * 
	 * @param frame A pointer to the `cv::Mat` frame where the path will be drawn.
	 * @param path A pointer to a vector of `Point<int>` objects representing the points of the path. 
	 */
	void drawPath(cv::Mat* frame, std::vector<Point<int>>* path) {
		int line_thickness = 3;
		
		for(int i = 0; i < path->size()-1; i++) {
			// Get the start and end point of each edge in the path
			int start_point_x = (*path)[i].getX() + frame->cols/2;
			int start_point_y = frame->rows - (*path)[i].getY();
			cv::Point start_point = cv::Point(start_point_x, start_point_y);
			int end_point_x = (*path)[i+1].getX() + frame->cols/2;
			int end_point_y = frame->rows - (*path)[i+1].getY();
			cv::Point end_point = cv::Point(end_point_x, end_point_y);
			// Draw the edge
			cv::line(*frame, start_point, end_point, COLOR_RED, line_thickness, cv::LINE_8);
		}
	}
}

// Public functions
namespace DrawMap {

	/**
	 * @brief Creates a frame with cones, track boundaries, and a path drawn on it.
	 * 
	 * This function generates a map frame and draws the cones, track boundaries, and path 
	 * onto it. The frame is created with predefined dimensions and background color, and
	 * then updated with the visual representations of the cones, track boundaries, and path
	 * using the respective drawing functions.
	 *
	 * @author Anton Haes
	 * 
	 * @param cones A pointer to a vector of `Cone` objects representing the detected cones to be drawn.
	 * @param edges A pointer to a vector of `Edge<double>` objects representing the edges to be drawn.
	 * @param path A pointer to a vector of `Point<int>` objects representing the path to be drawn.
	 * 
	 * @return A `cv::Mat` containing the final frame with the cones, edges, and path drawn on it.
	 */
    cv::Mat getMapFrame(std::vector<Cone>* cones, std::vector<Edge<double>>* edges, std::vector<Point<int>>* path) {
        int frame_width = 736*3;
		int frame_height = 480*3;
		cv::Mat frame(frame_height, frame_width, CV_8UC3, COLOR_GREY);
		
		drawCones3D(&frame, cones);
		drawEdges(&frame, edges);
		drawPath(&frame, path);
		
		return frame;
    }
}

    
#endif // DRAW_MAP_HPP

