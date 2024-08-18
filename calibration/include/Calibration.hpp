#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <fstream> // library to write and read from files
#include <Dense> // Eigen library for matrices

#include "BaslerCamera.hpp"
#include "UI.hpp"

/**
 * @struct Chessboard
 * @brief Represents a chessboard used for camera calibration.
 * 
 * This structure contains information about the chessboard pattern, including its pixel coordinates,
 * dimensions, and real-world location. It is used in the camera calibration process to map
 * real-world coordinates to pixel coordinates and vice versa.
 * 
 * @author Anton Haes
 */
struct Chessboard {
	/**
	 * @brief x, y pixel coordinates of upper left corner of chessboard_region
	 */
	int start_pixel[2];
	/**
	 * @brief pixel width of the whole chessboard
	 */
	int pixel_width;
	/**
	 * @brief pixel height of the whole chessboard
	 */
	int pixel_height;
	/**
	 * @brief x, y and z coordinates of upper left corner (= first element of std::vector<> corners)
	 */
	int world_coordinates_mm[3];
	/**
	 * @brief Number of squares on the chessboard, in the horizontal direction
	 */
	int horizontal_squares = 8;
	/**
	 * @brief Number of squares on the chessboard, in the vertical direction
	 */
	int vertical_squares = 4;
	/**
	 * @brief Real world size of each square, in millimeters
	 */
	int square_size_mm = 10;
	/**
	 * @brief Vector containing all the world coordinates of the Chessboard
	 */
	std::vector<cv::Point2f> corners;
};

/**
 * @struct CalibrationData
 * @brief Contains data related to camera calibration.
 * 
 * This structure holds the intrinsic and extrinsic parameters of the camera obtained from the calibration process.
 * It includes the projection matrix, inverse intrinsic matrix, inverse rotation matrix, translation vector, and image dimensions.
 * 
 * @author Anton Haes
 */
struct CalibrationData {
	/**
     * @brief The projection matrix of the camera.
     */
	Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, 4);
	/**
     * @brief The inverse of the intrinsic matrix of the camera.
     */
	Eigen::MatrixXd K_inv;
	/**
     * @brief The inverse of the rotation matrix of the camera.
     */
	Eigen::MatrixXd R_inv;
	/**
     * @brief The translation vector of the camera.
     */
	Eigen::MatrixXd t;
	/**
     * @brief Pixel width of the calibrated camera.
     */
	int pixel_width = 1920;
	/**
     * @brief Pixel height of the calibrated camera.
     */
	int pixel_height = 1200;
};

/**
 * @class Calibration
 * @brief Performs camera calibration using a series of chessboard patterns.
 * 
 * This class handles the process of camera calibration by finding chessboard corners in images,
 * computing the camera's intrinsic and extrinsic parameters, and saving these parameters to a file.
 * It also provides functionality to visualize the detected chessboard corners.
 * 
 * @author Anton Haes
 */
class Calibration {
public:
    
    /**
     * @brief Constructs a Calibration object with the given BaslerCamera.
     * 
     * @author Anton Haes
     * 
     * @param cam Pointer to a BaslerCamera object used to capture images for calibration.
     */
    Calibration(BaslerCamera* cam) : camera(cam) {}
    
    // Destructor for the Vision3D class
    ~Calibration() {}
    
    /**
     * @brief Finds and records chessboard corners in images.
     * 
     * This method iterates through an array of Chessboard objects and captures images from the camera
     * to find chessboard corners within specified regions. It displays the frames with detected corners
     * and adjusts corner coordinates based on the full frame.
     * 
     * @author Anton Haes
     * 
     * @param chessboards Array of pointers to Chessboard objects, each representing a calibration pattern.
     */
    void findChessboardCorners(Chessboard *chessboards[]) {
    	Window window("Calibration");
    	
    	for (int i = 0; chessboards[i] != NULL; i++) {
			bool found_chessboard = false;
			int start_x = chessboards[i]->start_pixel[0];
			int start_y = chessboards[i]->start_pixel[1];
			int width = chessboards[i]->pixel_width;
			int height = chessboards[i]->pixel_height;
			cv::Rect chessboard_region(start_x, start_y, width, height);
			cv::Size chessboard_size = cv::Size(chessboards[i]->horizontal_squares-1, chessboards[i]->vertical_squares-1);
			
			while (!found_chessboard) {
				cv::Mat frame = camera->getFrame();
				found_chessboard = cv::findChessboardCorners(frame(chessboard_region), chessboard_size , chessboards[i]->corners, cv::CALIB_CB_ADAPTIVE_THRESH);
				
				cv::rectangle(frame, chessboard_region, COLOR_GREEN, 3);
				window.loadFrame(frame, 768, 480);
			}
			
			// Change chessboard coordinates to full frame instead of chessboard_region
			for (size_t j = 0; j < chessboards[i]->corners.size(); j++) {
				chessboards[i]->corners[j].x += start_x;
				chessboards[i]->corners[j].y += start_y;
			}
			
			int index_top_left = 0;
			int index_top_right = chessboards[i]->horizontal_squares - 2;
			int index_bottom_left = (chessboards[i]->horizontal_squares-1) * (chessboards[i]->vertical_squares-2);
			
			// mirror chessboard so that x coordinates go from low to high
			if (chessboards[i]->corners[index_top_left].x > chessboards[i]->corners[index_top_right].x) {
				chessboards[i]->corners = mirrorXAxis(chessboards[i]);
			}
			// mirror chessboard so that y coordinates go from low to high
			if (chessboards[i]->corners[index_top_left].y > chessboards[i]->corners[index_bottom_left].y) {
				chessboards[i]->corners = mirrorYAxis(chessboards[i]);
			}	
		}
			
    }
    
    /**
     * @brief Performs camera calibration using the detected chessboard corners.
     * 
     * This method computes the camera's calibration parameters based on the corners detected in the chessboard images.
     * It constructs matrices for calibration, performs matrix decompositions, and computes the intrinsic and extrinsic parameters.
     * 
     * @author Anton Haes
     * 
     * @param chessboards Array of pointers to Chessboard objects containing the detected corners and chessboard details.
     */
    void calibrate(Chessboard* chessboards[]) {
    	// Count number of calibration points
    	int num_points = 0;
		for (int i = 0; chessboards[i] != NULL; i++) {
			num_points += chessboards[i]->corners.size();
		}
		
		// Create and fill Matrix A (Section 4.2 from preparatory research training)
		Eigen::MatrixXd A(num_points*2, 12);
		for (int k = 0; chessboards[k] != NULL; k++) {
			int num_cols = chessboards[k]->horizontal_squares-1;
			int num_rows = chessboards[k]->vertical_squares-1;
			double Ly = (double)chessboards[k]->world_coordinates_mm[1];
			double Lz = (double)chessboards[k]->world_coordinates_mm[2];
			for (int i = 0; i < num_rows; i++) {
				double Lx = (double)chessboards[k]->world_coordinates_mm[0]; // FIX THIS
		    	for (int j = 0; j < num_cols; j++) {
		    		int u = chessboards[k]->corners[i * num_cols + j].x;
		    		int v = chessboards[k]->corners[i * num_cols + j].y;
		    		
		    		int index = (k*chessboards[k]->corners.size()) + (i * num_cols + j);
		    		A.row(2*index) << Lx, Ly, Lz, 1, 0, 0, 0, 0, -u*Lx, -u*Ly, -u*Lz, -u;
		    		A.row(2*index+1) << 0, 0, 0, 0, Lx, Ly, Lz, 1, -v*Lx, -v*Ly, -v*Lz, -v;
		    		
		    		Lx += (double)chessboards[k]->square_size_mm;
		    	}
		    	Ly -= (double)chessboards[k]->square_size_mm;
		    }
		}
		
		// Find Eigenvectors and Eigenvalues (Section 4.2 from preparatory research training)
		Eigen::MatrixXd ATA = A.transpose() * A;
		Eigen::EigenSolver<Eigen::MatrixXd> es;
		Eigen::VectorXd eigenvalues(12, 1);
		Eigen::MatrixXd eigenvectors(12, 12);
		
		es.compute(ATA, true);
		eigenvalues = es.eigenvalues().real();
		eigenvectors = es.eigenvectors().real();
		
		// Find Eigenvalue and Eigenvector combination with smallest Loss value (Section 4.2 from preparatory research training)
		float min = FLT_MAX;
		uint8_t best_index = 0;
		for (uint8_t i = 0; i < 12; i++) {
			float lambda = eigenvalues.row(i).coeff(0, 0);
			Eigen::MatrixXd x_normalized = eigenvectors.col(i);
			float loss_func = (x_normalized.transpose() * ATA * x_normalized).coeff(0, 0) - lambda * ((x_normalized.transpose() * x_normalized).coeff(0, 0) - 1);
			if (loss_func < min) {
				best_index = i;
			}
		}
		
		// Fill matrix P
		Eigen::VectorXd p = es.eigenvectors().real().col(best_index);
		for (uint8_t i = 0; i < 3; i++) {
			for (uint8_t j = 0; j < 4; j++) {
				calibration_data.P(i, j) = p(i*4+j);
			}
		}
		
		// Create a new matrix P_1 by omitting the last column of P (section 3.1 from bachelor thesis)
		Eigen::MatrixXd P_1 = calibration_data.P.leftCols(calibration_data.P.cols() - 1);
		// Create a new matrix P_2 by extracting the last column of P (section 3.1 from bachelor thesis)
		Eigen::MatrixXd P_2 = calibration_data.P.col(calibration_data.P.cols() - 1);
		
		// RQ-Decomposition of P_1 (according to section 2 of bachelor thesis)
		Eigen::MatrixXd pAT = P_1.colwise().reverse().transpose(); // cfr. Equation 1 of bachelor thesis
		Eigen::HouseholderQR<Eigen::MatrixXd> qr(pAT); // Perform QR-Decomposition
		Eigen::MatrixXd q_tilde = qr.householderQ();
		Eigen::MatrixXd r_tilde = qr.matrixQR().triangularView<Eigen::Upper>();
		
		// Equation 8 from bachelor thesis
		Eigen::MatrixXd Q = q_tilde.transpose().colwise().reverse();
		Eigen::MatrixXd R = r_tilde.transpose().colwise().reverse().rowwise().reverse();
		
		// fill calibration_data
		calibration_data.K_inv = R.inverse();
		calibration_data.R_inv = Q.inverse();
		calibration_data.t = calibration_data.K_inv * P_2; // see Equation 9 from bachelor thesis
    }
    
    /**
     * @brief Saves the calibration data to a file.
     * 
     * This method writes the camera calibration parameters, including the projection matrix, intrinsic matrix,
     * rotation matrix, and translation vector, to a specified binary file. It also includes image dimensions.
     * 
     * @author Anton Haes
     * 
     * @param filename The name of the file where calibration data will be saved.
     * 
     * @throw std::runtime_error If the file cannot be opened or created for writing.
     */
	void save(const std::string& filename) {
		// Create file
		std::ofstream file(filename, std::ios::out | std::ios::binary);
		if (!file.is_open()) {
		    throw std::runtime_error("Failed to open or create file for writing.");
		}

		// Write pixel width and height
		file.write(reinterpret_cast<const char*>(&calibration_data.pixel_width), sizeof(int));
		file.write(reinterpret_cast<const char*>(&calibration_data.pixel_height), sizeof(int));

		// Write matrix P
		int rows, cols;
		rows = calibration_data.P.rows();
		cols = calibration_data.P.cols();
		file.write(reinterpret_cast<const char*>(&rows), sizeof(int));
		file.write(reinterpret_cast<const char*>(&cols), sizeof(int));
		file.write(reinterpret_cast<const char*>(calibration_data.P.data()), rows * cols * sizeof(double));

		// Write matrix K_inv
		rows = calibration_data.K_inv.rows();
		cols = calibration_data.K_inv.cols();
		file.write(reinterpret_cast<const char*>(&rows), sizeof(int));
		file.write(reinterpret_cast<const char*>(&cols), sizeof(int));
		file.write(reinterpret_cast<const char*>(calibration_data.K_inv.data()), rows * cols * sizeof(double));
		
		// Write matrix R_inv
		rows = calibration_data.R_inv.rows();
		cols = calibration_data.R_inv.cols();
		file.write(reinterpret_cast<const char*>(&rows), sizeof(int));
		file.write(reinterpret_cast<const char*>(&cols), sizeof(int));
		file.write(reinterpret_cast<const char*>(calibration_data.R_inv.data()), rows * cols * sizeof(double));
		
		// Write matrix t
		rows = calibration_data.t.rows();
		cols = calibration_data.t.cols();
		file.write(reinterpret_cast<const char*>(&rows), sizeof(int));
		file.write(reinterpret_cast<const char*>(&cols), sizeof(int));
		file.write(reinterpret_cast<const char*>(calibration_data.t.data()), rows * cols * sizeof(double));

		file.close();
	}
    
    /**
     * @brief Draws detected chessboard corners on the provided image.
     * 
     * This method visualizes the detected chessboard corners by drawing circles around them on the given image frame.
     * 
     * @author Anton Haes
     * 
     * @param frame Pointer to a cv::Mat object representing the image where the corners will be drawn.
     * @param chessboards Array of pointers to Chessboard objects containing the detected corners.
     */
    void drawChessboardCorners(cv::Mat* frame, Chessboard* chessboards[]) {
    	for (int i = 0; chessboards[i] != NULL; i++) {
			for (size_t j = 0; j < chessboards[i]->corners.size(); j++) {
				cv::circle(*frame, chessboards[i]->corners[j], 5, COLOR_GREEN, -1);
			}
		}
    }
    


private:
	BaslerCamera* camera;
	CalibrationData calibration_data;
	
	/**
     * @brief Mirrors chessboard corners along the X-axis.
     * 
     * This method mirrors the positions of the chessboard corners along the X-axis to correct orientation issues.
     * The need for this function comes from the fact that the cv::findChessboardCorners does not always return
     * the chessboard_corners in the same orientation.
     * 
     * @author Anton Haes
     * 
     * @param chessboard Pointer to a Chessboard object containing the corners to be mirrored.
     * @return A vector of mirrored cv::Point2f representing the updated corner positions.
     */
	std::vector<cv::Point2f> mirrorXAxis(Chessboard* chessboard) {
		std::vector<cv::Point2f> mirroredVector(chessboard->corners.size());
		
		int num_cols = chessboard->horizontal_squares-1;
		int num_rows = chessboard->vertical_squares-1;
		
		for (int i = 0; i < num_rows; i++) {
		    for (int j = 0; j < num_cols; j++) {
		    	mirroredVector[i * num_cols + j] = chessboard->corners[i * num_cols + num_cols - 1 - j];
		    }
		}

		return mirroredVector;
	}

	/**
     * @brief Mirrors chessboard corners along the Y-axis.
     * 
     * This method mirrors the positions of the chessboard corners along the Y-axis to correct orientation issues.
     * The need for this function comes from the fact that the cv::findChessboardCorners does not always return
     * the chessboard_corners in the same orientation.
     * 
     * @author Anton Haes
     * 
     * @param chessboard Pointer to a Chessboard object containing the corners to be mirrored.
     * @return A vector of mirrored cv::Point2f representing the updated corner positions.
     */
	std::vector<cv::Point2f> mirrorYAxis(Chessboard* chessboard) {
		std::vector<cv::Point2f> mirroredVector(chessboard->corners.size());
		
		int num_cols = chessboard->horizontal_squares-1;
		int num_rows = chessboard->vertical_squares-1;
		
		for (int i = 0; i < num_rows; i++) {
		    for (int j = 0; j < num_cols; j++) {
		    	mirroredVector[i * num_cols + j] = chessboard->corners[(num_rows - 1 - i) * num_cols + j];
		    }
		}

		return mirroredVector;
	}
};

#endif // CALIBRATION_HPP
