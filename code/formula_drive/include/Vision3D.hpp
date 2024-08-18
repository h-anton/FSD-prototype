#ifndef VISION3D_HPP
#define VISION3D_HPP

#include <fstream> // library to write and read from files
#include <Dense> // Eigen library for matrices

#include "Cone.hpp"
#include "UI.hpp"

/**
 * @struct CalibrationData
 * @brief Stores calibration parameters and matrices for a camera system.
 * 
 * This struct contains various calibration matrices and parameters used in camera 
 * calibration, and in the linear camera model. It also stores the pixel width and
 * height of the camera frame.
 *
 * @author Anton Haes
 */
struct CalibrationData {
	Eigen::MatrixXd P;
	Eigen::MatrixXd K_inv;
	Eigen::MatrixXd R_inv;
	Eigen::MatrixXd t;
	Eigen::MatrixXd E;
	Eigen::MatrixXd F;
	int pixel_width;
	int pixel_height;
};

/**
 * @class Vision3D
 * @brief A class to calculate the real world position of cones.
 * 
 * This class allows users to read the calibration data from the calibration program,
 * and to calculate the real world position (world coordinates) of the cones given
 * their pixel coordinates.
 *
 * @author Anton Haes
 */
class Vision3D {
public:
	/**
     * @brief Constructs a Vision3D object for calculating the position of cones.
     * 
     * Reads the calibration file (containing the calibration data of the camera),
     * and store the values in a CalibrationData struct.
     *
     * @author Anton Haes
     * 
     * @param calibration_file The path to the calibration file (in a specific format) 
 	 *        				   that contains camera calibration data.
     * @throws std::runtime_error if the file containing the calibration data could
     *							  not be opened.
     */
    Vision3D(const std::string& calibration_file) {
		// Open calibration file
		std::ifstream file(calibration_file, std::ios::in | std::ios::binary);
		if (!file.is_open()) {
		    throw std::runtime_error("Failed to open calibration file.");
		}

		// Read pixel width and height
		file.read(reinterpret_cast<char*>(&calibration_data.pixel_width), sizeof(int));
		file.read(reinterpret_cast<char*>(&calibration_data.pixel_height), sizeof(int));

		// Read matrix P
		int rows, cols;
		file.read(reinterpret_cast<char*>(&rows), sizeof(int));
		file.read(reinterpret_cast<char*>(&cols), sizeof(int));
		calibration_data.P.resize(rows, cols);
		file.read(reinterpret_cast<char*>(calibration_data.P.data()), rows * cols * sizeof(double));

		// Read matrix K_inv
		rows, cols;
		file.read(reinterpret_cast<char*>(&rows), sizeof(int));
		file.read(reinterpret_cast<char*>(&cols), sizeof(int));
		calibration_data.K_inv.resize(rows, cols);
		file.read(reinterpret_cast<char*>(calibration_data.K_inv.data()), rows * cols * sizeof(double));
		
		// Read matrix R_inv
		rows, cols;
		file.read(reinterpret_cast<char*>(&rows), sizeof(int));
		file.read(reinterpret_cast<char*>(&cols), sizeof(int));
		calibration_data.R_inv.resize(rows, cols);
		file.read(reinterpret_cast<char*>(calibration_data.R_inv.data()), rows * cols * sizeof(double));
		
		// Read matrix t
		rows, cols;
		file.read(reinterpret_cast<char*>(&rows), sizeof(int));
		file.read(reinterpret_cast<char*>(&cols), sizeof(int));
		calibration_data.t.resize(rows, cols);
		file.read(reinterpret_cast<char*>(calibration_data.t.data()), rows * cols * sizeof(double));

		file.close();
		
		// Perform precomputations, according to section 3.4 of bachelor thesis
		calibration_data.E = calibration_data.R_inv * calibration_data.K_inv;
		calibration_data.F = calibration_data.R_inv * calibration_data.t;
    }

	// Destructor for the Vision3D class
    ~Vision3D() {}
    
    /**
	 * @brief Calculates the real-world position of cones.
	 * 
	 * This function computes the real-world coordinates (world coordinates) for each 
	 * cone in a vector of cones. The calculated positions are stored directly in the 
	 * provided vector.
	 *
	 * @author Anton Haes
	 * 
	 * @param cones A pointer to the vector containing all the cones the car has detected.
	 *        The real-world positions of these cones will be calculated and updated
	 *        within this vector.
	 */
    void calculatePosition(std::vector<Cone>* cones) {
    	for(Cone& cone: *cones) {
    		// Calculate the center of the cone
    		float center_x = 0;
    		float center_y = 0;
    		for(std::pair<float, float> keypoint: cone.keypoints) {
				center_x += (float)cone.start_x + keypoint.first*cone.width;
				center_y += (float)cone.start_y + keypoint.second*cone.height;
			}
			center_x /= 7;
			center_y /= 7;
			
			// Calculate the world coordinates of the cone
			float z_height = 16.5; // the height of the cone is known
			Eigen::VectorXd pixel_coord(3); // vector to store the pixel_coordinates of the center of the cone
    		pixel_coord << center_x, center_y, 1; // populate pixel_coord
			float LzC = (z_height + calibration_data.F(2)) / (calibration_data.E * pixel_coord)(2); // calculate LzC according to Equation 15 of bachelor thesis
			Eigen::VectorXd Lw = LzC * calibration_data.E * pixel_coord - calibration_data.F; // calculate Lw according to Equation 14 of bachelor thesis
			
			// Store the world coordinates in the cone object
			cone.world_coordinates_mm[0] = (int)Lw(0);
			cone.world_coordinates_mm[1] = (int)Lw(1);
    	}
    }

private:
	CalibrationData calibration_data; // Struct where the calibration data from the camera is stored.
};

#endif // VISION3D_HPP

