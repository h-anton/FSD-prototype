#ifndef CAR_HPP
#define CAR_HPP

#include <iostream>
#include <cstdlib>
#include <iomanip>
#include <cstdint>

/**
 * @class Car
 * @brief Provides an interface to control a car via CAN bus communication.
 * 
 * This class is responsible for controlling the car's steering and speed by sending
 * appropriate commands through the CAN bus. It initializes the CAN interface, sends
 * commands to the car, and formats CAN messages for transmission.
 * 
 * @author Anton Haes
 */
class Car {
public:

	uint8_t max_angle_left = 52; // The maximum steering angle to the left in degrees.
	uint8_t max_angle_right = 132; // The maximum steering angle to the right in degrees.
	
	/**
     * @brief Constructor for the Car class.
     * 
     * Initializes the CAN interface and the car's robot system. Calls the 
     * `initCAN` and `initRobot` functions to set up the car for operation.
     * 
     * @author Anton Haes
     */
    Car() {
		initCAN(can_bitrate);
		initRobot();
    }
	
	// Destructor for the Car class
    ~Car() {}
    
    /**
     * @brief Sends a drive command to the car.
     * 
     * Sends a command to control the car's steering angle and wheel speeds. The 
     * angle is clamped within the allowed range, and a CAN message is constructed
     * and sent to the car.
     *
     * @author Anton Haes
     * 
     * @param angle The desired steering angle in degrees.
     * @param speedL The speed of the left wheel.
     * @param speedR The speed of the right wheel.
     * 
     * @return An integer status code from the `system` call, indicating success or failure
     *         of the command execution.
     */
    int drive(uint8_t angle, uint8_t speedL, uint8_t speedR) {
    	// Make sure the angle does not exceed the steering capabilities of the car
    	if (angle < max_angle_left) {
		angle = max_angle_left;
		}
		if (angle > max_angle_right) {
			angle = max_angle_right;
		}
		
		int can_id = 124;

		uint8_t message[8];
		message[0] = angle;
		message[1] = speedL;
		message[2] = speedR;
		for (uint8_t i = 3; i < 8; i++) {
			message[i] = 0;
		}
		
		std::string command = "cansend can0 " + formatCANMessage(can_id, message);
		return system(command.c_str());
	}
    

private:
	int can_bitrate = 250000; // bitrate of the CAN network
	
	/**
     * @brief Initializes the CAN interface.
     * 
     * Checks if the CAN interface is already enabled and, if not, configures it with
     * the specified bitrate. This function also handles potential errors during the 
     * setup process.
     * 
     * @author Anton Haes
     *
     * @param bitrate The bitrate to be set for the CAN interface.
     */
	void initCAN(int bitrate) {
		// First we need to see if the CAN controller is enabled
		FILE* stream = popen("ip link show can0", "r");
		if (stream) {
		    char buffer[128];
		    bool found_up = false;
		    bool found_down = false;
		    while (fgets(buffer, sizeof(buffer), stream) != NULL) {
		        if (std::string(buffer).find("UP") != std::string::npos) {
		            found_up = true;
		            break;
		        } else if (std::string(buffer).find("DOWN") != std::string::npos) {
		            found_down = true;
		            break;
		        }
		    }
		    pclose(stream);

		    if (found_up) { // CAN is already running, there is nothing to do
		        std::cout << "CAN0 is already turned on." << std::endl;
		    } else if (found_down) { // CAN should be turned on
		        std::string command = "ip link set can0 up type can bitrate " + std::to_string(bitrate) + " dbitrate " + std::to_string(bitrate) + " berr-reporting on fd on";
		        system(command.c_str());
		    } else {
		        std::cout << "Problem turning CAN0 on." << std::endl;
		    }
		} else {
		    std::cerr << "Error: Unable to execute ip link command." << std::endl;
		}
	}
	
	/**
     * @brief Formats a CAN message for transmission.
     * 
     * Converts a CAN message into a string format suitable for sending via the 
     * `cansend` command. The message is formatted as a hexadecimal string with the
     * CAN ID followed by the message bytes.
     * 
     * @author Anton Haes
     * 
     * @param id The CAN ID for the message.
     * @param message Pointer to the array of bytes that make up the CAN message.
     * 
     * @return A formatted string representing the CAN message.
     */
	std::string formatCANMessage(int id, const uint8_t* message) {
		// Create a string stream to build the output
		std::stringstream ss;
		
		size_t length = sizeof(message);
		
		// Write the ID
		ss << id << "#";
		
		// Write the hexadecimal values of each byte in the message
		for (size_t i = 0; i < length; ++i) {
		    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(message[i]);
		}
		
		// Return the formatted string
		return ss.str();
	}
	
	/**
     * @brief Initializes the small scale robot.
     * 
     * Sends an initialization message to the robot to prepare it for operation.
     * This message is sent using a specific CAN ID and an array of zeroed bytes.
     * 
     * @author Anton Haes
     * 
     * @return An integer status code from the `system` call, indicating success or failure
     *         of the command execution.
     */
	int initRobot() {
		int can_id = 123;
		uint8_t message[8];
		for (uint8_t i = 0; i < 8; i++) {
			message[i] = 0;
		}
		std::string command = "cansend can0 " + formatCANMessage(can_id, message);
		return system(command.c_str());
	}
};

#endif // CAR_HPP

