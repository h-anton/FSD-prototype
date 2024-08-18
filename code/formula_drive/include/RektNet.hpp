#ifndef REKTNET_HPP
#define REKTNET_HPP

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#include "TensorEngine.hpp"
#include "Cone.hpp"

/**
 * @class RektNet
 * @brief A class for running Rektnet inference with TensorRT.
 * 
 * This class extends the TensorEngine class to provide functionality specific
 * to RektNet, which is used for detecting keypoints on objects. It handles
 * preprocessing of input data, running inference, and post-processing to
 * extract keypoints from the results.
 * 
 * @author Anton Haes
 */
class RektNet : public TensorEngine {
public:

	/**
     * @brief Constructs a RektNet object and initializes the TensorRT engine.
     * 
     * @author Anton Haes
     * 
     * @param engine_path Path to the TensorRT engine file.
     */
	RektNet(std::string engine_path) : TensorEngine(engine_path, Precision::FP16, 32) {}
	
	// Destructor for the RektNet class
	~RektNet () {}
	
	/**
     * @brief Processes a vector of Cone objects to extract keypoints.
     * 
     * This method performs preprocessing on the input cones, runs inference using
     * the RektNet model, and then post-processes the results to extract keypoints 
     * for each cone.
     * 
     * @param cones A pointer to a vector of Cone objects. The keypoints for each 
     *              cone will be populated after inference.
     */
	void getKeypoints(std::vector<Cone>* cones) {
		if (cones->size() != 0) {
			preProcess(cones);
			runInference();
			postProcess(cones);
		}
	}
	

private:
	/**
     * @brief Preprocesses the input cones for RektNet inference.
     * 
     * @author Anton Haes
     * 
     * @param cones A pointer to a vector of Cone objects. The frames of the cones
     *              will be processed and uploaded to GPU memory.
     */
	void preProcess(std::vector<Cone>* cones) {
		// Create the cuda stream that will be used for pre processing
		cudaStream_t inferenceCudaStream;
		checkCudaErrorCode(cudaStreamCreate(&inferenceCudaStream));
		
		number_of_batches = cones->size();
		std::vector<int> input_size = {number_of_batches, input_dimensions[0].number_of_channels, input_dimensions[0].width, input_dimensions[0].height};
		cv::Mat processed_input(input_size, CV_32FC1);
		
		nvinfer1::Dims4 dimensions = {number_of_batches, input_dimensions[0].number_of_channels, input_dimensions[0].height, input_dimensions[0].width};
		context->setInputShape(engine->getIOTensorName(0), dimensions);

		for (int i = 0; i < number_of_batches; i++) {
			cv::resize((*cones)[i].frame, (*cones)[i].frame, cv::Size(input_dimensions[0].width, input_dimensions[0].height));
			cv::Mat float_img;
			(*cones)[i].frame.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
			for (int c = 0; c < float_img.channels(); ++c) {
				for (int j = 0; j < float_img.cols; ++j) {
					for (int k = 0; k < float_img.rows; ++k) {
						int index[4] = {i, c, j, k};
					    processed_input.at<float>(index) = float_img.at<cv::Vec3f>(j, k)[c];
					}
				}
			}
			
		}
		
		size_t input_size_bytes = processed_input.channels() * processed_input.rows * processed_input.cols; 
		void* input_data_pointer = processed_input.ptr<void>();

		// Copy processedInput to input buffer
		checkCudaErrorCode(cudaMemcpy(buffers[0], input_data_pointer, input_size_bytes, cudaMemcpyHostToDevice));
		
		// Synchronize the cuda stream
		checkCudaErrorCode(cudaStreamSynchronize(inferenceCudaStream));
		checkCudaErrorCode(cudaStreamDestroy(inferenceCudaStream));

	}
	
	/**
     * @brief Post-processes the results from RektNet inference to extract keypoints.
     * 
     * @Anton Haes
     * 
     * @param cones A pointer to a vector of Cone objects. The keypoints will be
     *              populated based on the inference results.
     */
	void postProcess(std::vector<Cone>* cones) {
		// Create the cuda stream that will be used for post processing
		cudaStream_t inferenceCudaStream;
		checkCudaErrorCode(cudaStreamCreate(&inferenceCudaStream));
		
		size_t output_size_bytes = number_of_batches * output_dimensions[1].number_of_channels * output_dimensions[1].number_of_anchors * sizeof(float);
		float* output_data_host = new float[number_of_batches * output_dimensions[1].number_of_channels * output_dimensions[1].number_of_anchors]; // batch_size * 7 * 2
		checkCudaErrorCode(cudaMemcpyAsync(output_data_host, buffers[2], output_size_bytes, cudaMemcpyDeviceToHost, inferenceCudaStream));
		
		std::vector<std::vector<std::pair<float, float>>> result(number_of_batches);
		for (int i = 0; i < number_of_batches; i++) {
		    result[i].reserve(7); // Reserve memory for 7 tuples

		    // Populate each vector with tuples
		    for (int j = 0; j < 7; ++j) {
		        float* tuple_ptr = output_data_host + (i * 7 * 2) + (j * 2);
		        result[i].emplace_back(tuple_ptr[0], tuple_ptr[1]);
		    }
		    (*cones)[i].keypoints = result[i];
		}
		
		// Synchronize the cuda stream
		checkCudaErrorCode(cudaStreamSynchronize(inferenceCudaStream));
		checkCudaErrorCode(cudaStreamDestroy(inferenceCudaStream));
	}
};

#endif // REKTNET_HPP

