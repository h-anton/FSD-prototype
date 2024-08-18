#ifndef YOLO_HPP
#define YOLO_HPP

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#include "TensorEngine.hpp"
#include "Cone.hpp"

/**
 * @class Yolo
 * @brief A class for running YOLO object detection with TensorRT.
 * 
 * This class extends the TensorEngine class to provide functionality specific
 * to YOLO object detection. It handles preprocessing of input frames, running
 * inference, and post-processing of results to extract detected objects, such
 * as traffic cones.
 * 
 * @author Anton Haes
 */
class Yolo : public TensorEngine {
public:

	/**
     * @brief Constructs a Yolo object and initializes the TensorRT engine.
     * 
     * @author Anton Haes
     * 
     * @param engine_path Path to the TensorRT engine file.
     */
	Yolo(std::string engine_path) : TensorEngine(engine_path, Precision::FP16) {}
	
	// Destructor for the Yolo class
	~Yolo () {}
	
	/**
     * @brief Processes a frame to detect cones.
     * 
     * This method performs preprocessing on the input frame, runs inference using
     * the YOLO model, and then post-processes the results to detect and return 
     * cones found in the frame.
     * 
     * @author Anton Haes
     * 
     * @param frame The input frame (image) on which to run detection.
     * @return A vector of Cone objects representing detected cones.
     */
	std::vector<Cone> getCones(cv::Mat frame) {
		preProcess(frame);
		runInference();
		return postProcess();
	}
	
private:
	cv::Mat frame;	/**< The current frame being processed. */
	
	/**
     * @brief Preprocesses the input frame for YOLO inference.
     * 
     * @author Anton Haes
     * 
     * @param cpu_frame The input frame (image) on the CPU.
     */
	void preProcess(cv::Mat cpu_frame) {
		// Save frame for later use
		frame = cpu_frame;
		// Upload the image GPU memory
		cv::cuda::GpuMat gpu_frame;
		gpu_frame.upload(cpu_frame);
		// The model expects RGB input
		cv::cuda::cvtColor(gpu_frame, gpu_frame, cv::COLOR_BGR2RGB);
		// Resize image to input size of engine
		cv::cuda::resize(gpu_frame, gpu_frame, cv::Size(input_dimensions[0].width, input_dimensions[0].height));
		//	Convert to format expected by inference engine
		//		every image should be inside a vector (batch)
		//		all the images (batch) should be combined in a vector (input)
		std::vector<cv::cuda::GpuMat> batch{std::move(gpu_frame)}; // put all the image in a vector
		std::vector<std::vector<cv::cuda::GpuMat>> input {std::move(batch)}; // make vector of all the batches
		
		number_of_batches = static_cast<int32_t>(input.size());
		
		// Create the cuda stream that will be used for pre processing
		cudaStream_t inferenceCudaStream;
		checkCudaErrorCode(cudaStreamCreate(&inferenceCudaStream));
		
		// Load all the inputs
		for (size_t i = 0; i < number_of_batches; i++) {
			nvinfer1::Dims4 dimensions = {input_dimensions[0].max_number_of_batches, input_dimensions[0].number_of_channels, input_dimensions[0].height, input_dimensions[0].width};
			context->setInputShape(engine->getIOTensorName(i), dimensions);
		}
		
		cv::cuda::GpuMat processed_input = blobFromGpuMats(input[0], true);
		
		size_t input_size_bytes = processed_input.channels() * processed_input.rows * processed_input.cols * sizeof(float); 
		void* input_data_pointer = processed_input.ptr<void>();
		
		// Copy processedInput to input buffer
		checkCudaErrorCode(cudaMemcpyAsync(buffers[0], input_data_pointer, input_size_bytes, cudaMemcpyHostToDevice, inferenceCudaStream));
		
		// Synchronize the cuda stream
		checkCudaErrorCode(cudaStreamSynchronize(inferenceCudaStream));
		checkCudaErrorCode(cudaStreamDestroy(inferenceCudaStream));
	}
	
	/**
     * @brief Post-processes the results from YOLO inference to extract detected cones.
     * 
     * @author Anton Haes
     * 
     * @return A vector of Cone objects representing detected cones.
     */
	std::vector<Cone> postProcess() {
		// Create the cuda stream that will be used for post processing
		cudaStream_t inferenceCudaStream;
		checkCudaErrorCode(cudaStreamCreate(&inferenceCudaStream));
		
		std::vector<std::vector<std::vector<float>>> result;
	
		for (int batch = 0; batch < number_of_batches; batch++) {
			// Batch
			std::vector<std::vector<float>> batch_outputs{};
			for (int32_t output_binding = input_dimensions.size(); output_binding < engine->getNbIOTensors(); output_binding++) {
			    // We start at index inputDims.size() to account for the inputs in our buffers
			    std::vector<float> output;
			    uint32_t size_tensor = output_dimensions[output_binding - input_dimensions.size()].size;
			    output.resize(size_tensor);
			    // Copy the output
			    checkCudaErrorCode(cudaMemcpyAsync(output.data(), static_cast<char*>(buffers[output_binding]) + (batch * sizeof(float) * size_tensor), size_tensor * sizeof(float), cudaMemcpyDeviceToHost, inferenceCudaStream));
			    batch_outputs.emplace_back(std::move(output));
			}
			result.emplace_back(std::move(batch_outputs));
		}
		
		// Synchronize the cuda stream
		checkCudaErrorCode(cudaStreamSynchronize(inferenceCudaStream));
		checkCudaErrorCode(cudaStreamDestroy(inferenceCudaStream));
		
		// Extract bounding box informations
		std::vector<float> output_vector = result[0][0];
		float probability_threshold = 0.25f;
		float NMS_threshold = 0.65f;
		int num_anchors = output_dimensions[0].number_of_anchors;
		int num_channels = output_dimensions[0].number_of_channels;
		
		cv::Mat output = cv::Mat(num_channels, num_anchors, CV_32F, output_vector.data());
    	output = output.t();
    	
		std::vector<cv::Rect> bounding_boxes; // bounding boxes of all detected objects before NMS
		std::vector<int> labels; // labels of all detected objects, used for NMS
		std::vector<float> scores; // scores of all detected objects, used for NMS
		std::vector<int> indices; // indices of the vector objects that will be kept after NMS
		
		for (int i = 0; i < num_anchors; i++) {
			float* output_ptr = output.row(i).ptr<float>();
		    // the model gives a score to each possible 'class'
		    //float* max_score_ptr = std::max_element(output_ptr+4, output_ptr+9);
		    float* ptr_score_first_class = &output.at<float>(i, 4);
		    float* ptr_score_last_class = &output.at<float>(i, 9);
		    float* max_score_ptr = std::max_element(ptr_score_first_class, ptr_score_last_class);
		    float score = *max_score_ptr;
		    int label = max_score_ptr - ptr_score_first_class; // index of the class with highest score
		    if (score > probability_threshold) {
		        float x = output.at<float>(i, 0);
		        float y = output.at<float>(i, 1);
		        float w = output.at<float>(i, 2);
		        float h = output.at<float>(i, 3);
		        
		        // Increase size of bounding boxes to aid keypoint detection
		        w *= 1.1;
		        h *= 1.1;
		        
		        float start_x = x - w/2;
		        float start_y = y - h/2;
		        
		        // The above coordinates are in the 'resized' frame
		        int start_x_full_frame = (int)(start_x / input_dimensions[0].width * frame.cols);
		        int start_y_full_frame = (int)(start_y / input_dimensions[0].height * frame.rows);
		        int width_full_frame = (int)(w / input_dimensions[0].width * frame.cols);
		        int height_full_frame = (int)(h / input_dimensions[0].height * frame.rows);
		        
		        // Make sure the bounding box remains within the image bounds
		        start_x_full_frame = std::max(0, start_x_full_frame);
		        start_y_full_frame = std::max(0, start_y_full_frame);
		        width_full_frame = std::min(frame.cols-start_x_full_frame, width_full_frame);
		        height_full_frame = std::min(frame.rows-start_y_full_frame, height_full_frame);
		        
		        // Discard bounding boxes which are too big to be a cone
		        // Discard bounding boxes with abnormal shapes
		        if (width_full_frame * height_full_frame < 0.015 * frame.cols * frame.rows
		        	|| width_full_frame > 3*height_full_frame || height_full_frame > 3*width_full_frame) {
		        	cv::Rect bounding_box(start_x_full_frame, start_y_full_frame, width_full_frame, height_full_frame);
		        	bounding_boxes.push_back(bounding_box);
		        	labels.push_back(label);
		        	scores.push_back(score);
		        }
		    }
		}
		
		// Perform NMS to remove duplicate bounding boxes
		cv::dnn::NMSBoxesBatched(bounding_boxes, scores, labels, probability_threshold, NMS_threshold, indices);
		
		// Construct cone objects
		std::vector<Cone> objects;
		
		for(int& chosen_index: indices) {
			cv::Mat extracted_frame = frame(bounding_boxes[chosen_index]).clone(); // Use clone() to create a copy
			int start_x_full_frame = bounding_boxes[chosen_index].x;
			int start_y_full_frame = bounding_boxes[chosen_index].y;
			objects.push_back(Cone(labels[chosen_index], start_x_full_frame, start_y_full_frame, extracted_frame));
		}
		
		return objects;
	}
	
	/**
     * @brief Converts a batch of GPU images into a format suitable for YOLO inference.
     * 
     * @param batchInput A vector of GPU images to be converted.
     * @param normalize Whether to normalize the images.
     * @return A GpuMat representing the batch of processed images.
     */
	cv::cuda::GpuMat blobFromGpuMats(const std::vector<cv::cuda::GpuMat>& batchInput, bool normalize) {
		cv::cuda::GpuMat gpu_dst(1, batchInput[0].rows * batchInput[0].cols * batchInput.size(), CV_8UC3);

		size_t width = batchInput[0].cols * batchInput[0].rows;
		for (size_t img = 0; img < batchInput.size(); img++) {
			std::vector<cv::cuda::GpuMat> input_channels{
			        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U, &(gpu_dst.ptr()[0 + width * 3 * img])),
			        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U, &(gpu_dst.ptr()[width + width * 3 * img])),
			        cv::cuda::GpuMat(batchInput[0].rows, batchInput[0].cols, CV_8U,
			                         &(gpu_dst.ptr()[width * 2 + width * 3 * img]))
			};
			cv::cuda::split(batchInput[img], input_channels);  // HWC -> CHW
		}

		cv::cuda::GpuMat m_float;
		if (normalize) {
			// [0.f, 1.f]
			gpu_dst.convertTo(m_float, CV_32FC3, 1.f / 255.f);
		} else {
			// [0.f, 255.f]
			gpu_dst.convertTo(m_float, CV_32FC3);
		}

		// Apply scaling and mean subtraction
		std::array<float, 3> sub_values{0.f, 0.f, 0.f};
    	std::array<float, 3> div_values{1.f, 1.f, 1.f};
		cv::cuda::subtract(m_float, cv::Scalar(sub_values[0], sub_values[1], sub_values[2]), m_float, cv::noArray(), -1);
		cv::cuda::divide(m_float, cv::Scalar(div_values[0], div_values[1], div_values[2]), m_float, 1, -1);

		return m_float;
	}
	
};

#endif // YOLO_HPP

