#ifndef TENSOR_ENGINE_HPP
#define TENSOR_ENGINE_HPP

#include <cuda_runtime.h>
#include <fstream> // library to write and read from files

#include "NvInfer.h"

enum class Precision {
    FP32,
    FP16,
    INT8,
};

/**
 * @brief A structure representing the dimensions of a tensor.
 * 
 * @author Anton Haes
 */
struct TensorDimensions {
	int max_number_of_batches;	/**< The maximum number of batches. */
	int number_of_channels;		/**< The number of channels in the tensor. */
	int number_of_anchors;		/**< The number of anchors (for output tensors). */
	int width;					/**< The width of the tensor (for input tensors). */
	int height;					/**< The height of the tensor (for input tensors). */
	size_t size;				/**< The total size of the tensor, calculated based on the dimensions. */
	
	 /**
     * @brief Constructor for input tensor dimensions.
     * 
     * @param batches The maximum number of batches.
     * @param channels The number of channels in the tensor.
     * @param w The width of the tensor.
     * @param h The height of the tensor.
     */
	TensorDimensions(int batches, int channels, int w, int h)
		: max_number_of_batches(batches), number_of_channels(channels), width(w), height(h) {
		size = max_number_of_batches * number_of_channels * width * height;
	}
	
	/**
     * @brief Constructor for output tensor dimensions.
     * 
     * @param batches The maximum number of batches.
     * @param channels The number of channels in the tensor.
     * @param anchors The number of anchors in the output tensor.
     */
	TensorDimensions(int batches, int channels, int anchors)
		: max_number_of_batches(batches), number_of_channels(channels), number_of_anchors(anchors) {
		size = max_number_of_batches * number_of_channels * number_of_anchors;
	}
};

/**
 * @brief The default logger class for handling TensorRT logging messages.
 */
class Logger : public nvinfer1::ILogger {
	void log (Severity severity, const char* msg) noexcept {
		if (severity <= Severity::kERROR) {
		    std::cout << msg << std::endl;
		}
    }
};

/**
 * @class TensorEngine
 * @brief A class for managing TensorRT inference operations.
 * 
 * @author Anton Haes
 */
class TensorEngine {
public:
    
    /**
     * @brief Constructs a TensorEngine object and loads the network from the specified engine file.
     * 
     * This constructor initializes the TensorEngine by loading the network 
     * from the specified engine file and setting up necessary resources. 
     * 
     * @author Anton Haes
     * 
     * @param engine_path Path to the TensorRT engine file.
     * @param precision The precision to be used for inference (e.g., FP16, INT8).
     */
    TensorEngine(std::string engine_path, Precision precision) {
		loadNetwork(engine_path);
    }
    
    /**
     * @brief Constructs a TensorEngine object with a specified maximum batch size and loads the network.
     * 
     * This constructor initializes the TensorEngine by loading the network 
     * from the specified engine file and setting up necessary resources, 
     * including setting the maximum batch size.
     * 
     * @author Anton Haes
     * 
     * @param engine_path Path to the TensorRT engine file.
     * @param precision The precision to be used for inference (e.g., FP16, INT8).
     * @param max_number_of_batches The maximum number of batches to be used for inference.
     */
    TensorEngine(std::string engine_path, Precision precision, int max_number_of_batches)
    	: max_batch_size(max_number_of_batches) {
		loadNetwork(engine_path);
    }

    // Destructor for the TensorEngine class
    virtual ~TensorEngine() {}
    
    /**
     * @brief Runs inference on the loaded TensorRT engine.
     * 
     * @author Anton Haes
     * 
     * This method creates a CUDA stream for inference, sets tensor addresses, 
     * performs inference, and synchronizes the CUDA stream.
     */
    void runInference() {
		// Create the cuda stream that will be used for inference
		cudaStream_t inferenceCudaStream;
		checkCudaErrorCode(cudaStreamCreate(&inferenceCudaStream));

		for (int i = 0; i < buffers.size(); i++) {
			bool status = context->setTensorAddress(engine->getIOTensorName(i), buffers[i]);
		}
		
		// Run inference
		bool status = context->enqueueV3(inferenceCudaStream);

		// Synchronize the cuda stream
		checkCudaErrorCode(cudaStreamSynchronize(inferenceCudaStream));
		checkCudaErrorCode(cudaStreamDestroy(inferenceCudaStream));
	}
    
protected:
	int device_index = 0;			/**< The index of the GPU device to be used. */
	std::vector<void*> buffers;		/**< A vector of pointers to input and output buffers. */
	int32_t number_of_batches;		/**< The number of batches for inference. */
	int max_batch_size = -1;		/**< The maximum batch size for the engine. */
	std::vector<TensorDimensions> input_dimensions;		/**< Dimensions of input tensors. */
    std::vector<TensorDimensions> output_dimensions;	/**< Dimensions of output tensors. */
	std::unique_ptr<nvinfer1::IRuntime> runtime = nullptr;				/**< TensorRT runtime object. */
    std::unique_ptr<nvinfer1::ICudaEngine> engine = nullptr;			/**< TensorRT engine object. */
    std::unique_ptr<nvinfer1::IExecutionContext> context = nullptr;		/**< TensorRT execution context object. */
    
    /**
     * @brief Default function to checks for CUDA error codes and throws an exception if an error occurs.
     */
    virtual void checkCudaErrorCode(cudaError_t code) {
		if (code != 0) {
			std::string error_message = "CUDA operation failed with code: " + std::to_string(code) + "(" + cudaGetErrorName(code) + "), with message: " + cudaGetErrorString(code);
			throw std::runtime_error(error_message);
		}
	}
    
private:
	/**
     * @brief Loads the TensorRT network from the specified engine file.
	 *
	 * @author Anton Haes
	 * 
     * @param engine_filename Path to the TensorRT engine file.
     */
	virtual void loadNetwork(std::string engine_filename) {
		// Open the engine file
		std::ifstream file(engine_filename, std::ios::binary | std::ios::ate);
		std::streamsize size = file.tellg();
		file.seekg(0, std::ios::beg);

		std::vector<char> buffer(size);
		if (!file.read(buffer.data(), size)) {
			throw std::runtime_error("Unable to read engine file");
		}
		file.close();

		// Create a runtime to deserialize the engine file.
		Logger logger;
		runtime = std::unique_ptr<nvinfer1::IRuntime> {nvinfer1::createInferRuntime(logger)};
		
		// Set the device index
		int ret = cudaSetDevice(device_index);
		if (ret != 0) {
			int num_GPUs;
			cudaGetDeviceCount(&num_GPUs);
			std::string error_message = "Unable to set GPU device index to: " + std::to_string(device_index) +
			        ". Note, your device has " + std::to_string(num_GPUs) + " CUDA-capable GPU(s).";
			throw std::runtime_error(error_message);
		}
		
		// Create an engine, a representation of the optimized model.
		engine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(buffer.data(), buffer.size()));
		
		// The execution context contains all of the state associated with a particular invocation
		context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());
		
		// Storage for holding the input and output buffers
		// This will be passed to TensorRT for inference
		buffers.resize(engine->getNbIOTensors());
		
		// Create a cuda stream
		cudaStream_t stream;
		checkCudaErrorCode(cudaStreamCreate(&stream));

		// Allocate GPU memory for input and output buffers
		for (int i = 0; i < engine->getNbIOTensors(); i++) {
			const char* tensor_name = engine->getIOTensorName(i);
			const nvinfer1::TensorIOMode tensor_type = engine->getTensorIOMode(tensor_name);
			const nvinfer1::Dims tensor_shape = engine->getTensorShape(tensor_name);
			int max_number_of_batches = tensor_shape.d[0];
			if (max_number_of_batches == -1) { // the network allows unlimited batch size
				max_number_of_batches = max_batch_size;
			}
			int number_of_channels = tensor_shape.d[1];
			// Store information about the input and output dimensions
			if (tensor_type == nvinfer1::TensorIOMode::kINPUT) { // the binding is an input
			    int input_height = tensor_shape.d[2];
			    int input_width = tensor_shape.d[3];
			    TensorDimensions tensor_dimensions = TensorDimensions(max_number_of_batches, number_of_channels, input_width, input_height);
			    input_dimensions.emplace_back(tensor_dimensions);
			    
			    // Allocate memory for the input (allocate enough to fit the max batch size, we could end up using less later)
			    int input_size_bytes = tensor_dimensions.size * sizeof(float);
			    checkCudaErrorCode(cudaMallocAsync(&buffers[i], input_size_bytes, stream));
			} else if (tensor_type == nvinfer1::TensorIOMode::kOUTPUT) { // The binding is an output
				int number_of_anchors = tensor_shape.d[2];
				TensorDimensions tensor_dimensions = TensorDimensions(max_number_of_batches, number_of_channels, number_of_anchors);
			    output_dimensions.emplace_back(tensor_dimensions);
				
			    // Allocate memory for the output
			    int output_size_bytes = tensor_dimensions.size * sizeof(float);
			    checkCudaErrorCode(cudaMallocAsync(&buffers[i], output_size_bytes, stream));
			} else {
			    throw std::runtime_error("Error, IO Tensor is neither an input or output!");
			}
		}
		// Synchronize and destroy the cuda stream
		checkCudaErrorCode(cudaStreamSynchronize(stream));
		checkCudaErrorCode(cudaStreamDestroy(stream));
	}
};

#endif // TENSOR_ENGINE_HPP

