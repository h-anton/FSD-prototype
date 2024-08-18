# Python code to import and display the "FSOCO bounding boxes" training data

## Import the training data
The `import_training.py` file allows to import the training data from the "FSOCO bounding boxes" training dataset. This file contains a function, `import_training(path)`, which will return all the cones and the coördinates of its bounding box for every image. You can use the function as follows (and as shown in the `example.py` file):
* the function takes one argument, the path of the fsoco_bounding_boxes_train folder (which you can download from fsoco-dataset.com)
* the function will return a list of n elements, where n is the number of images found in the fsoco_bounding_boxes_train directory. The list countains the data in the following way: `list = [data_img0, data_img1, ...]`, where `data_img = [yellow_cones, blue_cones, orange_cones, large_orange_cones, unknown_cones]` and where `cones = [[x0, y0], [x1, y1]]` (with x0, y0, x1, y1 the coördinates of the bounding box).

## Display the training data
The `display_training.py` file allows to display all the images and its bounding boxes from the "FSOCO bounding boxes" training dataset. This file contains a function, `display_training(path, img_data, number_of_image_per_team)`, which will display the images with a bounding box arround each cone. You can use the function as follows (and as shown in the `example.py` file):
* the function takes three arguments: the path of the fsoco_bounding_boxes_train folder (which you can download from fsoco-dataset.com), the locations and types of every cone (you can extract them from the trainig dataset using the import_training.py file), and the number of images of each team that will be displayed (this is to avoid displaying all of the 11572 images, which is impractical). Note that there are 41 teams who have submitted training data.
* the function does not return anything
* it is required to have the `opencv` library installed. You can install it using the `pip install opencv-python` command in your terminal
* the images are displayd one by one. To display the next image, press spacebar. To exit the program, press any other key.
