# ============================================================================
# Author      : Anton Haes
# Contact     : anton.haes@vub.be
# Description : Conversion of FSOCO dataset to the Ultralytics YOLO format
# ============================================================================


from import_training import import_training
import random
import cv2
import os

def remove_black_bars(img):
	(img_height, img_width, img_channels) = img.shape
	
	# offset for left black bar
	x = 0
	y = int(img_height / 2)
	while ((img[y][x] == [0, 0, 0]).all()):
		x += 1
	offset_x_left = x - 1
	
	# offset for right black bar
	x = img_width - 1
	y = int(img_height / 2)
	while ((img[y][x] == [0, 0, 0]).all()):
		x -= 1
	offset_x_right = x + 1
	
	# offset for top black bar
	x = int(img_width / 2)
	y = 0
	while ((img[y][x] == [0, 0, 0]).all()):
		y += 1
	offset_y_top = y - 1
	
	# offset for top black bar
	x = int(img_width * 0.8)
	y = img_height - 1
	while ((img[y][x] == [0, 0, 0]).all()):
		y -= 1
	offset_y_bottom = y + 1
	
	return (offset_x_left, offset_x_right, offset_y_top, offset_y_bottom)



path = "../fsoco_bounding_boxes_train/"

export_resolution = (960, 600)
export_path = "../fsoco_yolo_960x600/"

print_freq = 100

# create the folders
os.makedirs(export_path + "train/labels", exist_ok=True)
os.makedirs(export_path + "train/images", exist_ok=True)
os.makedirs(export_path + "val/labels", exist_ok=True)
os.makedirs(export_path + "val/images", exist_ok=True)
os.makedirs(export_path + "test/labels", exist_ok=True)
os.makedirs(export_path + "test/images", exist_ok=True)

# import the image annotations in FSOCO format
img_data = import_training(path)

# import the images
images = []
team_names = [team_name for team_name in os.listdir(path) if os.path.isdir(path + team_name)]
for team_name in team_names:
	img_list = os.listdir(path + team_name + "/img")
	for image_path in img_list:
		images.append(path + team_name + "/img/" + image_path)

print("image labels and paths loaded")

# convert img_data form [[x0, y0], [x1, y1]] to normalized xywh
data_yolo = []
counter = -1
for img_cones, img_path in zip(img_data, images):
	counter += 1
	if (counter % print_freq == 0):
		print("loaded image " + str(counter) + "\t/" + str(len(images)))
	img_annotation = []
	for i in range(len(img_cones)):
		for cone in img_cones[i]:
			image = cv2.imread(img_path)
			
			[[x0, y0], [x1, y1]] = cone
			
			(offset_x_left, offset_x_right, offset_y_top, offset_y_bottom) = remove_black_bars(image)
			image = image[offset_y_top+1:offset_y_bottom-1, offset_x_left+1:offset_x_right-1]
			(old_height, old_width, old_channels) = image.shape
			
			x0 -= offset_x_left
			x1 -= offset_x_left
			y0 -= offset_y_top
			y1 -= offset_y_top
			
			image = cv2.resize(image, export_resolution)
			(img_height, img_width, img_channels) = image.shape
			
			x0 *= img_width / old_width
			x1 *= img_width / old_width
			y0 *= img_height / old_height
			y1 *= img_height / old_height
			
			'''
			X0 = round(x0)
			Y0 = round(y0)
			X1 = round(x1)
			Y1 = round(y1)
			cv2.rectangle(image, (X0, Y0), (X1, Y1), (0, 0, 255), 2)
			image = cv2.resize(image, (960, 640))
			cv2.imshow("Training image with boxes", image)
			k = -1
			#store the 'key value' of the pressed key in k, refreshes every 20 ms
			while k == -1:
				k = cv2.waitKey(20)
			#go to the next image is spacebar is pressed, otherwise exit program
			if k != 32:
				cv2.destroyAllWindows'''
			
			# convert to normalized xywh:
			x_normalized = (x0 + abs(x1 - x0) / 2) / img_width
			y_normalized = (y0 + abs(y1 - y0) / 2) / img_height
			w_normalized = abs(x1 - x0) / img_width
			h_normalized = abs(y1 - y0) / img_height

			
			img_annotation.append([i, x_normalized, y_normalized, w_normalized, h_normalized])
	data_yolo.append(img_annotation)

print("done converting labels")

#shuffle data
combined_list = list(zip(data_yolo, images))
random.shuffle(combined_list)
data_yolo, images = zip(*combined_list)
print("data shuffled")

# data is split as follows
#	80% training data
#	15% validation data
#	5% test data
training_index = len(data_yolo) * 0.8
validation_index = len(data_yolo) * 0.15
test_index = len(data_yolo) * 0.05

# export train data
for i in range(int(training_index)):
	if (i % print_freq == 0):
		print("saved image " + str(i) + "\t/" + str(len(images)))
	img = cv2.imread(images[i])
	(offset_x_left, offset_x_right, offset_y_top, offset_y_bottom) = remove_black_bars(img)
	img = img[offset_y_top+1:offset_y_bottom-1, offset_x_left+1:offset_x_right-1]
	img = cv2.resize(img, export_resolution)
	with open(export_path + "train/labels/" + str(i) + ".txt", 'w') as file:
		for label in data_yolo[i]:
			file.write(' '.join([str(e) for e in label]) + '\n')
	export_img_path = export_path + "train/images/" + str(i) + ".png"
	cv2.imwrite(export_img_path, img)

# export val data
for i in range(int(training_index), int(training_index+validation_index)):
	if (i % print_freq == 0):
		print("saved image " + str(i) + "\t/" + str(len(images)))
	img = cv2.imread(images[i])
	(offset_x_left, offset_x_right, offset_y_top, offset_y_bottom) = remove_black_bars(img)
	img = img[offset_y_top+1:offset_y_bottom-1, offset_x_left+1:offset_x_right-1]
	img = cv2.resize(img, export_resolution)
	with open(export_path + "val/labels/" + str(i) + ".txt", 'w') as file:
		for label in data_yolo[i]:
			file.write(' '.join([str(e) for e in label]) + '\n')
	export_img_path = export_path + "val/images/" + str(i) + ".png"
	cv2.imwrite(export_img_path, img)

# export test data
for i in range(int(training_index+validation_index), len(data_yolo)):
	if (i % print_freq == 0):
		print("saved image " + str(i) + "\t/" + str(len(images)))
	img = cv2.imread(images[i])
	(offset_x_left, offset_x_right, offset_y_top, offset_y_bottom) = remove_black_bars(img)
	img = img[offset_y_top+1:offset_y_bottom-1, offset_x_left+1:offset_x_right-1]
	img = cv2.resize(img, export_resolution)
	with open(export_path + "test/labels/" + str(i) + ".txt", 'w') as file:
		for label in data_yolo[i]:
			file.write(' '.join([str(e) for e in label]) + '\n')
	export_img_path = export_path + "test/images/" + str(i) + ".png"
	cv2.imwrite(export_img_path, img)




