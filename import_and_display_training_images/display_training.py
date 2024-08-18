# ============================================================================
# Author      : Anton Haes
# Contact     : anton.haes@vub.be
# Description : see readme file
# ============================================================================
 

import cv2
import os

rectangle_thickness = 2

#BGR values of rectangles that will be drawn
yellow = (0, 255, 255)
blue = (255, 0, 0)
orange = (0, 165, 255)
dark_orange = (0, 140, 255)
white = (255, 255, 255)


def display_training(path, img_data, number_of_images_per_team):
	team_names = [team_name for team_name in os.listdir(path) if os.path.isdir(path + team_name)]
	print(len(img_data))
	#Display images with their boxes
	img_index = 0;
	for team_name in team_names:
		img_list = os.listdir(path + team_name + "/img")
		for i in range(number_of_images_per_team):
			img = cv2.imread(path + team_name + "/img/" + img_list[i])
			print(path + team_name + "/img/" + img_list[i])
			print(len(img_data[img_index+i]), len(img_data[img_index+i][0]))
			for cone in img_data[img_index+i][0]:
				img = cv2.rectangle(img, (cone[0][0], cone[0][1]), (cone[1][0], cone[1][1]), yellow, rectangle_thickness)
				print(cone)
			for cone in img_data[img_index+i][1]:
				img = cv2.rectangle(img, (cone[0][0], cone[0][1]), (cone[1][0], cone[1][1]), blue, rectangle_thickness)
			for cone in img_data[img_index+i][2]:
				img = cv2.rectangle(img, (cone[0][0], cone[0][1]), (cone[1][0], cone[1][1]), orange, rectangle_thickness)
			for cone in img_data[img_index+i][3]:
				img = cv2.rectangle(img, (cone[0][0], cone[0][1]), (cone[1][0], cone[1][1]), dark_orange, rectangle_thickness)
			for cone in img_data[img_index+i][4]:
				img = cv2.rectangle(img, (cone[0][0], cone[0][1]), (cone[1][0], cone[1][1]), white, rectangle_thickness)
			img = cv2.resize(img, (960, 640))
			cv2.imshow("Training image with boxes", img)
			print("\n\n")
			k = -1
			#store the 'key value' of the pressed key in k, refreshes every 20 ms
			while k == -1:
				k = cv2.waitKey(20)
			#go to the next image is spacebar is pressed, otherwise exit program
			if k != 32:
				break
		if k != 32:
			break
		img_index += len(os.listdir(path + team_name + "/img"))
	cv2.destroyAllWindows()
