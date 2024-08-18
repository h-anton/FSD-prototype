# ============================================================================
# Author      : Anton Haes
# Contact     : anton.haes@vub.be
# Description : see readme file
# ============================================================================
 
import cv2
import os

def import_images(path):
	team_names = [team_name for team_name in os.listdir(path) if os.path.isdir(path + team_name)]

	#Import images
    images = []
	for team_name in team_names:
		img_list = os.listdir(path + team_name + "/img")
		for i in range(len(img_list)):
			images.append(cv2.imread(path + team_name + "/img/" + img_list[i]))

    return images
