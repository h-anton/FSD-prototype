# ============================================================================
# Author      : Anton Haes
# Contact     : anton.haes@vub.be
# Description : see readme file
# ============================================================================


import json
import os

def import_training(path):
	team_names = [team_name for team_name in os.listdir(path) if os.path.isdir(path + team_name)]

	#img_data = [data_img0, data_img1, ...] where data_img = [yellow_cones, blue_cones, orange_cones, large_orange_cones, unknown_cones]
	img_data = []

	for team_name in team_names:
		img_list = os.listdir(path + team_name + "/img")
	
		#Collect all data from json files
		for img_path in img_list:
			yellow_cones = []
			blue_cones = []
			orange_cones = []
			large_orange_cones = []
			unknown_cones = []
		
			file = open(path + team_name + "/ann/" + img_path + ".json")
			data = json.load(file)
		
			for i in range(len(data["objects"])):
				if data["objects"][i]["classTitle"] == "yellow_cone":
					yellow_cones.append(data["objects"][i]["points"]["exterior"])
				elif data["objects"][i]["classTitle"] == "blue_cone":
					blue_cones.append(data["objects"][i]["points"]["exterior"])
				elif data["objects"][i]["classTitle"] == "orange_cone":
					orange_cones.append(data["objects"][i]["points"]["exterior"])
				elif data["objects"][i]["classTitle"] == "large_orange_cone":
					large_orange_cones.append(data["objects"][i]["points"]["exterior"])
				elif data["objects"][i]["classTitle"] == "unknown_cone":
					unknown_cones.append(data["objects"][i]["points"]["exterior"])
				else:
					print("No list for " + data["objects"][i]["classTitle"])
			
			img_data.append([yellow_cones, blue_cones, orange_cones, large_orange_cones, unknown_cones])
	return img_data
