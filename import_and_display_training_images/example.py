# ============================================================================
# Author      : Anton Haes
# Contact     : anton.haes@vub.be
# Description : example showing how to use the import_training
#		and display_training functions
# ============================================================================


from import_training import import_training
from display_training import display_training

path = "../fsoco_bounding_boxes_train/"

img_data = import_training(path)
print("A total of " + str(len(img_data)) + " images have been processed.")

display_training(path, img_data, 25)
