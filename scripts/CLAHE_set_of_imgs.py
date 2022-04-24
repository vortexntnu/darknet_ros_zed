from re import I
from img_preprocesser import img_preprocesser

import os
import cv2 as cv

my_preprocesser = img_preprocesser(2, 8)


img_path = r"D:\Darknet\mclab_badge_and_spy_close_wiggle_labeled"
front_cutoff = "./mclab_training_data/mclab_training_data/"
write_path = r"D:\Darknet\mclab_training_data_CLAHEd"
""" 
for img in glob.glob(img_path):
    img_n = cv.imread(img)

    save_name = img[len(front_cutoff):]
    img_clahe = my_preprocesser.CLAHE(img_n)

    cv.imwrite(os.path.join(write_path, save_name), img_clahe) """


for img in os.listdir(img_path):

    if img[-4:] == ".jpg":
        print("Saving CLAHE image to : " + img_path + "/" + img)
        img_n = cv.imread(img_path + "/" + img)
        img_clahe = my_preprocesser.CLAHE(img_n)
        cv.imwrite(write_path + "/" + img, img_clahe)
