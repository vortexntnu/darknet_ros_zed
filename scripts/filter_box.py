import cv2
import os
import numpy
import time


directory = r"D:\Darknet\labelImg\spy_and_gman_images"

img = cv2.imread(
    r'D:\Darknet\labelImg\spy_and_gman_images\spy_and_gman0094.jpg')

for path in os.listdir(directory):
    if path.endswith(".jpg"):
        img = cv2.imread(directory + "/" + path)
        yolostrings = []
        print(directory + "/" + path[:-3] + "txt")

        try:
            with open(directory + "/" + path[:-3] + "txt", "r") as f:
                first_string = f.readline()
                yolostrings.append(first_string)

        except:
            print("No such file")
        try:
            with open(directory + "/" + path[:-3] + "txt", "r") as f:
                for line in f.readlines():
                    temp_string = line
                    line = line.split(" ")
                    if line[0] != "4":
                        print(line)
                        first_line = yolostrings[0].split(" ")
                        print(first_line)
                        if (abs(float(line[1]) - float(first_line[1])) < 0.1):
                            print("duplicate")
                        else:
                            yolostrings.append(temp_string)
        except:
            print("No such file")

        #print(yolostrings)
        print(directory + "/" + path[:-3] + "txt")
        with open(directory + "/" + path[:-3] + "txt", "w") as f:
            f.writelines(yolostrings)
