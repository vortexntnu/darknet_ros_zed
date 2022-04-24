import cv2
import os
import numpy
import time


directory = r"D:\Darknet\labelImg\spy_and_gman_images"

img = cv2.imread(
    r'D:\Darknet\labelImg\spy_and_gman_images\spy_and_gman0000.jpg')

print(img.shape)

shape = img.shape[0:2]
print(shape)

with open(r'D:\Darknet\darknet\data\obj2.names', 'r') as f:
    classes = f.read().splitlines()

net = cv2.dnn.readNetFromDarknet(r'D:\Darknet\darknet\cfg\yolov4-tiny-custom_agent_and_badge.cfg',
                                 r'D:\Darknet\darknet\backup_badge_and_spy_v4_tiny_avoid_overfit_clahe\yolov4-tiny-custom_agent_and_badge_6000.weights')
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
model = cv2.dnn_DetectionModel(net)
model.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)

"""
for path in os.listdir(directory):
    if path.endswith(".jpg"):
        img = cv2.imread(directory + "/" + path)
        print(directory + "/" + path)
        start = time.time()

        classIds, scores, boxes = model.detect(
            img, confThreshold=0.4, nmsThreshold=0.4)
        fps = 1/(time.time() - start)

        print(fps)
        for (classId, score, box) in zip(classIds, scores, boxes):

            print(f"ClassId: {classId} Score: {score} Location: {box}")

            cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
                          color=(0, 255, 0), thickness=2)
            print(classes[classId])
            text = '%s: %.2f' % (classes[classId], score)
            cv2.putText(img, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color=(0, 255, 0), thickness=2)

        cv2.imshow('Image', img)
        cv2.waitKey(500)
        cv2.destroyAllWindows() """


for path in os.listdir(directory):
    if path.endswith(".jpg"):
        img = cv2.imread(directory + "/" + path)
        yolostrings = []
        print(directory + "/" + path[:-3] + "txt")
        try:
            with open(directory + "/" + path[:-3] + "txt", "r") as f:
                for line in f.readlines():
                    yolostrings.append(line)
        except:
            print("No such file")

        print(yolostrings)
        print(directory + "/" + path)
        start = time.time()

        classIds, scores, boxes = model.detect(
            img, confThreshold=0.4, nmsThreshold=0.4)
        fps = 1/(time.time() - start)

        print(fps)
        for (classId, score, box) in zip(classIds, scores, boxes):

            print(f"ClassId: {classId} Score: {score} Location: {box}")

            cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
                          color=(0, 255, 0), thickness=2)

            yolo = []

            yolo.append(((2*box[0] + box[2])/2)/1280)
            yolo.append(((2*box[1] + box[3])/2)/720)
            yolo.append(box[2]/1280)
            yolo.append(box[3]/720)
            yolostring = f"2 {yolo[0]} {yolo[1]} {yolo[2]} {yolo[3]}\n"
            print(yolostring)

            yolostrings.append(yolostring)

        print(directory + "/" + path[:-3] + "txt")
        with open(directory + "/" + path[:-3] + "txt", "w") as f:
            f.writelines(yolostrings)

""" text = '%s: %.2f' % (classes[classId], score)
            cv2.putText(img, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        color=(0, 255, 0), thickness=2)

        cv2.imshow('Image', img)
        cv2.waitKey(80)
        cv2.destroyAllWindows()
 """
""" img = cv2.imread(
    R"D:\Darknet\labelImg\spy_and_gman_images\spy_and_gman0004.jpg")
start = time.time()

classIds, scores, boxes = model.detect(
    img, confThreshold=0.4, nmsThreshold=0.4)
fps = 1/(time.time() - start)

print(fps)
for (classId, score, box) in zip(classIds, scores, boxes):

    print(f"ClassId: {classId} Score: {score} Location: {box}")

    cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]),
                  color=(0, 255, 0), thickness=2)

    # top left: (box[0], box[1])

    # xmin = box[0], ymin = box[1], width = box[2], height = box[3]

    # yoloformat: ((box[0] + box[2])/2)/1280  ((box[1] + box[3])/2)/720 box[2]/1280 box[3]/1280

    yolo = []

    yolo.append(((2*box[0] + box[2])/2)/1280)
    yolo.append(((2*box[1] + box[3])/2)/720)
    yolo.append(box[2]/1280)
    yolo.append(box[3]/720)

    yolostring = f"\n2 {yolo[0]} {yolo[1]} {yolo[2]} {yolo[3]}"

    with open(r"D:\Darknet\labelImg\spy_and_gman_images\spy_and_gman0004.txt", "a") as f:
        f.write(yolostring)

    print(yolostring)

    print(classes[classId])
    text = '%s: %.2f' % (classes[classId], score)
    cv2.putText(img, text, (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1,
                color=(0, 255, 0), thickness=2)

cv2.imshow('Image', img)
cv2.waitKey()
cv2.destroyAllWindows() """
