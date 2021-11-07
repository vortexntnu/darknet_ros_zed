import numpy as np
import pyautogui
import cv2

dir = "/home/vortex/vortex_ws/src/darknet_ros_zed/screenshots2/"


counter = 121
for i in range(300):
    counter += 1
    filename = f"{dir}{counter}.jpg"
    image = pyautogui.screenshot(region=(0, 35, 1920/2, 376 +160))
    image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    cv2.imwrite(filename, image)
    print(f"Image saved at: {filename}")