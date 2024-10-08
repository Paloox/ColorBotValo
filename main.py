import cv2
import os
import numpy as np
import mss
import win32api
import serial
import random
import time
import pyautogui

class MouseMover:
    def __init__(self, serial_port, filter_length=5):
        self.serial_port = serial.Serial(serial_port, 115200)
        self.filter_length = filter_length
        self.x_history = [0] * filter_length
        self.y_history = [0] * filter_length

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def move_mouse(self, x, y):
        self.x_history.append(x)
        self.y_history.append(y)

        self.x_history.pop(0)
        self.y_history.pop(0)

        smooth_x = int(sum(self.x_history) / self.filter_length)
        smooth_y = int(sum(self.y_history) / self.filter_length)

        finalx = smooth_x + 256 if smooth_x < 0 else smooth_x
        finaly = smooth_y + 256 if smooth_y < 0 else smooth_y
        self.serial_port.write(b"M" + bytes([self.clamp(finalx, 0, 255), self.clamp(finaly, 0, 255)]))

    def click(self):
        delay = random.uniform(0.01, 0.1)
        self.serial_port.write(b"C")
        time.sleep(delay)

os.system("color 2")
os.system("cls")

fov = int(input("FOV: "))
fov = fov * 2
sct = mss.mss()

# COM5 should be your arduino port, if not, change it here
mouse_mover = MouseMover('COM5')

screenshot = sct.monitors[1]
screenshot['left'] = int((screenshot['width'] / 2) - (fov / 2))
screenshot['top'] = int((screenshot['height'] / 2) - (fov / 2))
screenshot['width'] = fov
screenshot['height'] = fov
center_x = screenshot['width'] // 2
center_y = screenshot['height'] // 2





#yellow
embaixo = np.array([30, 125, 150]) 
emcima = np.array([30, 255, 255])

# replace 0.35 with your sensitivity
speed = 1 / (7 * 0.35)
os.system("cls")
print("Loaded")

min_movement = 0.5 


def darken_image(img, factor=0.2):
    return (img * factor).astype(np.uint8)

def create_circular_mask(h, w, center=None, radius=None):
    if center is None: 
        center = (int(w/2), int(h/2))
    if radius is None:  
        radius = min(center[0], center[1], w-center[0], h-center[1])

    Y, X = np.ogrid[:h, :w]
    dist_from_center = np.sqrt((X - center[0])**2 + (Y - center[1])**2)

    mask = dist_from_center <= radius
    return mask


sct = mss.mss()

screen_width = sct.monitors[1]['width']
screen_height = sct.monitors[1]['height']


capture_width = fov
capture_height = fov


top = (screen_height - capture_height) // 2
left = (screen_width - capture_width) // 2


monitor = {"top": top, "left": left, "width": capture_width, "height": capture_height}

circular_mask = create_circular_mask(fov, fov)
pyautogui.FAILSAFE = False
while True:
    screenshot = sct.grab(monitor)
    img = np.array(screenshot)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, embaixo, emcima)

    black_img = np.zeros_like(img)

    highlighted_img = cv2.bitwise_and(img, img, mask=mask)

    cv2.imshow("screen", highlighted_img)

    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(mask, kernel, iterations=5)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    if win32api.GetAsyncKeyState(0x05) < 0:   # hintere maus taste x
        if contours:

            contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(contour)
            y_offset = int(h * 0.3)
            center = (x + w // 2, y + h // 2)
            cX = center[0]
            cY = y + y_offset
            
            x_diff = cX - monitor['width'] // 2
            y_diff = cY - monitor['height'] // 2

            if abs(x_diff) > min_movement or abs(y_diff) > min_movement:
                mouse_mover.move_mouse(x_diff * speed, y_diff * speed)


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
