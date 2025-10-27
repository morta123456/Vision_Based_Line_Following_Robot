import numpy as np
import cv2 as cv

from random import randint
from math import *

LOWERYELLOW = np.array([23, 100, 80])
UPPERYELLOW = np.array([60, 230, 255])

# LOWERORANGE = np.array([0, 20, 10])
# UPPERORANGE= np.array([20, 255, 255])

# LOWERORANGE = np.array([10, 80, 100])
# UPPERORANGE= np.array([20, 170, 165])

LOWERORANGE = np.array([5, 50, 50])
UPPERORANGE= np.array([15, 255, 255])

BLACK_THRESHOLD = 100


class LineFollower():

    def __init__(self, kernel_size = 3, debug: bool = True) -> None:

        self.kernel_size = kernel_size
        self.debug_color = (randint(0,256), randint(0,256), randint(0,256))
        self.debug = debug

        self.wait_orange = True
        self.current_color = -1
        self.control_color = (LOWERORANGE, UPPERORANGE, None)
        self.colors = [
            (None, None, BLACK_THRESHOLD),
            (LOWERYELLOW, UPPERYELLOW, None)
        ]
        self.orange_threshold = 1800

        self.last_center = np.array([96, 72])


    def mask(self, img, lower=None, upper=None, threshold=BLACK_THRESHOLD):
        if lower is None:
            return self.binary_mask(img, threshold)
        
        return self.color_mask(img, lower, upper)

    def color_mask(self, img, lower, upper):
        # Gaussian blur
        # blur_img = cv.GaussianBlur(img,(self.kernel_size, self.kernel_size), 0)

        # image = cv.cvtColor(blur_img,cv.COLOR_BGR2HSV)
        image = cv.cvtColor(img,cv.COLOR_BGR2HSV)
        mask = cv.inRange(image, lower, upper) 

        mask = cv.erode(mask, (self.kernel_size, self.kernel_size), iterations=1)
        mask = cv.dilate(mask, (self.kernel_size, self.kernel_size), iterations=1)
        return mask
    
    def binary_mask(self, img, threshold):
        # blur_img = cv.medianBlur(img, self.kernel_size)

        # image = cv.cvtColor(blur_img,cv.COLOR_BGR2GRAY)
        image = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        _, mask = cv.threshold(image, threshold, 255, cv.THRESH_BINARY_INV)

        mask = cv.erode(mask, (self.kernel_size, self.kernel_size), iterations=1)
        mask = cv.dilate(mask, (self.kernel_size, self.kernel_size), iterations=1)
        return mask
    
    # Deprecated
    # def get_rect_direction(self, rect):
    #     box = cv.boxPoints(rect)
    #     start1 = box[0]
    #     start2 = box[1]
    #     end1 = box[2]
    #     end2 = box[3]
    #     angle = 90 - rect[2] 
    #     # print("length : ", np.linalg.norm(box[0] - box[3]))
    #     if np.linalg.norm(box[0] - box[3]) < np.linalg.norm(box[0] - box[1]):
    #         start1 = box[0]
    #         start2 = box[3]
    #         end1 = box[1]
    #         end2 = box[2]
    #         angle += 90
            
    #     start = mathutils.lerp(start1, start2, 0.5)
    #     end = mathutils.lerp(end1, end2, 0.5)
    #     # print("angle : ", angle)

    #     return start, end, angle
    
    def update(self, frame):
        orange_mask = self.color_mask(frame, LOWERORANGE, UPPERORANGE)
        count = cv.countNonZero(orange_mask)
        if self.wait_orange and count >= self.orange_threshold:
            self.current_color += 1
            self.wait_orange = False
            if self.current_color >= len(self.colors):
                return None, None, True
        elif count < self.orange_threshold:
            self.wait_orange = True
        
        center, angle = None, None
        bottom = np.array([frame.shape[1] // 2, frame.shape[0]])
        drawing = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
        if self.current_color >= 0 :
            # frame = cv.rotate(frame, cv.ROTATE_180)
            m = self.mask(frame, self.colors[self.current_color][0], self.colors[self.current_color][1], self.colors[self.current_color][2])
            
            # Récupération des contours
            contours, _ = cv.findContours(m, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # On considère le contour le plus étendu et on calcule le centre de masse du contour
                contour = max(contours, key=cv.contourArea)
                M = cv.moments(contour)
                cX = int(M["m10"] / (M["m00"] + 1e-5))
                cY = int(M["m01"] / (M["m00"] + 1e-5))

                # Calcule de l'angle par rapport au bas de l'image
                center = np.array([cX, cY])
                angle = np.arccos((cX - frame.shape[1] // 2) / np.linalg.norm(center - bottom))

        if self.debug:
            if angle is not None:
                # print("computed angle : ", np.rad2deg(angle))
                cv.drawContours(frame, contours, -1, (0,255,0), 1)
                # cv.line(drawing, np.intp(bottom), np.intp(center), self.debug_color)
                # cv.imshow("boxes", drawing)
                cv.imshow('mask', m)
            # print("Orange pixels count ", count)
            cv.imshow('orange mask', orange_mask)
            cv.imshow('frame', frame)

        return center, angle, False
    
    def update_bis(self, frame):
        orange_mask = self.color_mask(frame, LOWERORANGE, UPPERORANGE)
        count = cv.countNonZero(orange_mask)
        if self.wait_orange and count >= self.orange_threshold:
            self.current_color += 1
            self.wait_orange = False
            if self.current_color >= len(self.colors):
                return None, None, True
        elif count < self.orange_threshold:
            self.wait_orange = True
        
        center, angle = None, None
        bottom = np.array([frame.shape[1] // 2, frame.shape[0]])
        # drawing = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)
        if self.current_color >= 0 :
            # frame = cv.rotate(frame, cv.ROTATE_180)
            m = self.mask(frame, self.colors[self.current_color][0], self.colors[self.current_color][1], self.colors[self.current_color][2])
            
            # Récupération des contours
            contours, _ = cv.findContours(m, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                candidates = []
                cnt = 0
                for i in range(len(contours)):
                    bounding = cv.minAreaRect(contours[i])
                    center, _, angle = bounding
                    box = cv.boxPoints(bounding)
                    y_box = box[0][1]
                    if y_box >= frame.shape[0] -1:
                        cnt += 1
                    candidates.append((y_box, i, np.array(center), angle))

                candidates.sort(reverse=True)
                if cnt > 1:
                    try:
                        contour = min(candidates, key=lambda c: np.linalg.norm(c[2] - self.last_center))
                    except:
                        print("bruh ", candidates)
                else:
                    contour = candidates[0]
                self.last_center = contour[2]
                center = contour[2]
                angle = np.arccos((center[0] - frame.shape[1] // 2) / np.linalg.norm(center - bottom))

        if self.debug:
            if angle is not None:
                # print("computed angle : ", np.rad2deg(angle))
                cv.drawContours(frame, contours, -1, (0,255,0), 1)
                cv.line(frame, np.intp(bottom), np.intp(center), self.debug_color)
                # cv.imshow("boxes", drawing)
                cv.imshow('mask', m)
            # print("Orange pixels count ", count)
            cv.imshow('orange mask', orange_mask)
            cv.imshow('frame', frame)

        return center, angle, False
    
if __name__ == "__main__":
    import time
    from signal import signal, SIGINT

    done = False
    def handle(_,_2):
        global done
        done = True
    signal(SIGINT, handle)
    test = LineFollower(debug=True)
    stream = cv.VideoCapture(0)
    if not stream.isOpened():
        print("Cannot open camera")
        exit(-1)
    stream.set(cv.CAP_PROP_FRAME_HEIGHT, 144)
    stream.set(cv.CAP_PROP_FRAME_WIDTH, 192)
    i = 0
    start = time.monotonic()
    while not done:
        ret, frame = stream.read()
        if not ret:
            print("Died")
            break
        test.update_bis(frame)
        i += 1
        if cv.waitKey(1) == ord('q'):
            break
    end = time.monotonic()
    print(f"{end - start} seconds total, {i} iterations, {i / (end - start)} fps")
