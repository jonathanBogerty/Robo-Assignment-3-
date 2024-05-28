import cv2
import numpy as np
import math
import time
from matplotlib import pyplot as plt
import serial
import time

min_distance_vertical = 40
min_distance_horizontal = 60

cam=cv2.VideoCapture(0)
while True:
    #e1 = time.time()
    ret,image=cam.read()
   
    vertical_lines = 0
    vertical_lines_positions = []
    horizontal_lines_positions = []

    image = cv2.resize(image, (640, 480))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,120,180)
    #e2 = time.time()
    #print(e2-e1)
    lines = cv2.HoughLines(edges, 2, np.pi / 160, 90, None, 0, 0)

    if lines is not None:
       for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))

            if theta < np.pi/4 or theta > 3 * np.pi/4:
                too_close = False
                for x in vertical_lines_positions:
                 if (x0 - x) < min_distance_vertical:
                    too_close = True

                if vertical_lines < 2:
                    if too_close == False:
                        cv2.line(image, pt1, pt2, (0,0,255), 3)
                        vertical_lines_positions.append(x0)
                        vertical_lines += 1
            
            else:
                too_close = False
                for y in horizontal_lines_positions:
                    if (y0 - y) < min_distance_horizontal:
                        too_close = True

                if too_close == False:
                    cv2.line(image, pt1, pt2, (0,255,0), 3)
                    horizontal_lines_positions.append(y0)

    if vertical_lines == 2:
            x_centre = int((vertical_lines_positions[0] + vertical_lines_positions [1]) / 2)
            y_centre = int(image.shape[0] / 2)

            centre_point = (x_centre, y_centre)
            cv2.circle(image, centre_point, 5, (255, 0, 0), -1)

            print(f"Centre Point: {centre_point}")

    cv2.imshow("hough transform", image)
    cv2.imshow("Camera", edges)
    k=cv2.waitKey(1)
    if k==27:
        break
cam.release()
cv2.destroyAllWindows

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    while True:
        #ser.write(b"Hello from Raspberry Pi!\n")
        #line = ser.readline().decode('utf-8').rstrip()
        #print(line)
        time.sleep(1)