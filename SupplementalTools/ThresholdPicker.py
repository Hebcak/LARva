import cv2
import numpy as np

def empty(value):
    # print(value)
    pass

color_min = [[0, 0, 0], [0, 0, 0], [0, 0, 0]] # for R, G, B in format (h_min, s_min, v_min)
color_max = [[255, 255, 255], [255, 255, 255], [255, 255, 255]] # for R, G, B in format (h_max, s_max, v_max)
current_color = 0
# create window for trackbars
cv2.namedWindow("TrackBars")  # create window to be able to adjust values in real time
cv2.resizeWindow("TrackBars", 640, 240)  # resize the window
color_dict = {"Red":(0, 0, 255), "Green":(0, 255, 0), "Blue":(255, 0, 0), "Yellow":(0, 255, 255), "Cyan":(255, 255, 0), "Magenta":(255, 0, 255)}

# create a trackbars, in opencv max hue is 179, the trackbar calls function empty on change
cv2.createTrackbar("Color", "TrackBars", 0, 2, empty)
cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)
cv2.createTrackbar("Hue Max", "TrackBars", 179, 179, empty)
cv2.createTrackbar("Sat Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Sat Max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Val Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)

img = cv2.imread("../Data/Robot_view_png.png")
imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert colors from RGB to HSV
color = "Red"
while True:
    new_color = cv2.getTrackbarPos("Color", "TrackBars")
    if (new_color != current_color):
        cv2.setTrackbarPos("Hue Min", "TrackBars", color_min[new_color][0])
        cv2.setTrackbarPos("Hue Max", "TrackBars", color_max[new_color][0])
        cv2.setTrackbarPos("Sat Min", "TrackBars", color_min[new_color][1])
        cv2.setTrackbarPos("Sat Max", "TrackBars", color_max[new_color][1])
        cv2.setTrackbarPos("Val Min", "TrackBars", color_min[new_color][2])
        cv2.setTrackbarPos("Val Max", "TrackBars", color_max[new_color][2])
        current_color = new_color
    else:
        h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
        s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
        s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
        v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
        v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
        color_min[new_color] = [h_min, s_min, v_min]
        color_max[new_color] = [h_max, s_max, v_max]
    lower = np.array(color_min[new_color])
    upper = np.array(color_max[new_color])
    mask = cv2.inRange(imgHSV, lower, upper) #create a mask
    imgResult = cv2.bitwise_and(img, img, mask=mask) #apply mask
    imgContour = img.copy()
    for i in range(3):
        color = "Red"
        if i == 1:
            color = "Green"
        elif i == 2:
            color = "Blue"

        lower = np.array(color_min[i])
        upper = np.array(color_max[i])
        imgMask = cv2.inRange(imgHSV, lower, upper)  # create a mask
        contours, hierarchy = cv2.findContours(imgMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if len(approx) != 4:  # not a square
                cv2.drawContours(imgContour, cnt, -1, color_dict["Cyan"], 2)
                imgContour = cv2.putText(imgContour, "Not a cone", (x, y - 3), cv2.FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 0), 1)
                continue
            if float(h) / w > 8:  # standing Bannister
                imgContour = cv2.drawContours(imgContour, cnt, -1, color_dict[color], 2)
                imgContour = cv2.putText(imgContour, color, (x, y - 3), cv2.FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 0), 1)
            elif float(w) / h > 8:  # Bannister on the floor
                cv2.drawContours(imgContour, cnt, -1, color_dict["Yellow"], 2)
                imgContour = cv2.putText(imgContour, "Fallen", (x, y - 3), cv2.FONT_HERSHEY_COMPLEX, 0.4, (0, 0, 0), 1)
            else:
                continue

    #cv2.imshow("Original", img)
    #cv2.imshow("HSV", imgHSV)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", imgResult)
    cv2.imshow("Contours", imgContour)
    cv2.waitKey(50)