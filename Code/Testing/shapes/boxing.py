import cv2
import numpy as np

def get_hsv(colors, ranges):
    for color in colors:
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        hue_low, hue_high = hsv[0][0][0] - 10, hsv[0][0][0] + 10
        ranges.append((
            np.array([hue_low, 75, 75]),
            np.array([hue_high, 255, 255])))

def make_colors(img, ranges):
    mask_acc = np.zeros((img.shape[0], img.shape[1]), np.uint8)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    for hsv_range in ranges:
        mask_cur = cv2.inRange(img_hsv, hsv_range[0], hsv_range[1])
        mask_acc = cv2.add(mask_acc, mask_cur)
    out_img = cv2.bitwise_and(img, img, mask = mask_acc)
    return out_img

def make_shapes(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    for c in cnts:
        #make_bounding_box(img, c)

        #epsilon = 0.1 * cv2.arcLength(c, True)
        #approx = cv2.approxPolyDP(c, epsilon, True)
        #cv2.drawContours(img, approx, -1, (255, 255, 255), 2)

        hull = cv2.convexHull(c)
        tar_range = len(hull)
        for i in range(tar_range):
            px, py = list(hull[i][0]), list(hull[(i + 1) % tar_range][0])
            cv2.line(img, (px[0], px[1]), (py[0], py[1]), (255, 255, 255), 2)
    return img

def make_bounding_box(img, cnts):
    for cnt in cnts:
        rect_rot = cv2.minAreaRect(cnt)
        rect_pts = cv2.boxPoints(rect_rot)
        for i in range(4):
            p1 = [int(x) for x in rect_pts[0][i]]
            p2 = [int(x) for x in rect_pts[0][(i + 1) % 4]]
            cv2.line(img,
                    (p1[0], p1[1]),
                    (p2[0], p2[1]),
                    (255, 255, 255), 2)


img = cv2.imread("camera.png")
color_def = [    # BGR vals
        np.uint8([[[255,   0,   0]]]),  # blue
        #np.uint8([[[  0, 255,   0]]]),  # green
        #np.uint8([[[  0,   0, 255]]]),  # red
        ]
color_ranges = []
get_hsv(color_def, color_ranges)

out_img = make_colors(img, color_ranges)

out_img = make_shapes(out_img)

cv2.imshow("test", out_img)
cv2.waitKey(0)



