import sys
import numpy as np
import cv2

DISCRETIZATION = 21

# Use for creating new csv file from image files.

# arg1 image file path
# arg2 serial number

# example
# list=`find Images_from_rosbag/mydata/images | grep jpg | sort`; count=0; for i in $list; do python3 analysis_image.py $i $count; let count++; done > new.csv

def resize(img, rx, ry):
    h, w = img.shape[:2]           # get size of image
    hr = int(h * rx)
    wr = int(w * ry)
    img = cv2.resize(img, (wr, hr)) # Resize image
    return img

def detectCenterLine(img):
    hsvLower = np.array([25, 100, 100])
    hsvUpper = np.array([35, 255, 255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(hsv, hsvLower, hsvUpper)
    img = cv2.bitwise_and(img, img, mask=hsv_mask)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    h, w = gray.shape[:2]
#    for y in range((h * 545 / 1000), h): # adjust for 2.5
    for y in range(int((h * 570 / 1000)), h): # adjust for 1.6 (twist.lenear.x) 
        for x in range(w):
            if gray[y, x] > 0:
                #print(x, y, gray[y, x])
                break
        else:
            continue
        break

    dist = (float(x) - (float(w) / 2)) / (float(w) / 2) * -1
    #print('dist=', dist, x, y)
    cv2.circle(img, (x, y), 10, (0, 0, 255))
    return img, dist

args = sys.argv
img = cv2.imread(args[1])
img = resize(img, 0.5, 0.5)
img, dist = detectCenterLine(img)
label = int((dist * ((DISCRETIZATION - 1) / 2)) + 0.5) + ((DISCRETIZATION - 1) / 2) 
out = "{},{},{}".format(args[2], args[1], label)
print(out)
#cv2.imshow('Test', img)
#cv2.waitKey(0)
#cv2.destroyAllWindows()

