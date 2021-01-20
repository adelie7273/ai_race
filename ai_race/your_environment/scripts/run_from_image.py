#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import argparse
import numpy as np
import cv2
from cv_bridge import CvBridge

def init_inference():
	return
        
def resize(img, rx, ry):
    h, w = img.shape[:2]           # get size of image
    hr = int(h * rx)
    wr = int(w * ry)
    img = cv2.resize(img, (wr, hr)) # Resize image
    return img

def image_process(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)         # gray scale
    img_sx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3)  # Sobel filter(x)
    img_sy = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3)  # Sobel filter(y)
    img = np.sqrt(img_sx ** 2 + img_sy ** 2)            # combining
    img = cv2.GaussianBlur(img, (9, 9), 0)              # Gaussian
    img = resize(img, 0.5, 0.5)                         # Resize
    return img

i=0
pre = time.time()
now = time.time()
bridge = CvBridge()
twist = Twist()

def detectCenterLine(img):
    hsvLower = np.array([25, 100, 100])
    hsvUpper = np.array([35, 255, 255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(hsv, hsvLower, hsvUpper)
    img = cv2.bitwise_and(img, img, mask=hsv_mask)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    h, w = gray.shape[:2] 
#    for y in range((h * 545 / 1000), h): # adjust for  2.5
    for y in range((h * 570 / 1000), h):
        for x in range(w):
            if gray[y, x] > 0:
                #print(x, y, gray[y, x])
                break
        else:
            continue
        break

    dist = (float(x) - (float(w) / 2)) / (float(w) / 2) * -1
    return dist

speed = 1.5

def set_throttle_steer(data):
    global i
    global pre
    global now
    global bridge
    global twist
    global speed

    i=i+1
    if i == 20 :
        pre = now
        now = time.time()
        i = 0
        print ("average_time:{0}".format((now - pre)/20) + "[sec]")
        if args.variable_speed:
            speed = speed + 0.1
            if speed  > 3.2:
                speed = 1.5
    start = time.time()
    img = bridge.imgmsg_to_cv2(data, "bgr8")
   
    img = resize(img, 0.5, 0.5) 
    dist = detectCenterLine(img)

    if args.median_control:
        speed = np.roll(speed, 1)
        speed[0] = dist
        print(speed, np.median(speed))
        dist = np.median(speed)
    
    angular_z = dist
    if args.variable_speed:
        twist.linear.x = speed
    else:
        twist.linear.x = args.speed #1.6

    time.sleep(0.10)
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_z
    twist_pub.publish(twist)
    end = time.time()
    print ("speed:{0:.2f}".format(twist.linear.x) + " angular:{0:.3f}".format(twist.angular.z) + " time_each:{0:.3f}".format((end - start)) + "[sec]")


def inference_from_image():
    global twist_pub
    rospy.init_node('inference_from_image', anonymous=True)
    twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/front_camera/image_raw", Image, set_throttle_steer)
    r = rospy.Rate(10)
    #while not rospy.is_shutdown():
    #    r.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def parse_args():
	# Set arguments. 
	arg_parser = argparse.ArgumentParser(description="Autonomous with inference")
	
	arg_parser.add_argument("--speed", type=float, default=1.6)
        arg_parser.add_argument("--variable_speed", action='store_true')
        arg_parser.add_argument("--median_control", action='store_true')
	args = arg_parser.parse_args()

	return args

if __name__ == '__main__':
    args = parse_args()
#    init_inference()
    try:
        inference_from_image()
    except rospy.ROSInterruptException:
        pass
