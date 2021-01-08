#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import torch.backends.cudnn as cudnn
import torchvision
import torchvision.transforms as transforms
import torchvision.models as models

import os
import argparse
import numpy as np
import time
from PIL import Image as IMG

import cv2
from cv_bridge import CvBridge

model = models.resnet18()

device = 'cuda' if torch.cuda.is_available() else 'cpu'
speed = np.zeros(5, dtype=np.float64)

def init_inference():
    global model
    global device
    model.fc = torch.nn.Linear(512, 3)
    model.eval()
    #model.load_state_dict(torch.load(args.pretrained_model))
    
    if args.trt_module :
        from torch2trt import TRTModule
        if args.trt_conversion :
            model.load_state_dict(torch.load(args.pretrained_model))
            model = model.cuda()
            x = torch.ones((1, 3, 120, 160)).cuda()
            from torch2trt import torch2trt
            model_trt = torch2trt(model, [x], max_batch_size=100, fp16_mode=True)
            #model_trt = torch2trt(model, [x], max_batch_size=100)
            torch.save(model_trt.state_dict(), args.trt_model)
            exit()
        model_trt = TRTModule()
        #model_trt.load_state_dict(torch.load('road_following_model_trt_half.pth'))
        model_trt.load_state_dict(torch.load(args.trt_model))
        model = model_trt.to(device)
    else :
        model.load_state_dict(torch.load(args.pretrained_model))
        model = model.to(device)
        
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
tmp = torch.zeros([1,3,120,160])
bridge = CvBridge()
twist = Twist()

def detectColor(img):
    hsvLower = np.array([25, 100, 100])
    hsvUpper = np.array([35, 255, 255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(hsv, hsvLower, hsvUpper)
    img = cv2.bitwise_and(img, img, mask=hsv_mask)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, gray = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    h, w = gray.shape[:2] 
#    for y in range((h * 545 / 1000), h):
    for y in range((h * 545 / 1000), h):
        for x in range(w):
            if gray[y, x] > 0:
                print(x, y, gray[y, x])
                break
        else:
            continue
        break

    dist = (float(x) - (float(w) / 2)) / (float(w) / 2) * -1
    print('dist=', dist, x, y)
    return dist

def set_throttle_steer(data):
    global i
    global pre
    global now
    global tmp
    global bridge
    global twist
    global model
    global devic
    global speed

    i=i+1
    if i == 100 :
        pre = now
        now = time.time()
        i = 0
        print ("average_time:{0}".format((now - pre)/100) + "[sec]")
    start = time.time()
    img = bridge.imgmsg_to_cv2(data, "bgr8")
   
    img = resize(img, 0.5, 0.5) 
    dist = detectColor(img)

    if args.median_control:
        speed = np.roll(speed, 1)
        speed[0] = dist
        print(speed, np.median(speed))
        dist = np.median(speed)
    
    #angular_z = (float(output)-256)/100
    angular_z = dist
    twist.linear.x = args.speed #1.6
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = angular_z
    twist_pub.publish(twist)
    end = time.time()
    print ("time_each:{0:.3f}".format((end - start)) + "[sec]")


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
	
	arg_parser.add_argument("--trt_conversion", action='store_true')
	arg_parser.add_argument("--trt_module", action='store_true')
	arg_parser.add_argument("--pretrained_model", type=str, default='/home/shiozaki/work/experiments/models/checkpoints/sim_race_joycon_ResNet18_6_epoch=20.pth')
	arg_parser.add_argument("--trt_model", type=str, default='road_following_model_trt.pth' )
	arg_parser.add_argument("--speed", type=float, default=1.6)
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
