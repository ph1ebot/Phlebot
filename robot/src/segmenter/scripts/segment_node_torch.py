#!/usr/bin/env python3
import torch
import cv2
import sys
import gc
import time
import numpy as np

import rospy
from segmenter import model as M
#import segmenter.model as M
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#import tensorrt as trt 
#import pycuda.driver as cuda
#import pycuda.autoinit

my_device = 'cuda'

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " received image. Segmenting...")
    global model
    global bridge
    global my_device
    
    # Convert img msg into cv image
    cv_image = np.copy(bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough'))
    cv_image = cv_image.transpose(2, 0, 1)
    cv_image = cv_image.astype(np.float32)

    # need to convert from uint8 to float
    cv_image /= 255
    # convert into torch tensor
    image = torch.from_numpy(np.expand_dims(cv_image,axis=0)).to(my_device)
    
    start = time.time()
    # segment (vanilla pytorch)
    with torch.inference_mode():
        classification = model(image)
        # convert binarized mask to cv image (numpy)
        probs = torch.argmax(classification, dim = 1) # channel 1 is the vein channel
        mask = probs.cpu().numpy() 
    duration = time.time() - start

    # convert to an appropriate type (may not work)
    # mask = mask.astype('int') # for now skip it and hope that bridge can handle float64
    # for reference, check out cv2 CvtColor and ConvertScale

    # publish to the topic
    img_msg = bridge.cv2_to_imgmsg(mask[0].astype(np.float32), encoding='passthrough')

    #int_mask = (mask * 255).astype('uint8')
    rospy.loginfo(f"Number of classification: {np.count_nonzero(mask > 0.5)}")
    masked_image = np.copy(image[0].cpu().numpy())# 3 x 480 x 640
    big_mask = np.repeat(mask, 3, 0)
    masked_image[big_mask == 1] = 1 # mask should be 1 x 480 x 640
    rospy.loginfo(f"Image shape: {masked_image.shape}, {masked_image.dtype} Mask shape: {big_mask.shape}, {big_mask.dtype}")

    global image_count
    masked_image = masked_image.transpose(1, 2, 0)
    rospy.loginfo(f"Inference on image {image_count} in {duration} seconds")
    
    debug_image = (masked_image*255).astype(np.uint8)
    #debug_image = np.ones(masked_image.shape, dtype = np.uint8) *255
    rospy.loginfo(f"The debug image has shape {debug_image.shape} and type {type(debug_image)} and dtype {debug_image.dtype}")

    status = cv2.imwrite(f"segmented{image_count:05d}.jpg", debug_image)
    rospy.loginfo(f"CV2 saved image correctly: {status}")
    image_count += 1
    pub.publish(img_msg)

    # del image # not sure if garbage collection needed here, underlying image data might be needed by other msgs

if __name__ == '__main__':
    # Load model, vanilla pytorch way
    model = M.VeinU_Net(48, 0).to(my_device).eval()
    prev_state = torch.load('UNeXt3_8.pth')
    model.load_state_dict(prev_state['model_state_dict'])
    del prev_state
    gc.collect()
    torch.cuda.empty_cache()
    model.eval()

    gc.collect()

    print("Model loaded. Let's go!") 
    bridge = CvBridge()
    image_count = 0
    
    rospy.init_node('segmenter', anonymous=True)

    rospy.Subscriber('image_data', Image, callback)

    pub = rospy.Publisher('segmented_images', Image, queue_size = 1)

    rospy.spin()
