#!/usr/bin/env python3
import torch
import cv2
import sys
import gc

import rospy
#from segmenter import model as M
import segmenter.model as M
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

device = 'cuda'

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " received image. Segmenting...")
    #global model
    #global bridge
    #global device
    
    # Convert img msg into cv image
    cv_image = cv_bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough')

    # convert into torch tensor
    image = torch.from_numpy(cv_image).to(device)
    
    # segment
    with torch.inference_mode():
        classification = model(image)
        # convert binarized mask to cv image (numpy)
        probs = torch.softmax(classification, dim = 0)[1] # channel 1 is the vein channel
        mask = probs.cpu().numpy()

    # convert to an appropriate type (may not work)
    # mask = mask.astype('int') # for now skip it and hope that bridge can handle float64
    # for reference, check out cv2 CvtColor and ConvertScale

    # publish to the topic
    img_msg = bridge.cv2_to_imgmsg(mask, desired_encoding='64FC')
    
    int_mask = (mask * 255).astype('uint8')
    global image_count
    cv2.imwrite("segmented" + str(image_count), int_mask)
    image_count += 1
    pub.publish(img_msg)

    # del image # not sure if garbage collection needed here, underlying image data might be needed by other msgs

if __name__ == '__main__':
    model = M.VeinU_Net(64, 0).to(device).eval()
    #prev_state = torch.load('UNeXt3_4.pth')
    #model.load_state_dict(prev_state['model_state_dict'])
    #del prev_state
    #gc.collect()
    #torch.cuda.empty_cache()
    #model.eval()
    print("Model loaded. Let's go!") 
    bridge = CvBridge()
    image_count = 0
    
    rospy.init_node('segmenter', anonymous=True)

    rospy.Subscriber('image_data', Image, callback)

    pub = rospy.Publisher('segmented_images', Image, queue_size = 20)

    rospy.spin()
