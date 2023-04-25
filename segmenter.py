import torch
import model
import cv2

import rospy
from sensor_msgs import Image
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
    cv2.imwrite(f"segmented{image_count:05d}", int_mask)
    pub.publish(img_msg)

    # del image # not sure if garbage collection needed here, underlying image data might be needed by other msgs

if __name__ == '__main__':
    model = VeinU_Net(64, 0).to(device)
    model.eval()

    bridge = CvBridge()
    image_count = 0
    
    rospy.init_node('segmenter', anonymous=True)

    rospy.Subscriber('image_data', Image, callback)

    pub = rospy.Publisher('segmented_images', Image, queue_size = 20)

    rospy.spin()
