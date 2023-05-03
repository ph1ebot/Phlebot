#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from jetcam.csi_camera import CSICamera
import cv2
#import threading
import numpy as np
from cv_bridge import CvBridge
import ctypes
libgcc_s = ctypes.CDLL('libgcc_s.so.1') # this line is to avert a runtime error that comes from the jetcam library's cv2.VideoCapture call. A call to pthread_create fails because this binary cannot be found.

def cam():
        pub = rospy.Publisher('image_data', Image, queue_size = 1)
        rospy.init_node('cam', anonymous=True)
        rate = rospy.Rate(1)

        # init jetcam
        camera = CSICamera(width = 640, height = 480, capture_width = 640, capture_height = 480, capture_fps = 1)
        
        # Preprocessing items
        bridge = CvBridge()
        correction = cv2.imread('correction_mask.jpg')
        params = np.load('camera_params.npz')
        mtx = params['mtx']
        dist = params['dist']
        newcameramtx = params['newcameramtx']
        roi = params['roi']
       
        avg_mask = cv2.imread('correction_mask.jpg')

        image_count = 0
        while not rospy.is_shutdown():
            image_data = np.copy(camera.read())
            # Preprocess
            CORRECTION = 1.5
            norm = np.mean(avg_mask, axis=2, keepdims = True)

            img = image_data / avg_mask * norm / CORRECTION
            max_intensity = np.amax(img)
            if max_intensity > 255:
                print(f"Image {image_count} slammed against color rail: {max_intensity}")
                img = img * 255 / max_intensity
        
            img = img.astype('uint8')

            # undistort
            dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
            # crop the image
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]
            image_data = np.pad(dst, ((0,1), (0,1), (0,0)))
            # Convert to img msg
            new_image = bridge.cv2_to_imgmsg(image_data, encoding="passthrough")
                
            rospy.loginfo(f"Image {image_count}")
            image_count += 1
            pub.publish(new_image)
            rate.sleep()

if __name__ == '__main__':
        try:
            cam()
        except rospy.ROSInterruptException:
            print("wtf")
            pass
