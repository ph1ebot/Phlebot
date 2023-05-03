#!/usr/bin/env python
import cv2
import sys
import gc
import time
import numpy as np
import os

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tensorrt as trt 
import pycuda.driver as cuda
#import pycuda.autoinit


def predict(image):
    global output
    global cuda_driver_context
    # transfer data to device
    cuda_driver_context.push()
    cuda.memcpy_htod_async(d_input, image, stream)
    # execute model
    rospy.loginfo("bkpt 1")
    context.execute_async_v2(bindings, stream.handle, None)
    # transfer predictions back
    rospy.loginfo("bkpt 2")
    cuda.memcpy_dtoh_async(output, d_output, stream)
    # synchronize threads
    stream.synchronize()
    cuda_driver_context.pop()
    #return output

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " received image. Segmenting...")
    
    # Convert img msg into cv image
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough')
    # then format into the batch shape (480 x 640 x 3) -> (1 x 3 x 480 x 640)
    cv_image = cv_image.transpose(2, 0, 1)
    cv_image = cv_image.astype(np.float16)

    # need to convert from uint8 to float
    cv_image /= 255
    cv_image = np.expand_dims(cv_image, axis = 0)
    assert cv_image.shape == (1, 3, 480, 640)

    # convert into torch tensor
    #image = torch.from_numpy(np.expand_dims(cv_image,axis=0)).to(device)
    
    # TensorRT way
    image =  np.ascontiguousarray(cv_image, dtype = np.float16) 
    start = time.time()
    #logits = predict(image)
    predict(image)
    global output
    logits = output

    duration = time.time() -start
    #high_logits = np.exp(logits[0, 1])
    #low_logits = np.exp(logits[0, 0])
    #mask = high_logits / (high_logits + low_logits)
    mask = np.argmax(logits, axis = 1)
    rospy.loginfo(f"mask nonzeros: {np.count_nonzero(mask)}")

    # convert to an appropriate type (may not work)
    # mask = mask.astype('int') # for now skip it and hope that bridge can handle float64
    # for reference, check out cv2 CvtColor and ConvertScale

    # publish to the topic
    img_msg = bridge.cv2_to_imgmsg(mask.astype(np.float32), encoding='passthrough')

    #int_mask = (mask * 255).astype('uint8')
    rospy.loginfo(f"Number of classification: {np.count_nonzero(mask > 0.5)}")
    masked_image = np.copy(image[0])# 3 x 480 x 640
    big_mask = np.repeat(mask, 3, 1)
    masked_image[big_mask == 1] = 1 # mask should be 1 x 480 x 640
    global image_count
    rospy.loginfo(f"Inference on image {image_count} in {duration} seconds")
    debug_image =  (masked_image*255).astype(np.uint8)
    rospy.loginfo(f"The debug image has shape {debug_image.shape} and type {type(debug_image)} and dtype {debug_image.dtype}")
    cv2.imwrite(f"segmented{image_count:05d}.jpg", debug_image)
    image_count += 1
    pub.publish(img_msg)

    # del image # not sure if garbage collection needed here, underlying image data might be needed by other msgs

if __name__ == '__main__':
    
    # Test fix
    cuda.init()
    device = cuda.Device(0)
    cuda_driver_context = device.make_context()

    # Load model, tensorrt way
    f = open("unext_pytorch.trt", "rb")
    runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
    engine = runtime.deserialize_cuda_engine(f.read())
    context = engine.create_execution_context()

    input_batch = np.empty([1, 3, 480, 640], dtype = np.float16)
    output = np.empty([1, 2, 480, 640], dtype =np.float16)
    d_input = cuda.mem_alloc(1 * input_batch.nbytes)
    d_output = cuda.mem_alloc(1 * output.nbytes)

    #del input_batch
    gc.collect()

    bindings = [int(d_input), int(d_output)]
    stream = cuda.Stream()

    print("Model loaded. Let's go!") 
    bridge = CvBridge()
    image_count = 0
    
    rospy.init_node('segmenter', anonymous=True)

    rospy.Subscriber('image_data', Image, callback)

    pub = rospy.Publisher('segmented_images', Image, queue_size = 20)

    rospy.spin()
