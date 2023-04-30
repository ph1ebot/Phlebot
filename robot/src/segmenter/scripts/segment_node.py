#!/usr/bin/env python
#import torch
import cv2
import sys
import gc
import time
import numpy as np

import rospy
#from segmenter import model as M
#import segmenter.model as M
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tensorrt as trt 
import pycuda.driver as cuda
import pycuda.autoinit

#device = 'cuda'

def predict(image):
    # transfer data to device
    cuda.memcpy_htod_async(d_input, image, stream)
    # execute model
    context.execute_async_v2(bindings, stream.handle, None)
    # transfer predictions back
    cuda.memcpy_dtoh_async(output, d_output, stream)
    # synchronize threads
    stream.synchronize()
    return output

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " received image. Segmenting...")
    #global model
    #global bridge
    #global device
    
    # Convert img msg into cv image
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding = 'passthrough')

    # convert into torch tensor
    #image = torch.from_numpy(np.expand_dims(cv_image,axis=0)).to(device)
    
    # segment (vanilla pytorch)
    #with torch.inference_mode():
    #    classification = model(image)
    #    # convert binarized mask to cv image (numpy)
    #    probs = torch.softmax(classification, dim = 1) # channel 1 is the vein channel
    #    mask = probs.cpu().numpy()
    
    # TensorRT way
    image = np.expand_dims(cv_image, axis = 0).astype(np.float16) / 255 
    start = time.time()
    logits = predict(image)
    #high_logits = np.exp(logits[0, 1])
    #low_logits = np.exp(logits[0, 0])
    #mask = high_logits / (high_logits + low_logits)
    mask = np.argmax(logits[0], axis = 0)
    rospy.loginfo(f"mask nonzeros: {np.count_nonzero(mask)}")
    duration = time.time() -start

    # convert to an appropriate type (may not work)
    # mask = mask.astype('int') # for now skip it and hope that bridge can handle float64
    # for reference, check out cv2 CvtColor and ConvertScale

    # publish to the topic
    img_msg = bridge.cv2_to_imgmsg(mask.astype(np.float32), encoding='passthrough')

    #int_mask = (mask * 255).astype('uint8')
    rospy.loginfo(f"Number of classification: {np.count_nonzero(mask > 0.5)}")
    masked_image = np.copy(image[0])# 3 x 480 x 640
    masked_image[mask == 1] = 1 # mask should be 1 x 480 x 640

    global image_count
    rospy.loginfo(f"Inference on image {image_count} in {duration} seconds")
    cv2.imwrite(f"segmented{image_count:05d}.jpg", (masked_image*255).astype(np.uint8))
    image_count += 1
    pub.publish(img_msg)

    # del image # not sure if garbage collection needed here, underlying image data might be needed by other msgs

if __name__ == '__main__':
    # Load model, vanilla pytorch way
    #model = M.VeinU_Net(64, 0).to(device).eval()
    #prev_state = torch.load('UNeXt3_4.pth')
    #model.load_state_dict(prev_state['model_state_dict'])
    #del prev_state
    #gc.collect()
    #torch.cuda.empty_cache()
    #model.eval()

    # Load model, tensorrt way
    f = open("unext_pytorch.trt", "rb")
    runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
    engine = runtime.deserialize_cuda_engine(f.read())
    context = engine.create_execution_context()

    input_batch = np.empty([1, 3, 480, 640], dtype = np.float16)
    output = np.empty([1, 2, 480, 640], dtype =np.float16)
    d_input = cuda.mem_alloc(1 * input_batch.nbytes)
    d_output = cuda.mem_alloc(1 * output.nbytes)

    del input_batch
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
