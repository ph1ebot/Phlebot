#!/usr/bin/env python3
import rospy
# from sensor_msgs import Image
# from std_msgs import Float64
# from cv_bridge import CvBridge
import numpy as np
import cv2

# splits an image full of lines into individual lines and solves for their equations
NEEDLE_DIST_THRESHOLD = 60 # more than 30 pixels away, we aren't interested in injecting
VEIN_SIZE_THRESHOLD = 50
needle_start = np.array([169, 427]) # where the needle is: row, col
VISUAL = False
DEBUG = True
'''
Returns a list of valid vein locations parametrized as line equations
Input:
    img - float img with pixels either 0 or 1
'''
def segment_image(img):
    rospy.loginfo("Clustering segmented images")

    img = np.copy(img)

    if VISUAL:
        pass
    limit = img.shape
    segmented = np.zeros(img.shape)

    directions = [np.array([0, 1]), np.array([1, 0]), np.array([0, -1]), np.array([-1, 0])]
    # while there are still 1s (vein pixel) in the image
    index = 1
    while (np.count_nonzero(img == 1)):
        # find a vein pixel
        xs, ys = np.nonzero(img)
        first_coord = np.array([xs[0], ys[0]])

        stack = [first_coord]
        # wavefront from there
        while len(stack) > 0:
            pixel = stack.pop()

            if np.all(np.logical_and(0 <= pixel, pixel <limit)) and \
               img[pixel[0], pixel[1]] and segmented[pixel[0], pixel[1]] != index: # is vein and has not been marked
                for direction in directions:
                    new_pixel = pixel + direction
                    stack.append(new_pixel)

                segmented[pixel[0], pixel[1]] = index # mark it
                img[pixel[0], pixel[1]] = 0 # delete from the open set
        
        if DEBUG: 
            rospy.loginfo("Vein segmented")
        index += 1

    num_veins = index - 1
    
    if VISUAL:
        pass
    veins = []
    for id in range(1, num_veins + 1):
        mask = segmented == id
        size = np.count_nonzero(mask)

        # filter out excessively small veins
        if size > VEIN_SIZE_THRESHOLD:
            veins.append(np.argwhere(mask))
    if DEBUG:
        rospy.loginfo(f"{len(veins)} Veins identified!")
    centers = np.array([np.mean(vein, axis = 0) for vein in veins])
    return veins

'''
Inputs:
    data    - binary image 1 or 0
Outputs:
    1D distance vector to travel. Unit in pixels.
'''
def plan(data, position, limit):
    # get thing to numpy
    # img_array = bridge.imgmsg_to_cv2(data)
    img_array = data
    # segment image into discrete lines
    lines = segment_image(img_array)
    # step 1: draw a ray along which we believe we will travel
    ray = np.array([0, 1]) # 1 in the x, 0 in the y
    global needle_start
    
    if VISUAL:
        t = np.linspace(-200,600,801)
        needle_line = needle_start + ray *np.expand_dims(t, axis=1)
    
    # ax + by + c = 0, a = ray[1], b = -ray[0] c = -a * x - b * y
    travel_line = np.array([ray[1], -ray[0], needle_start[0] * - ray[1] + ray[0] * needle_start[1]])

    # step 2: solve in closed form for the intersection between the travel ray and the veins
    min_dist = 1000
    min_intercept = -np.ones((2,))

    for line in lines:
        # ax + by + c = 0, y = -a/b x - c/b
        center = np.mean(line, axis = 0) # 2,
        
        # new equation a x + by + 1 =0
        ab = np.linalg.inv(line.T @ line) @ line.T @ - np.ones((line.shape[0], 1))
        a = ab[0, 0]
        b = ab[1, 0]
        if VISUAL:
            pass
        # vein_line = np.array([vein_vector[1], -vein_vector[0], center[0] * -vein_vector[1] + center[1] * vein_vector[0]])
        vein_line = np.array([a, b, 1])
        Ab = np.stack((travel_line, vein_line))
        intercept = np.linalg.inv(Ab[:, :2]) @ -Ab[:, 2]

        # Filter out veins that end too far from needle point
        if DEBUG:
            rospy.loginfo(f"Aiming point for vein at {center}: {intercept}")
        augmented = np.concatenate((line, np.ones((line.shape[0], 1))), axis = 1)
        dot = augmented @ travel_line
        min_dist_to_needle = np.min(np.abs(dot))
        if DEBUG:
            rospy.loginfo(f"Expected distance to nearest visible point on vein: {min_dist_to_needle}")
        if min_dist_to_needle > NEEDLE_DIST_THRESHOLD:
            continue
        
        # the planner needs to throw away veins it cannot reach
        direction = np.sign(np.dot(intercept - needle_start, ray))
        distance = np.linalg.norm(intercept - needle_start)
        pixel_vector = direction *distance
        distance_estimate = pixel_vector * 80 / 640 
        destination = distance_estimate + position
        if destination < 0 or destination >= limit:
            rospy.loginfo(f"Vein expected at {destination}. Cannot reach")
            continue

        # # step 3: Choose the closest vein
        if distance < min_dist:
            min_dist = distance
            min_intercept = intercept

    # step 4: Approximate the distance to this vein from the needle
    direction = np.sign(np.dot(min_intercept - needle_start, ray))
    command = min_dist * direction
    if DEBUG:
        rospy.loginfo(f"Going for vein at {min_intercept}. Commanding {command}")
    if VISUAL:
        pass
    return command, min_intercept

    # Step 5: publish the distance
    # pub.publish(command)


if __name__ == '__main__':
    # rospy.init_node('planner', anonymous = True)
    
    # rospy.Subscriber('segmented_data', Image, plan)

    # pub = rospy.Publisher('command', Float64, queue_size = 1)

    # rospy.Rate(5)

    # bridge = CvBridge()
    
    # rospy.spin()

    image = cv2.imread('output01.jpg')
    result = plan(image[:, :, 0])
