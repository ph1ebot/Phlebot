from jetcam.csi_camera import CSICamera
import cv2
import threading
import numpy as np

camera = CSICamera(width = 640, height = 480, capture_width = 640, capture_height = 480, capture_fps = 5)

# Set the camera to run async with callback to save image locally
camera.running = True

latest_image = None
image_lock = threading.Lock()

def callback(change):
    new_image = change['new']
    with image_lock:
        global latest_image
        latest_image = np.copy(new_image)

camera.observe(callback, names='value')
# Prompt user for a number of images to capture
number_received = False
while not number_received:
    try:
        num_images = int(input("How many images would you like to capture?\n"))
        number_received = True
        print(f"You chose {num_images} images")
    except:
        print("Please enter a valid integer!")

# Take pictures upon user keyboard prompt
pictures_taken = 0
while pictures_taken < num_images:
    try:
        prompt = input("Press any character and enter to capture an image.")
        '''
        #ret = True
        images_captured = 0
        while not images_captured:
            ret, image = camera.cap.read()
            if ret:
                images_captured += 1
        pictures_taken += 1
        print(image.shape)
        cv2.imwrite(f"image{pictures_taken:04d}.jpg", image)
        '''
        with image_lock:
            image = latest_image
            if image is None:
                print("none image")
            cv2.imwrite(f"image{pictures_taken:04d}.jpg", image)
            pictures_taken += 1
    except Exception as e:
        print(f"Something went wrong with taking pictures: {e}. Try again!")

print("Capture complete! Have a great day!")
