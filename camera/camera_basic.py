import cv2
cam = cv2.VideoCapture(0)
print(cam)
cam.release()
cam = cv2.VideoCapture(0)
ret, frame = cam.read()
print(f"Captured image of size {frame.shape()}")
