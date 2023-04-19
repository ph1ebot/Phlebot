import cv2
import os
import numpy as np

pwd = os.getcwd()
images = os.listdir(pwd)
correction_masks = []
for image_name in images:
    if image_name.find("nvcamtest_9434_s00_") == -1:
        continue
    image_path = os.path.join(pwd, image_name)
    print(image_path)
    
    img = cv2.imread(image_path)
    # cv2.imshow(image_name, img)
    correction_masks.append(img)

correction_masks = np.stack(correction_masks)
print(correction_masks.shape)
avg_mask = correction_masks[0]#np.mean(correction_masks, axis = 0)
print(avg_mask.shape)

image_path = os.path.join(pwd, "nvcamtest_10624_s00_00000.jpg")
img = cv2.imread(image_path).astype(np.float64)
cv2.imshow("Original Image", img.astype('uint8'))
print(img[0,0].dtype)

# Normalize
color_scale = 1
norm = np.mean(avg_mask, axis=2, keepdims=True)#np.linalg.norm(avg_mask, axis=2, keepdims=True)
# correction_max = np.amax(avg_mask)
# img_max = np.amax(img)
# if img_max * correction_max > 


img = img / avg_mask * norm
if np.amax(img) > 255:
    img = img * 255 / np.amax(img)

img = img.astype('uint8')
avg_mask = avg_mask.astype('uint8')
cv2.imshow("Average Image", avg_mask)
cv2.imwrite("correction_mask.jpg", avg_mask)
# cv2.waitKey(0)

cv2.imshow("Corrected Image", img)
cv2.imwrite("corrected.jpg", img)
cv2.waitKey(0)
cv2.destroyAllWindows()