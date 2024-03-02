## Nefunguje zatial
import cv2
import numpy as np
import open3d as o3d
import cv2

# Load images
img1 = cv2.imread('pictures/cube.jpg')
img2 = cv2.imread('pictures/cube.jpg')

# Initialize feature detector
sift = cv2.SIFT_create()

# Find keypoints and descriptors
kp1, des1 = sift.detectAndCompute(img1, None)
kp2, des2 = sift.detectAndCompute(img2, None)

# Initialize feature matcher
bf = cv2.BFMatcher()
matches = bf.knnMatch(des1, des2, k=2)

# Apply ratio test
good_matches = []
for m, n in matches:
    if m.distance < 0.75 * n.distance:
        good_matches.append(m)

# Draw matches
img_matches = cv2.drawMatches(img1, kp1, img2, kp2, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

cv2.imshow('Matches', img_matches)
cv2.waitKey(0)
cv2.destroyAllWindows()
