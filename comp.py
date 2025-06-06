import cv2
import numpy as np

# Load the image
image = cv2.imread('frame_6.png')
image = cv2.transpose(image)
image = cv2.flip(image, flipCode=1)
# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


# Apply a binary threshold to the grayscale image
_, thresh = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
thresh = cv2.bitwise_not(thresh)

# Find contours in the thresholded image
contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
print(contours)
# Count closed contours
closed_contours_count = 0
for contour in contours:
            closed_contours_count += 1

print(f"Number of closed contours: {closed_contours_count}")

# Optional: Draw contours on the image and save/display it
output_image = image.copy()
cv2.drawContours(output_image, contours, -1, (0, 0, 0), 10)
cv2.imshow("Camera View", output_image)
cv2.waitKey(0)
