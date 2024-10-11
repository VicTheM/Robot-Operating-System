# (c) robotics.snowcron.com
# Use: MIT license

# This code will create an image for one of Aruco default markers
import cv2

# Define the dictionary to use
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

# Generate the marker image
marker_size = 200  # in pixels
marker_id = 42
marker_image = cv2.aruco.drawMarker(dictionary, marker_id, marker_size)

# Save the marker image as a PNG file
cv2.imwrite("aruco_marker_42.png", marker_image)
