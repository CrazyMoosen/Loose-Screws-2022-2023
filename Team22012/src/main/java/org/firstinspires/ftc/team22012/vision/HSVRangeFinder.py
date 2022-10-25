# # finding hsv range of target object(pen)
# import cv2
# import numpy as np
# import time
#
#
# # A required callback method that goes into the trackbar function.
# def nothing(x):
#     pass
#
#
# # Initializing the webcam feed.
# cap = cv2.VideoCapture(0)
# cap.set(3, 1280)
# cap.set(4, 720)
#
# # Create a window named trackbars.
# cv2.namedWindow("Trackbars")
#
# # Now create 6 trackbars that will control the lower and upper range of
# # H,S and V channels. The Arguments are like this: Name of trackbar,
# # window name, range,callback function. For Hue the range is 0-179 and
# # for S,V its 0-255.
# cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
# cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
# cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
# cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
# cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
#
# while True:
#
#     # Start reading the webcam feed frame by frame.
#     ret, frame = cap.read()
#     if not ret:
#         break
#     # Flip the frame horizontally (Not required)
#     frame = cv2.flip(frame, 1)
#
#     # Convert the BGR image to HSV image.
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#
#     # Get the new values of the trackbar in real time as the user changes
#     # them
#     l_h = cv2.getTrackbarPos("L - H", "Trackbars")
#     l_s = cv2.getTrackbarPos("L - S", "Trackbars")
#     l_v = cv2.getTrackbarPos("L - V", "Trackbars")
#     u_h = cv2.getTrackbarPos("U - H", "Trackbars")
#     u_s = cv2.getTrackbarPos("U - S", "Trackbars")
#     u_v = cv2.getTrackbarPos("U - V", "Trackbars")
#
#     # Set the lower and upper HSV range according to the value selected
#     # by the trackbar
#     lower_range = np.array([l_h, l_s, l_v])
#     upper_range = np.array([u_h, u_s, u_v])
#
#     # Filter the image and get the binary mask, where white represents
#     # your target color
#     mask = cv2.inRange(hsv, lower_range, upper_range)
#
#     # You can also visualize the real part of the target color (Optional)
#     res = cv2.bitwise_and(frame, frame, mask=mask)
#
#     # Converting the binary mask to 3 channel image, this is just so
#     # we can stack it with the others
#     mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
#
#     # stack the mask, orginal frame and the filtered result
#     stacked = np.hstack((mask_3, frame, res))
#
#     # Show this stacked frame at 40% of the size.
#     cv2.imshow('Trackbars', cv2.resize(stacked, None, fx=0.4, fy=0.4))
#
#     # If the user presses ESC then exit the program
#     key = cv2.waitKey(1)
#     if key == 27:
#         break
#
#     # If the user presses `s` then print this array.
#     if key == ord('s'):
#         thearray = [[l_h, l_s, l_v], [u_h, u_s, u_v]]
#         print(thearray)
#
#         # Also save this array as penval.npy
#         np.save('hsv_value', thearray)
#         break
#
# # Release the camera & destroy the windows.
# cap.release()
# cv2.destroyAllWindows()


import cv2
import numpy as np


def nothing(x):
    pass


def scale(img, scale=0.4):
    dim1 = int(img.shape[1] * scale)
    dim2 = int(img.shape[0] * scale)
    dims = (dim1, dim2)
    return cv2.resize(img, dims, interpolation=cv2.INTER_AREA)


# Load image
image = cv2.imread('C:/School/Robotics/FTC 22012/Cone Detection Pythion/blueCone.png')

image = scale(image, 0.3)

(h, w) = image.shape[:2]
(cX, cY) = (w // 2, h // 2)
# rotate our image by 45 degrees around the center of the image
# M = cv2.getRotationMatrix2D((cX, cY), -90, 1.0)
# image = cv2.warpAffine(image, M, (w, h))
# Create a window

cv2.namedWindow('image')

# Create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

while (1):
    # Get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'image')
    sMin = cv2.getTrackbarPos('SMin', 'image')
    vMin = cv2.getTrackbarPos('VMin', 'image')
    hMax = cv2.getTrackbarPos('HMax', 'image')
    sMax = cv2.getTrackbarPos('SMax', 'image')
    vMax = cv2.getTrackbarPos('VMax', 'image')

    # Set minimum and maximum HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])

    # Convert to HSV format and color threshold
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask=mask)

    # Print if there is a change in HSV value
    if ((phMin != hMin) | (psMin != sMin) | (pvMin != vMin) | (phMax != hMax) | (psMax != sMax) | (pvMax != vMax)):
        print("(hMin = %d , sMin = %d, vMin = %d), (hMax = %d , sMax = %d, vMax = %d)" % (
        hMin, sMin, vMin, hMax, sMax, vMax))
        phMin = hMin
        psMin = sMin
        pvMin = vMin
        phMax = hMax
        psMax = sMax
        pvMax = vMax

    # Display result image
    cv2.imshow('image', result)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
